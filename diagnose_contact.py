"""Contact & actuation diagnostic for the Bittle X model.

Plays a gait open-loop exactly like example_trot.py and measures stance
slip, contact location, joint tracking, actuator torque and body motion.
Design: docs/CONTACT_DIAGNOSTIC.md

The model file is never modified. --damping/--armature/--kp/--forcerange/
--friction change the loaded model in memory only and are recorded in
every output.

Usage (cmd.exe, from the repo root):
    py -3.11 diagnose_contact.py --gait trF
    py -3.11 diagnose_contact.py --gait wkF --json out\\wkF_baseline.json
    py -3.11 diagnose_contact.py --gait stand
    py -3.11 diagnose_contact.py --gait trF --friction 0.05
    py -3.11 diagnose_contact.py --selfcheck
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

from gaits import STAND_CTRL, convert_gait

HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE / "bittle.xml"

FRAME_DT = 0.020          # OpenCat 50 Hz, as in example_trot.py
WARMUP_STEPS = 400        # amplitude ramp, as in example_trot.py
STAND_CYCLE_S = 1.0       # "cycle" length for the stand scenario
FALL_TILT_DEG = 60.0      # torso tilt beyond this counts as a fall
FORCE_EPS = 1e-6          # N, contact counts as loaded above this
LOAD_FRAC = 0.10          # leg carries >= 10 % of body weight -> 'loaded stance'
MAX_LAG_S = 0.2           # tracking lag search window

LEGS = ("RF", "LF", "RR", "LR")
LEG_TAGS = {"rf": "RF", "lf": "LF", "rr": "RR", "lr": "LR"}
FOOT_SITES = {"RF": "rf_foot_site", "LF": "lf_foot_site",
              "RR": "rr_foot_site", "LR": "lr_foot_site"}


# ---------------------------------------------------------------------------
# Model setup
# ---------------------------------------------------------------------------

@dataclass
class Overrides:
    damping: Optional[float] = None
    armature: Optional[float] = None
    kp: Optional[float] = None
    forcerange: Optional[float] = None
    friction: Optional[float] = None
    knee_offset_deg: Optional[float] = None   # control, not model: added to all knee commands
    impratio: Optional[float] = None
    noslip: Optional[int] = None
    contact_timeconst: Optional[float] = None   # geom solref[0]; larger = softer contact

    def active(self) -> dict:
        return {k: v for k, v in self.__dict__.items() if v is not None}


def load_model(ov: Overrides) -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    act_joints = [int(model.actuator_trnid[a, 0]) for a in range(model.nu)]
    dofs = [int(model.jnt_dofadr[j]) for j in act_joints]
    if ov.damping is not None:
        model.dof_damping[dofs] = ov.damping
    if ov.armature is not None:
        model.dof_armature[dofs] = ov.armature
    if ov.kp is not None:
        model.actuator_gainprm[:, 0] = ov.kp
        model.actuator_biasprm[:, 1] = -ov.kp
    if ov.forcerange is not None:
        model.actuator_forcerange[:, 0] = -ov.forcerange
        model.actuator_forcerange[:, 1] = ov.forcerange
        model.actuator_forcelimited[:] = 1
    if ov.friction is not None:
        for g in range(model.ngeom):
            if model.geom_contype[g] or model.geom_conaffinity[g]:
                model.geom_friction[g, 0] = ov.friction
    if ov.impratio is not None:
        model.opt.impratio = ov.impratio
    if ov.noslip is not None:
        model.opt.noslip_iterations = ov.noslip
    if ov.contact_timeconst is not None:
        for g in range(model.ngeom):
            if model.geom_contype[g] or model.geom_conaffinity[g]:
                model.geom_solref[g, 0] = ov.contact_timeconst
    return model


def leg_of_body(model: mujoco.MjModel, body: int) -> str:
    """First ancestor body whose name carries exactly one leg tag."""
    while body > 0:
        name = model.body(body).name.lower()
        hits = [leg for tag, leg in LEG_TAGS.items() if tag in name]
        if len(hits) == 1:
            return hits[0]
        body = int(model.body_parentid[body])
    return "body"


def is_lower_leg_body(model: mujoco.MjModel, body: int) -> bool:
    name = model.body(body).name.lower()
    return name.startswith("servos_") or name.startswith("shank_")


def model_hash() -> str:
    return hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()[:16]


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

@dataclass
class LegAcc:
    stance_steps: int = 0
    slip_distance: float = 0.0
    slip_speeds: list = field(default_factory=list)   # per stance step
    geom_steps: dict = field(default_factory=dict)    # geom name -> steps
    site_dist_w: float = 0.0                          # force-weighted sum
    site_dist_f: float = 0.0
    site_dist_max: float = 0.0
    site_off_w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    contact_w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    loaded_steps: int = 0
    loaded_slip: float = 0.0     # slip while the leg carries real load
    loaded_fwd: float = 0.0      # |forward component| of loaded slip, body heading frame
    loaded_lat: float = 0.0      # |lateral component| of loaded slip
    scuff: float = 0.0           # slip during light contact (touchdown, lift-off, toe drag)


def run(
    gait_name: str,
    cycles: int,
    ov: Overrides,
    servo_torque_nm: Optional[float] = None,
    csv_path: Optional[Path] = None,
) -> dict:
    model = load_model(ov)
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    if gait_name == "stand":
        gait = np.asarray(STAND_CTRL, dtype=float).reshape(1, -1)
        cycle_s = STAND_CYCLE_S
    else:
        gait = np.asarray(convert_gait(gait_name), dtype=float)
        cycle_s = len(gait) * FRAME_DT
    n_frames = len(gait)
    stand = np.asarray(STAND_CTRL, dtype=float).copy()
    if ov.knee_offset_deg is not None:
        knees = [1, 3, 5, 7]
        off = math.radians(ov.knee_offset_deg)
        gait = gait.copy()
        gait[:, knees] += off
        stand[knees] += off

    steps_per_cycle = int(math.ceil(cycle_s / dt))
    settle_steps = WARMUP_STEPS + steps_per_cycle
    measure_steps = cycles * steps_per_cycle
    total_steps = settle_steps + measure_steps

    floor_id = model.geom("floor").id
    geom_leg = {}
    geom_lower = {}
    for g in range(model.ngeom):
        b = int(model.geom_bodyid[g])
        geom_leg[g] = leg_of_body(model, b)
        geom_lower[g] = is_lower_leg_body(model, b)
    site_ids = {leg: model.site(s).id for leg, s in FOOT_SITES.items()}

    nu = model.nu
    act_names = [model.actuator(a).name for a in range(nu)]
    act_qpos = [int(model.jnt_qposadr[int(model.actuator_trnid[a, 0])])
                for a in range(nu)]

    ctrl_log = np.zeros((measure_steps, nu))
    qpos_log = np.zeros((measure_steps, nu))
    force_log = np.zeros((measure_steps, nu))
    body_log = np.zeros((measure_steps, 5))       # x, y, z, yaw (unwrapped), up_z
    fz_log = np.zeros(measure_steps)

    legs = {leg: LegAcc() for leg in LEGS}
    weight_n = float(np.sum(model.body_mass)) * float(-model.opt.gravity[2])
    load_n = LOAD_FRAC * weight_n
    foreign = {}                                   # non-lower-leg geoms in contact
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    f6 = np.zeros(6)
    csv_rows = [] if csv_path else None

    frame_idx = 0
    frame_timer = 0.0
    yaw_prev = None
    yaw_unwrapped = 0.0

    for step in range(total_steps):
        # --- control, identical to example_trot.py ---
        frame_timer += dt
        if frame_timer >= FRAME_DT:
            frame_timer -= FRAME_DT
            scale = min(1.0, step / WARMUP_STEPS)
            data.ctrl[:] = stand + (gait[frame_idx] - stand) * scale
            frame_idx = (frame_idx + 1) % n_frames

        mujoco.mj_step(model, data)

        k = step - settle_steps
        if k < 0:
            continue

        # --- body ---
        x, y, z = data.qpos[0:3]
        w, qx, qy, qz = data.qpos[3:7]
        yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        if yaw_prev is not None:
            d = yaw - yaw_prev
            d = (d + math.pi) % (2 * math.pi) - math.pi
            yaw_unwrapped += d
        yaw_prev = yaw
        up_z = 1 - 2 * (qx * qx + qy * qy)        # z of torso up-axis in world
        body_log[k] = (x, y, z, yaw_unwrapped, up_z)

        # --- actuators ---
        ctrl_log[k] = data.ctrl[:nu]
        qpos_log[k] = data.qpos[act_qpos]
        force_log[k] = data.actuator_force[:nu]

        # --- contacts ---
        leg_f = {leg: 0.0 for leg in LEGS}
        leg_fv = {leg: 0.0 for leg in LEGS}
        leg_fvec = {leg: np.zeros(3) for leg in LEGS}
        leg_geoms = {leg: set() for leg in LEGS}
        fz_total = 0.0
        for i in range(data.ncon):
            c = data.contact[i]
            if floor_id not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == floor_id else c.geom1
            mujoco.mj_contactForce(model, data, i, f6)
            fn = abs(float(f6[0]))
            frame = np.asarray(c.frame).reshape(3, 3)
            f_world = frame.T @ f6[:3]
            fz_total += float(f_world[2])

            leg = geom_leg[other]
            # Geoms in bittle.xml are unnamed; the owning body identifies them.
            gname = model.body(int(model.geom_bodyid[other])).name
            if not geom_lower[other]:
                foreign[gname] = foreign.get(gname, 0) + 1
            if leg not in legs or fn <= FORCE_EPS:
                continue

            p = np.asarray(c.pos)
            n = frame[0]
            mujoco.mj_jac(model, data, jacp, jacr, p, int(model.geom_bodyid[other]))
            v = jacp @ data.qvel
            vt = v - np.dot(v, n) * n
            speed = float(np.linalg.norm(vt))

            leg_f[leg] += fn
            leg_fv[leg] += fn * speed
            leg_fvec[leg] += fn * vt
            leg_geoms[leg].add(gname)

            dist = float(np.linalg.norm(p - data.site_xpos[site_ids[leg]]))
            acc = legs[leg]
            acc.site_off_w += fn * (p - data.site_xpos[site_ids[leg]])
            acc.contact_w += fn * p
            acc.site_dist_w += fn * dist
            acc.site_dist_f += fn
            acc.site_dist_max = max(acc.site_dist_max, dist)

        fz_log[k] = abs(fz_total)
        for leg in LEGS:
            acc = legs[leg]
            if leg_f[leg] > FORCE_EPS:
                s = leg_fv[leg] / leg_f[leg]
                acc.stance_steps += 1
                acc.slip_distance += s * dt
                acc.slip_speeds.append(s)
                if leg_f[leg] >= load_n:
                    acc.loaded_steps += 1
                    acc.loaded_slip += s * dt
                    v = leg_fvec[leg] / leg_f[leg]
                    c, sn = math.cos(yaw), math.sin(yaw)
                    acc.loaded_fwd += abs(c * v[0] + sn * v[1]) * dt
                    acc.loaded_lat += abs(-sn * v[0] + c * v[1]) * dt
                else:
                    acc.scuff += s * dt
                for g in leg_geoms[leg]:
                    acc.geom_steps[g] = acc.geom_steps.get(g, 0) + 1

        if csv_rows is not None:
            row = [k * dt, x, y, z, yaw_unwrapped, abs(fz_total)]
            for leg in LEGS:
                row += [leg_f[leg],
                        (leg_fv[leg] / leg_f[leg]) if leg_f[leg] > FORCE_EPS else 0.0]
            row += list(data.ctrl[:nu]) + list(data.qpos[act_qpos]) \
                + list(data.actuator_force[:nu])
            csv_rows.append(row)

    # ---------------------------------------------------------------- summary
    duration = measure_steps * dt
    travel_x = float(body_log[-1, 0] - body_log[0, 0])
    drift_y = float(body_log[-1, 1] - body_log[0, 1])
    yaw_drift = math.degrees(float(body_log[-1, 3] - body_log[0, 3]))
    weight = float(np.sum(model.body_mass)) * float(-model.opt.gravity[2])
    tilt_max = math.degrees(math.acos(float(np.clip(body_log[:, 4].min(), -1.0, 1.0))))

    leg_summary = {}
    shoulder_act = {"RF": 0, "LF": 2, "RR": 4, "LR": 6}
    for leg in LEGS:
        acc = legs[leg]
        jid = int(model.actuator_trnid[shoulder_act[leg], 0])
        anchor = np.array(data.xanchor[jid])
        site_rel = (np.array(data.site_xpos[site_ids[leg]]) - anchor) * 1000
        contact_rel = ((acc.contact_w / acc.site_dist_f - anchor) * 1000
                       if acc.site_dist_f > 0 else None)
        speeds = np.asarray(acc.slip_speeds) if acc.slip_speeds else np.zeros(1)
        ratio = (acc.slip_distance / abs(travel_x)) if abs(travel_x) > 1e-3 else None
        total = max(acc.stance_steps, 1)
        leg_summary[leg] = {
            "stance_share": acc.stance_steps / measure_steps,
            "slip_distance_mm": acc.slip_distance * 1000,
            "slip_ratio": ratio,
            "loaded_share": acc.loaded_steps / measure_steps,
            "loaded_slip_mm": acc.loaded_slip * 1000,
            "loaded_slip_ratio": (acc.loaded_slip / abs(travel_x))
            if abs(travel_x) > 1e-3 else None,
            "scuff_mm": acc.scuff * 1000,
            "loaded_fwd_mm": acc.loaded_fwd * 1000,
            "loaded_lat_mm": acc.loaded_lat * 1000,
            "slip_speed_mean_mm_s": float(speeds.mean()) * 1000,
            "slip_speed_p95_mm_s": float(np.percentile(speeds, 95)) * 1000,
            "contact_geoms": {g: n / total for g, n in
                              sorted(acc.geom_steps.items(), key=lambda t: -t[1])},
            "site_dist_mean_mm": (acc.site_dist_w / acc.site_dist_f * 1000)
            if acc.site_dist_f > 0 else None,
            "site_dist_max_mm": acc.site_dist_max * 1000,
            "contact_minus_site_mm": (acc.site_off_w / acc.site_dist_f * 1000).tolist()
            if acc.site_dist_f > 0 else None,
            "contact_rel_shoulder_mm": contact_rel.tolist() if contact_rel is not None else None,
            "site_rel_shoulder_end_mm": site_rel.tolist(),
        }

    max_lag = int(MAX_LAG_S / dt)
    act_summary = {}
    for a in range(nu):
        err = ctrl_log[:, a] - qpos_log[:, a]
        best_lag, best_rms = 0, float(np.sqrt(np.mean(err ** 2)))
        for lag in range(1, max_lag + 1):
            e = ctrl_log[:-lag, a] - qpos_log[lag:, a]
            r = float(np.sqrt(np.mean(e ** 2)))
            if r < best_rms:
                best_lag, best_rms = lag, r
        tau = np.abs(force_log[:, a])
        entry = {
            "track_rms_deg": math.degrees(float(np.sqrt(np.mean(err ** 2)))),
            "track_max_deg": math.degrees(float(np.max(np.abs(err)))),
            "lag_ms": best_lag * dt * 1000,
            "track_rms_after_lag_deg": math.degrees(best_rms),
            "torque_peak_nm": float(tau.max()),
            "torque_rms_nm": float(np.sqrt(np.mean(tau ** 2))),
        }
        if servo_torque_nm is not None:
            entry["share_above_ref"] = float(np.mean(tau > servo_torque_nm))
        act_summary[act_names[a]] = entry

    result = {
        "gait": gait_name,
        "cycles": cycles,
        "cycle_s": cycle_s,
        "duration_s": duration,
        "timestep_s": dt,
        "model_file": MODEL_PATH.name,
        "model_sha256_16": model_hash(),
        "mujoco_version": mujoco.__version__,
        "overrides": ov.active(),
        "servo_torque_ref_nm": servo_torque_nm,
        "body": {
            "travel_x_mm": travel_x * 1000,
            "travel_x_per_cycle_mm": travel_x * 1000 / cycles,
            "speed_mm_s": travel_x * 1000 / duration,
            "drift_y_mm": drift_y * 1000,
            "yaw_drift_deg": yaw_drift,
            "height_mean_mm": float(body_log[:, 2].mean()) * 1000,
            "height_min_mm": float(body_log[:, 2].min()) * 1000,
            "tilt_max_deg": tilt_max,
            "fell": bool(tilt_max > FALL_TILT_DEG),
            "fz_mean_n": float(fz_log.mean()),
            "weight_n": weight,
            "fz_over_weight": float(fz_log.mean()) / weight,
        },
        "foreign_contacts": foreign,
        "legs": leg_summary,
        "actuators": act_summary,
    }

    if csv_rows is not None:
        header = ["t", "x", "y", "z", "yaw", "fz_total"]
        for leg in LEGS:
            header += [f"{leg}_fn", f"{leg}_slip"]
        header += [f"ctrl_{n}" for n in act_names] \
            + [f"qpos_{n}" for n in act_names] \
            + [f"tau_{n}" for n in act_names]
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(csv_rows)

    return result


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def _fmt(v, spec=".1f", none="-"):
    return none if v is None else format(v, spec)


def print_summary(r: dict) -> None:
    b = r["body"]
    print(f"\n=== {r['gait']}  {r['cycles']} cycles ({r['duration_s']:.1f} s)  "
          f"model {r['model_sha256_16']}  MuJoCo {r['mujoco_version']}")
    if r["overrides"]:
        print(f"    OVERRIDES: {r['overrides']}  (model in memory only)")
    print(f"\nBody: travel {b['travel_x_mm']:.1f} mm "
          f"({b['travel_x_per_cycle_mm']:.1f} mm/cycle, {b['speed_mm_s']:.1f} mm/s)  "
          f"drift y {b['drift_y_mm']:.1f} mm  yaw {b['yaw_drift_deg']:.1f} deg")
    print(f"      height mean {b['height_mean_mm']:.1f} mm  min {b['height_min_mm']:.1f} mm"
          f"  tilt max {b['tilt_max_deg']:.1f} deg  fell={b['fell']}  "
          f"Fz/weight {b['fz_over_weight']:.3f}")

    print("\nLeg  stance  slip[mm]  ratio  v_mean  v_p95[mm/s]  site_d mean/max[mm]  contact geoms")
    for leg, s in r["legs"].items():
        geoms = ", ".join(f"{g} {share:.0%}" for g, share in s["contact_geoms"].items())
        print(f"{leg:<4} {s['stance_share']:>5.0%}  {s['slip_distance_mm']:>8.1f}  "
              f"{_fmt(s['slip_ratio'], '.2f'):>5}  {s['slip_speed_mean_mm_s']:>6.1f}  "
              f"{s['slip_speed_p95_mm_s']:>11.1f}  "
              f"{_fmt(s['site_dist_mean_mm']):>8} / {s['site_dist_max_mm']:<8.1f}  {geoms}")
    print(f"\nLoaded stance = leg carries >= {LOAD_FRAC:.0%} of body weight")
    print("Leg  loaded  slip_loaded[mm]  ratio_loaded  scuff[mm]  fwd[mm]  lat[mm]")
    for leg, s in r["legs"].items():
        print(f"{leg:<4} {s['loaded_share']:>6.0%}  {s['loaded_slip_mm']:>15.1f}  "
              f"{_fmt(s['loaded_slip_ratio'], '.2f'):>12}  {s['scuff_mm']:>9.1f}  "
              f"{s['loaded_fwd_mm']:>7.1f}  {s['loaded_lat_mm']:>7.1f}")
    if r["gait"] == "stand":
        print("\nContact point minus foot site, world frame (x fwd, y left, z up):")
        for leg, s in r["legs"].items():
            v = s["contact_minus_site_mm"]
            if v is not None:
                print(f"  {leg}: dx {v[0]:+.1f}  dy {v[1]:+.1f}  dz {v[2]:+.1f} mm")
        print("\nRelative to the shoulder joint axis (x fwd, z up), for comparison with the real robot:")
        for leg, s in r["legs"].items():
            c = s["contact_rel_shoulder_mm"]
            st = s["site_rel_shoulder_end_mm"]
            if c is not None:
                print(f"  {leg}: contact x {c[0]:+.1f} z {c[2]:+.1f} mm   "
                      f"foot site x {st[0]:+.1f} z {st[2]:+.1f} mm")
    if r["foreign_contacts"]:
        print(f"\nContacts outside lower legs (steps): {r['foreign_contacts']}")

    print("\nActuator              rms[deg]  max[deg]  lag[ms]  rms@lag  tau_peak  tau_rms[Nm]")
    for name, a in r["actuators"].items():
        extra = f"  >ref {a['share_above_ref']:.0%}" if "share_above_ref" in a else ""
        print(f"{name:<20} {a['track_rms_deg']:>9.2f} {a['track_max_deg']:>9.2f} "
              f"{a['lag_ms']:>8.0f} {a['track_rms_after_lag_deg']:>8.2f} "
              f"{a['torque_peak_nm']:>9.3f} {a['torque_rms_nm']:>9.3f}{extra}")


def mean_slip_ratio(r: dict) -> Optional[float]:
    vals = [s["slip_ratio"] for s in r["legs"].values() if s["slip_ratio"] is not None]
    return float(np.mean(vals)) if vals else None


# ---------------------------------------------------------------------------
# Self-check (docs/CONTACT_DIAGNOSTIC.md, "Validation of the diagnostic")
# ---------------------------------------------------------------------------

def selfcheck() -> bool:
    ok_all = True

    def report(name: str, ok: bool, detail: str) -> None:
        nonlocal ok_all
        ok_all &= ok
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    print("Self-check 1: stand scenario")
    r = run("stand", 3, Overrides())
    b = r["body"]
    report("force balance", abs(b["fz_over_weight"] - 1.0) <= 0.02,
           f"Fz/weight = {b['fz_over_weight']:.4f} (tolerance 2 %)")
    p95 = max(s["slip_speed_p95_mm_s"] for s in r["legs"].values())
    report("slip near zero", p95 < 1.0, f"max p95 slip speed = {p95:.3f} mm/s (limit 1.0)")
    report("only lower-leg contacts", not r["foreign_contacts"],
           f"foreign = {r['foreign_contacts'] or 'none'}")
    report("standing", not b["fell"], f"min height {b['height_min_mm']:.1f} mm")

    print("\nSelf-check 2: deliberate slip (trF, friction 0.05 vs default)")
    base = run("trF", 5, Overrides())
    low = run("trF", 5, Overrides(friction=0.05))
    rb, rl = mean_slip_ratio(base), mean_slip_ratio(low)
    sb = sum(s["slip_distance_mm"] for s in base["legs"].values())
    sl = sum(s["slip_distance_mm"] for s in low["legs"].values())
    report("slip distance rises", sl > 1.5 * sb,
           f"total slip {sb:.1f} mm -> {sl:.1f} mm (need > 1.5x); "
           f"mean ratio {_fmt(rb, '.2f')} -> {_fmt(rl, '.2f')}")

    print("\nSelf-check 3: determinism (trF, 2 cycles, twice)")
    a = run("trF", 2, Overrides())
    c = run("trF", 2, Overrides())
    same = json.dumps(a, sort_keys=True) == json.dumps(c, sort_keys=True)
    report("identical results", same, "two runs " + ("match" if same else "DIFFER"))

    print(f"\nSELF-CHECK: {'PASS' if ok_all else 'FAIL'}")
    return ok_all


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def view(gait_name: str, ov: Overrides) -> None:
    """Play the gait in the interactive viewer, real time, no measurement."""
    import time
    import mujoco.viewer

    model = load_model(ov)
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    if gait_name == "stand":
        gait = np.asarray(STAND_CTRL, dtype=float).reshape(1, -1)
    else:
        gait = np.asarray(convert_gait(gait_name), dtype=float)
    stand = np.asarray(STAND_CTRL, dtype=float).copy()
    if ov.knee_offset_deg is not None:
        knees = [1, 3, 5, 7]
        off = math.radians(ov.knee_offset_deg)
        gait = gait.copy()
        gait[:, knees] += off
        stand[knees] += off

    print(f"[view] {gait_name}  overrides={ov.active() or 'none'}  (close window to end)")
    frame_idx, frame_timer, step = 0, 0.0, 0
    with mujoco.viewer.launch_passive(model, data) as v:
        t0 = time.perf_counter()
        while v.is_running():
            frame_timer += dt
            if frame_timer >= FRAME_DT:
                frame_timer -= FRAME_DT
                scale = min(1.0, step / WARMUP_STEPS)
                data.ctrl[:] = stand + (gait[frame_idx] - stand) * scale
                frame_idx = (frame_idx + 1) % len(gait)
            mujoco.mj_step(model, data)
            step += 1
            if step % 8 == 0:
                v.sync()
                lag = step * dt - (time.perf_counter() - t0)
                if lag > 0:
                    time.sleep(lag)


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--gait", default="trF",
                   help="Gait key from gaits.py (trF, wkF, ...) or 'stand'.")
    p.add_argument("--cycles", type=int, default=10)
    p.add_argument("--json", type=Path, help="Write summary JSON here.")
    p.add_argument("--csv", type=Path, help="Write per-step CSV here.")
    p.add_argument("--servo-torque-nm", type=float,
                   help="Reference torque from datasheet/measurement (no default).")
    p.add_argument("--damping", type=float)
    p.add_argument("--armature", type=float)
    p.add_argument("--kp", type=float)
    p.add_argument("--forcerange", type=float)
    p.add_argument("--friction", type=float)
    p.add_argument("--knee-offset-deg", type=float,
                   help="Constant offset on all knee commands (tests angle conversion).")
    p.add_argument("--impratio", type=float,
                   help="model.opt.impratio (frictional/normal impedance, default 1).")
    p.add_argument("--noslip", type=int,
                   help="model.opt.noslip_iterations (post-solver slip suppression, default 0).")
    p.add_argument("--contact-timeconst", type=float,
                   help="geom solref time constant for all collision geoms (default 0.02 s; "
                        "larger = softer, rubber-like contact).")
    p.add_argument("--selfcheck", action="store_true",
                   help="Run the diagnostic's own validation checks.")
    p.add_argument("--view", action="store_true",
                   help="Show the gait in the MuJoCo viewer instead of measuring.")
    args = p.parse_args(argv)

    if args.selfcheck:
        return 0 if selfcheck() else 1

    if args.cycles <= 0:
        p.error("--cycles must be positive")

    ov = Overrides(args.damping, args.armature, args.kp,
                   args.forcerange, args.friction, args.knee_offset_deg,
                   args.impratio, args.noslip, args.contact_timeconst)
    if args.view:
        view(args.gait, ov)
        return 0
    r = run(args.gait, args.cycles, ov, args.servo_torque_nm, args.csv)
    print_summary(r)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(r, indent=2))
        print(f"\nJSON -> {args.json}")
    if args.csv:
        print(f"CSV  -> {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
