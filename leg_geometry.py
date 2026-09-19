"""Leg geometry of the Bittle model in the stand pose, for comparison with
the real robot.

Real Bittle X (2026-09-19, measured by hand): shoulder servo horn 90 mm above
the floor in the stand pose (`kbalance`), ~80 mm while walking. The model
gives 79 mm. This script reports the numbers needed to find out why:
segment lengths (geometry) and segment angles (pose / angle conversion).

Prints per leg, relative to the shoulder joint axis (x forward, z up):
  - knee joint axis position
  - shank mesh tip (vertex farthest from the knee axis)
  - floor contact point after settling
  - thigh length (shoulder axis -> knee axis)
  - shank length (knee axis -> mesh tip)
  - thigh and shank angle from vertical (positive = pointing forward)

Also renders a side view of the stand pose to stand_side.png.

Usage (cmd.exe, repo root):
    py -3.11 leg_geometry.py
    py -3.11 leg_geometry.py --sweep     # knee offset -40..+40 deg
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from gaits import STAND_CTRL

HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE / "bittle.xml"
SETTLE_STEPS = 3000               # as in test_mass_stand.py

# leg -> (shoulder actuator, knee actuator, shank body)
LEGS = {
    "RF": (0, 1, "shank_rf_1"),
    "LF": (2, 3, "shank_lf_1"),
    "RR": (4, 5, "shank_rr_1"),
    "LR": (6, 7, "shank_lr_1"),
}


def angle_from_vertical(v: np.ndarray) -> float:
    """Angle of v (x, z) from straight down, degrees; + = pointing forward."""
    return math.degrees(math.atan2(v[0], -v[2]))


def shank_tip(model, data, body_name: str, knee: np.ndarray) -> np.ndarray:
    """World position of the shank mesh vertex farthest from the knee axis."""
    bid = model.body(body_name).id
    best, best_d = None, -1.0
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != bid or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        m = model.geom_dataid[g]
        a, n = model.mesh_vertadr[m], model.mesh_vertnum[m]
        verts = model.mesh_vert[a:a + n]
        R = data.geom_xmat[g].reshape(3, 3)
        world = data.geom_xpos[g] + verts @ R.T
        d = np.linalg.norm(world - knee, axis=1)
        i = int(np.argmax(d))
        if d[i] > best_d:
            best_d, best = float(d[i]), world[i].copy()
    return best


def settle(model, data, knee_offset_deg: float = 0.0) -> None:
    """Stand pose with an optional constant offset on all four knee commands."""
    mujoco.mj_resetDataKeyframe(model, data, 0)
    ctrl = np.asarray(STAND_CTRL, dtype=float).copy()
    ctrl[[1, 3, 5, 7]] += math.radians(knee_offset_deg)
    for _ in range(SETTLE_STEPS):
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)


def sweep(model, data) -> None:
    """Shoulder and knee height over a range of knee offsets (RF leg)."""
    mujoco.mj_forward(model, data)   # geom_xpos is zero until the first forward pass
    floor_z = float(data.geom_xpos[model.geom("floor").id][2])
    j_sh = int(model.actuator_trnid[0, 0])
    j_kn = int(model.actuator_trnid[1, 0])
    print("Knee offset sweep (RF). Real robot: shoulder 90 mm, knee 50 mm above floor.\n")
    print("offset[deg]  shoulder_h  knee_h  thigh_ang  shank_ang  tilt[deg]")
    for off in range(-40, 41, 5):
        settle(model, data, off)
        sh = np.array(data.xanchor[j_sh])
        kn = np.array(data.xanchor[j_kn])
        tip = shank_tip(model, data, "shank_rf_1", kn)
        up_z = 1 - 2 * (data.qpos[4] ** 2 + data.qpos[5] ** 2)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
        print(f"{off:>+10}  {(sh[2]-floor_z)*1000:>10.1f}  {(kn[2]-floor_z)*1000:>6.1f}  "
              f"{angle_from_vertical(kn-sh):>9.1f}  {angle_from_vertical(tip-kn):>9.1f}  "
              f"{tilt:>9.1f}")


def main() -> int:
    import sys
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    if "--sweep" in sys.argv:
        sweep(model, data)
        return 0
    settle(model, data, 0.0)

    floor_id = model.geom("floor").id
    floor_z = float(data.geom_xpos[floor_id][2])

    print(f"Stand pose after {SETTLE_STEPS} steps. Floor at z = {floor_z*1000:.1f} mm.")
    print("All positions relative to the shoulder joint axis, mm (x forward, z up).\n")
    print("Leg  shoulder_h  knee(x,z)        tip(x,z)         contact(x,z)     "
          "thigh_len  shank_len  thigh_ang  shank_ang  tip_above_floor")

    for leg, (a_sh, a_kn, shank) in LEGS.items():
        j_sh = int(model.actuator_trnid[a_sh, 0])
        j_kn = int(model.actuator_trnid[a_kn, 0])
        sh = np.array(data.xanchor[j_sh])
        kn = np.array(data.xanchor[j_kn])
        tip = shank_tip(model, data, shank, kn)

        shank_bid = model.body(shank).id
        cps = []
        for i in range(data.ncon):
            c = data.contact[i]
            if floor_id not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == floor_id else c.geom1
            if model.geom_bodyid[other] == shank_bid:
                cps.append(np.array(c.pos))
        contact = np.mean(cps, axis=0) if cps else None

        rel = lambda p: (p - sh) * 1000
        k, t = rel(kn), rel(tip)
        c_str = (f"({rel(contact)[0]:+6.1f},{rel(contact)[2]:+6.1f})"
                 if contact is not None else "     none     ")
        print(f"{leg:<4} {(sh[2]-floor_z)*1000:>9.1f}   "
              f"({k[0]:+6.1f},{k[2]:+6.1f})   ({t[0]:+6.1f},{t[2]:+6.1f})   {c_str}   "
              f"{np.linalg.norm(kn-sh)*1000:>8.1f}  {np.linalg.norm(tip-kn)*1000:>9.1f}  "
              f"{angle_from_vertical(kn-sh):>9.1f}  {angle_from_vertical(tip-kn):>9.1f}  "
              f"{(tip[2]-floor_z)*1000:>14.1f}")

    # Side view render.
    try:
        renderer = mujoco.Renderer(model, 480, 640)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = data.xpos[model.body("torso").id] + np.array([0, 0, -0.02])
        cam.distance = 0.32
        cam.azimuth = 90.0     # view along -y: right side of the robot
        cam.elevation = 0.0
        renderer.update_scene(data, camera=cam)
        img = renderer.render()
        out = HERE / "stand_side.png"
        try:
            from PIL import Image
            Image.fromarray(img).save(out)
        except ImportError:
            import matplotlib.pyplot as plt
            plt.imsave(out, img)
        print(f"\nSide view -> {out}")
    except Exception as exc:  # rendering is optional
        print(f"\nSide view not rendered: {exc}")

    print("\nOn the real robot, please measure (legs in kbalance, power on):")
    print("  thigh: shoulder horn centre -> knee horn centre")
    print("  shank: knee horn centre -> foot tip")
    print("  and a side photo to read the thigh and shank angles.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
