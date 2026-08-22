# Changelog

All notable changes to the Bittle X MuJoCo model are documented here.

## 2026-08-22

- **Joint limits widened to match the hardware's actual range of motion.** The
  shoulders were limited to `-1.5708 1.6` rad and the knees to `-1.22173 1.5708`,
  with a single actuator `ctrlrange` of `-1.57 1.57` applied to all eight joints —
  a flat default rather than a measurement, and one that clamped the knees tighter
  than their own joint range allowed. Video of a real Bittle X righting itself from
  its back shows all four legs swinging flat out to one side, well past the body
  edge, to lever the chassis over; those angles are far beyond ±90°. OpenCat's own
  fall-recovery skill `rc` commands up to -251° at a shoulder in its first two
  frames, so under the old limits the roll phase was clipped away entirely and the
  robot could not right itself in simulation. Shoulders are now `-2.6 2.6` rad
  (±149°) and knees `-1.22173 2.6`, with matching `ctrlrange`. The value is the
  measured threshold at which the recovery sequence succeeds: it fails at 2.2 and
  2.4 rad and works reliably from 2.6 upward. For reference, Petoi documents a
  joint range of ±125° and a 270° servo travel, while OpenCat's `angleLimit` table
  permits -200..80° — the new limit sits between the documented range and the
  firmware's software guard.
- Mass, geometry, inertials and gait data are unchanged. Standing and open-loop
  trot are unaffected: the gait tables never approach the old limits, so walking
  behaviour is identical.

## 2026-06-14

- **Battery inertial corrected.** The URDF-to-MJCF conversion had sign-flipped the
  LiPo battery's inertial height, placing its 55 g mass *above* the chassis. On the
  real Bittle X the pack clips *underneath*. The battery inertial is now mirrored
  (`pos z = -0.047784`), lowering the total center of mass by ~9.6 mm to match the
  hardware. Body position and geometry are unchanged; only the inertial reference
  moves. This improves open-loop trot stability and sim-to-real transfer.
- Added a video of the model driven by a spiking neural network (MH-FLOCKE):
  https://youtu.be/VC29z_WPgSU

## 2026-06-08

- **Mass properties validated** from direct measurement of a Bittle X V1 / BiBoard
  V0_1 unit. Total simulated mass 273.5 g (Petoi spec 265-290 g). Per-component
  masses assigned from the measured breakdown (battery, servos, legs, torso).
- Foot contact sites corrected to Y = 0.072 m from the shank body origin
  (previously 0.05 m).
- Joint damping tuned to 1.5 for a stable open-loop trot.

## Initial release

- MuJoCo (MJCF) model of the Petoi Bittle X V1 (plastic P1S servos), with visual
  meshes and OpenCat gait data (trot, walk, crawl, gallop, bound, pivot, and more)
  converted from OpenCat degrees to MuJoCo radians.
