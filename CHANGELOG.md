# Changelog

All notable changes to the Bittle X MuJoCo model are documented here.

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
