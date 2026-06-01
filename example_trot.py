"""Petoi Bittle X — MuJoCo Trot Demo

Trot gait played open-loop from OpenCat data. Without an active balance
controller the motion won't look as natural as on real hardware.

For SNN-based autonomous locomotion with active balance, see:
https://github.com/MarcHesse/mhflocke
"""
import mujoco
import mujoco.viewer
import numpy as np
from gaits import convert_gait, STAND_CTRL

GAIT_NAME = "trF"    # trot forward
FRAME_DT = 0.020     # OpenCat: 50 Hz
WARMUP_STEPS = 400   # maturation ramp to prevent initial toppling
MAX_AMPLITUDE = 1.0  # full amplitude (proper masses provide stability)

model = mujoco.MjModel.from_xml_path("bittle.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)

gait = convert_gait(GAIT_NAME)
n_frames = len(gait)

frame_idx = 0
frame_timer = 0.0
step = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        frame_timer += model.opt.timestep
        if frame_timer >= FRAME_DT:
            frame_timer -= FRAME_DT
            scale = min(MAX_AMPLITUDE, step / WARMUP_STEPS)
            data.ctrl[:] = STAND_CTRL + (gait[frame_idx] - STAND_CTRL) * scale
            frame_idx = (frame_idx + 1) % n_frames

        mujoco.mj_step(model, data)
        viewer.sync()
        step += 1
