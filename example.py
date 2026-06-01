"""Petoi Bittle X — MuJoCo Pose Demo"""
import mujoco
import mujoco.viewer
import numpy as np
from gaits import convert_pose, STAND_CTRL

# Poses to cycle through (3 seconds each)
POSE_SEQUENCE = ["balance", "sit", "buttUp", "str", "rest", "lifted", "balance"]
POSE_HOLD = 3.0       # seconds per pose
TRANSITION = 0.8      # seconds to blend between poses
WARMUP_STEPS = 300

model = mujoco.MjModel.from_xml_path("bittle.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)

# Convert all poses
poses = [convert_pose(name) for name in POSE_SEQUENCE]

pose_idx = 0
pose_timer = 0.0
step = 0
current_ctrl = STAND_CTRL.copy()

print(f"Pose: {POSE_SEQUENCE[0]}")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        dt = model.opt.timestep

        # Warmup: ramp from stand
        if step < WARMUP_STEPS:
            scale = step / WARMUP_STEPS
            data.ctrl[:] = STAND_CTRL + (poses[0] - STAND_CTRL) * scale
        else:
            pose_timer += dt
            if pose_timer >= POSE_HOLD and pose_idx < len(poses) - 1:
                pose_timer = 0.0
                pose_idx += 1
                print(f"Pose: {POSE_SEQUENCE[pose_idx]}")

            # Smooth blend to target pose
            target = poses[pose_idx]
            blend = min(1.0, pose_timer / TRANSITION)
            current_ctrl += (target - current_ctrl) * blend * dt * 5
            data.ctrl[:] = current_ctrl

        mujoco.mj_step(model, data)
        viewer.sync()
        step += 1
