"""Quick validation: mass + stand test for bittle.xml
Usage: py -3.11 test_mass_stand.py
"""
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path('bittle.xml')
d = mujoco.MjData(m)

total_g = round(sum(m.body_mass) * 1000, 1)
print(f"Mass:     {total_g} g  ({'OK' if abs(total_g - 273.5) < 1.0 else 'WRONG'})")

mujoco.mj_resetDataKeyframe(m, d, 0)
ctrl = [-0.7854, 1.4835, -0.7854, 1.4835, 0.7854, 1.4835, 0.7854, 1.4835]
for _ in range(3000):
    d.ctrl[:] = ctrl
    mujoco.mj_step(m, d)

h = d.qpos[2]
print(f"Height:   {h*1000:.1f} mm  ({'STANDING' if h > 0.04 else 'FALLEN'})")
print(f"Contacts: {d.ncon}")

for name in ['rf_foot_site', 'lf_foot_site', 'rr_foot_site', 'lr_foot_site']:
    p = d.site_xpos[m.site(name).id]
    print(f"  {name}: z={p[2]*1000:.1f} mm")

ok = abs(total_g - 273.5) < 1.0 and h > 0.04
print()
print("RESULT:", "PASS" if ok else "FAIL")
