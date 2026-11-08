# Jump budget: G1 leg torque caps (Menagerie g1.xml, meshes stripped)
mujoco 3.3.7; numpy 2.0.2; input https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/98a70c569c4c1b6da467a20012f766ee1f8dc521/unitree_g1/g1.xml

(Printed by sim/jump_budget.py on 2026-11-07. Two edits since: the line above printed the run's local input path,
replaced here by the file's pinned URL, and this note. In the "knee cap at 0 rad/s" column, `unitree_curve` shows
131.0 because at zero joint speed the script uses Y2, the opposite-direction figure; the same-direction figure Y1 is
111.0, which is the "peak knee Nm" the curve runs reach during the push. "fell" is true when the pelvis tilt passes
60 degrees or the pelvis drops below 0.3 m at any time in the run, in the air included; see the README's limits.)

## base=planar (removed 61 mesh geoms, mass 33.341 kg, timestep 0.002 s, integrator 3)
collision geoms left: 4 cylinder (left_shoulder_pitch_link, left_shoulder_roll_link, right_shoulder_pitch_link, right_shoulder_roll_link); 1 plane (world); 8 sphere (left_ankle_roll_link, right_ankle_roll_link)
compiled leg caps N*m: left_ankle_pitch_joint 50, left_ankle_roll_joint 50, left_hip_pitch_joint 88, left_hip_roll_joint 139, left_hip_yaw_joint 88, left_knee_joint 139, right_ankle_pitch_joint 50, right_ankle_roll_joint 50, right_hip_pitch_joint 88, right_hip_roll_joint 139, right_hip_yaw_joint 88, right_knee_joint 139
compiled kv (biasprm[2]) knee: -15.847
a | caps | knee cap at 0 rad/s | held | liftoff | push ms | takeoff vz | ballistic rise cm | apex above stand cm | peak knee Nm | knee at cap % | peak knee rad/s | max tilt | fell
0.5 | spec90 | 90.0 | True | True | 56.0 | 1.817 | 16.84 | 15.69 | 90.0 | 100 | 21.5 | 99.5 | True
0.5 | spec120 | 120.0 | True | True | 46.0 | 1.922 | 18.82 | 16.69 | 120.0 | 100 | 25.44 | 101.1 | True
0.5 | unitree_curve | 131.0 | True | True | 62.0 | 1.514 | 11.68 | 11.51 | 111.0 | 100 | 21.05 | 98.5 | True
0.5 | rl_lab_flat | 139.0 | True | True | 52.0 | 1.935 | 19.09 | 17.64 | 139.0 | 100 | 36.58 | 97.9 | True
0.5 | menagerie139 | 139.0 | True | True | 42.0 | 2.041 | 21.23 | 18.78 | 139.0 | 100 | 27.06 | 100.6 | True
0.5 | legs300 | 300.0 | True | True | 32.0 | 3.045 | 47.25 | 43.49 | 300.0 | 100 | 43.65 | 155.1 | True
0.5 | g1cfg | 300.0 | True | True | 32.0 | 2.633 | 35.33 | 31.77 | 300.0 | 100 | 44.13 | 121.1 | True
0.5 | uncapped | 10000.0 | True | True | 14.0 | 5.589 | 159.19 | 152.61 | 3108.96 | 0 | 77.13 | 179.8 | True
0.8 | spec90 | 90.0 | True | True | 116.0 | 2.527 | 32.55 | 31.85 | 90.0 | 100 | 29.51 | 108.8 | True
0.8 | spec120 | 120.0 | True | True | 94.0 | 2.803 | 40.05 | 36.88 | 120.0 | 100 | 33.17 | 119.0 | True
0.8 | unitree_curve | 131.0 | True | True | 118.0 | 2.098 | 22.43 | 22.81 | 111.0 | 100 | 21.36 | 98.8 | True
0.8 | rl_lab_flat | 139.0 | True | True | 84.0 | 2.717 | 37.63 | 32.84 | 139.0 | 100 | 27.75 | 99.5 | True
0.8 | menagerie139 | 139.0 | True | True | 82.0 | 2.958 | 44.58 | 40.19 | 139.0 | 100 | 31.53 | 98.5 | True
0.8 | legs300 | 300.0 | True | True | 62.0 | 4.25 | 92.04 | 83.32 | 300.0 | 100 | 61.15 | 179.7 | True
0.8 | g1cfg | 300.0 | True | True | 58.0 | 4.17 | 88.62 | 82.03 | 300.0 | 100 | 39.19 | 179.8 | True
0.8 | uncapped | 10000.0 | True | True | 18.0 | 11.275 | 647.92 | 625.98 | 4968.13 | 0 | 98.07 | 179.9 | True

## base=free (removed 61 mesh geoms, mass 33.341 kg, timestep 0.002 s, integrator 3)
collision geoms left: 4 cylinder (left_shoulder_pitch_link, left_shoulder_roll_link, right_shoulder_pitch_link, right_shoulder_roll_link); 1 plane (world); 8 sphere (left_ankle_roll_link, right_ankle_roll_link)
compiled leg caps N*m: left_ankle_pitch_joint 50, left_ankle_roll_joint 50, left_hip_pitch_joint 88, left_hip_roll_joint 139, left_hip_yaw_joint 88, left_knee_joint 139, right_ankle_pitch_joint 50, right_ankle_roll_joint 50, right_hip_pitch_joint 88, right_hip_roll_joint 139, right_hip_yaw_joint 88, right_knee_joint 139
compiled kv (biasprm[2]) knee: -15.847
a | caps | knee cap at 0 rad/s | held | liftoff | push ms | takeoff vz | ballistic rise cm | apex above stand cm | peak knee Nm | knee at cap % | peak knee rad/s | max tilt | fell
0.5 | spec90 | 90.0 | True | True | 56.0 | 1.817 | 16.83 | 15.68 | 90.0 | 100 | 21.51 | 99.1 | True
0.5 | spec120 | 120.0 | True | True | 46.0 | 1.923 | 18.85 | 16.72 | 120.0 | 100 | 25.43 | 101.3 | True
0.5 | unitree_curve | 131.0 | True | True | 64.0 | 1.512 | 11.66 | 11.82 | 111.0 | 100 | 20.95 | 98.5 | True
0.5 | rl_lab_flat | 139.0 | True | True | 52.0 | 1.935 | 19.09 | 17.64 | 139.0 | 100 | 36.59 | 97.9 | True
0.5 | menagerie139 | 139.0 | True | True | 42.0 | 2.041 | 21.23 | 18.77 | 139.0 | 100 | 27.07 | 100.6 | True
0.5 | legs300 | 300.0 | True | True | 32.0 | 3.045 | 47.25 | 43.49 | 300.0 | 100 | 43.67 | 155.0 | True
0.5 | g1cfg | 300.0 | True | True | 30.0 | 2.832 | 40.87 | 37.63 | 300.0 | 100 | 32.82 | 143.7 | True
0.5 | uncapped | 10000.0 | True | True | 14.0 | 5.586 | 159.06 | 152.49 | 3109.02 | 0 | 77.14 | 179.7 | True
0.8 | spec90 | 90.0 | True | True | 116.0 | 2.53 | 32.61 | 31.95 | 90.0 | 100 | 29.4 | 107.8 | True
0.8 | spec120 | 120.0 | True | True | 98.0 | 2.804 | 40.08 | 37.49 | 120.0 | 100 | 39.37 | 113.8 | True
0.8 | unitree_curve | 131.0 | True | True | 110.0 | 2.153 | 23.62 | 22.39 | 111.0 | 100 | 21.07 | 101.1 | True
0.8 | rl_lab_flat | 139.0 | True | True | 84.0 | 2.717 | 37.64 | 32.84 | 139.0 | 100 | 27.75 | 99.6 | True
0.8 | menagerie139 | 139.0 | True | True | 82.0 | 2.958 | 44.59 | 40.19 | 139.0 | 100 | 31.54 | 98.5 | True
0.8 | legs300 | 300.0 | True | True | 62.0 | 4.25 | 92.04 | 83.31 | 300.0 | 100 | 61.16 | 179.8 | True
0.8 | g1cfg | 300.0 | True | True | 58.0 | 4.17 | 88.63 | 82.04 | 300.0 | 100 | 39.2 | 179.8 | True
0.8 | uncapped | 10000.0 | True | True | 18.0 | 11.272 | 647.6 | 625.68 | 4968.26 | 0 | 98.09 | 179.8 | True

## free base, CoM apex in flight above standing (cm), and ratios to unitree_curve
a=0.5: {"spec90": 15.68, "spec120": 16.72, "unitree_curve": 11.82, "rl_lab_flat": 17.64, "menagerie139": 18.77, "legs300": 43.49, "g1cfg": 37.63, "uncapped": 152.49}
a=0.5 ratios: {"spec90": 1.33, "spec120": 1.41, "unitree_curve": 1.0, "rl_lab_flat": 1.49, "menagerie139": 1.59, "legs300": 3.68, "g1cfg": 3.18, "uncapped": 12.9}
a=0.8: {"spec90": 31.95, "spec120": 37.49, "unitree_curve": 22.39, "rl_lab_flat": 32.84, "menagerie139": 40.19, "legs300": 83.31, "g1cfg": 82.04, "uncapped": 625.68}
a=0.8 ratios: {"spec90": 1.43, "spec120": 1.67, "unitree_curve": 1.0, "rl_lab_flat": 1.47, "menagerie139": 1.79, "legs300": 3.72, "g1cfg": 3.66, "uncapped": 27.94}

video.json: 3 parts, 33.4 s, 404 narration characters; apex cm used: {"unitree_curve": 22, "rl_lab_flat": 33, "g1cfg": 82}
