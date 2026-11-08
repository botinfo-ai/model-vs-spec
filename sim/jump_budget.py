#!/usr/bin/env python3
"""Jump budget: how high can MuJoCo Menagerie's G1 model jump from a crouch under each set of leg torque caps?

Usage: python3 jump_budget.py <inputs folder holding unitree_g1/g1.xml> <output folder>. Needs numpy and mujoco only.

Why: torque caps only separate when the joints are driven to their limits, and a max-effort push from a crouch does
that. The caps compared, all on the same model (Menagerie's g1.xml is g1_29dof_rev_1_0) and the same controller:
  spec90 / spec120   knee 90 / 120 N*m (unitree.com/g1: G1 / EDU), the rest as Menagerie compiles it
  menagerie139       as compiled (knee and hip roll 139, hip pitch/yaw 88, ankles 50)
  rl_lab_flat        unitree_rl_lab @4e08d96 unitree.py, UNITREE_G1_29DOF_CFG (the 29-DoF config, matching this
                     model): hip pitch/yaw 88 ("N7520-14.3"), hip roll/knee 139 ("N7520-22.5"), ankles 25 ("N5020-16")
  unitree_curve      the same three groups clipped by the torque-speed figures for those motors in
                     unitree_actuators.py @4e08d96 (N7520-22.5: Y1 111, Y2 131, X1 14.5, X2 22.7; N7520-14.3: Y1 71,
                     Y2 83.3, X1 22.63, X2 35.52; N5020-16: Y1 24.8, Y2 31.9, X1 30.86, X2 40.13), with that file's
                     own clip rule (_clip_effort and _compute_effort_limit, L0075-0088): Y1 when the commanded effort
                     and the joint speed point the same way, otherwise Y2; full below X1, then linear to 0 at X2
  legs300            hips and knees 300 (Isaac Lab @76aeb42 G1_CFG legs group), ankles as compiled (50)
  g1cfg              legs300 with the ankles at 20 (G1_CFG feet group)
  uncapped           legs at 1e4: what this controller does with no cap at all
Not modelled: velocity_limit_sim (the joint speed cap in the Isaac Lab configs) and the friction terms (Fs, Fd) in
unitree_actuators.py.

Model: unitree_g1/g1.xml with its mesh geoms and mesh files removed, keyframes removed, a ground plane added. Every
geom that can still collide is printed by type and body. Bases: "planar" (pelvis free
joint replaced by x slide, z slide and pitch hinge: a sagittal jump that cannot fall sideways) and "free" (the
original free joint).

Controller: the leg actuators stay MuJoCo position actuators (implicit damping); this script only changes their
gains and sets each actuator's forcerange every step (that is how the speed-dependent curve is applied). The effort
direction used to pick Y1 or Y2 is the PD effort the actuator would apply this step before clipping.
Crouch: hip pitch -a, knee 2a, ankle pitch -a (feet parallel to the pelvis), held HOLD_S at kp 500. Push: targets go
to 0 with kp 3000 on hip pitch and knee and 1500 on ankle pitch, so every cap below 300 N*m is saturated until the
joint is within ~0.1 rad of straight (a max-effort push, not an optimal jump). After liftoff the gains drop back to
500. Measured: whether and when the feet leave the ground, the whole-body CoM vertical speed at liftoff and its
ballistic rise v^2/2g, the CoM apex above the straight-leg standing CoM, peak knee torque and speed, the share of
push steps the knee sat at its cap, and the pelvis tilt.

Writes to the output folder: jump_budget.md (the printed tables), results.json (one record per run), the stripped
models g1_planar.stripped.xml and g1_free.stripped.xml, qpos_<caps>.npy for three free-base runs at a = 0.8 (one
row of qpos per 1/120 s of simulated time, from 0.4-0.8 s before the push to 0.3 s after touchdown; replay them in
MuJoCo's viewer), and video.json, the render spec (camera, captions, narration) BotInfo used to turn those three
runs into a video. Anyone else can ignore video.json.
This is what a model does in simulation, never a claim about the machine.
"""
import json
import math
import os
import sys
import traceback
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

IN, OUT = sys.argv[1], sys.argv[2]
HOLD_S = 0.8
AFTER_S = 1.2
DEPTHS = [0.5, 0.8]
KP_HOLD = 500.0
KP_PUSH = {"hip_pitch": 3000.0, "knee": 3000.0, "ankle_pitch": 1500.0}
LEG = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
KNEES = ["left_knee_joint", "right_knee_joint"]
GRAV = 9.81
AIR_STEPS = 10                        # off the ground this many steps in a row = liftoff

N7520_22P5 = (111.0, 131.0, 14.5, 22.7)       # unitree_actuators.py @4e08d96 L0169-0174 (Y1, Y2, X1, X2)
N7520_14P3 = (71.0, 83.3, 22.63, 35.52)       # unitree_actuators.py @4e08d96 L0150-0155
N5020_16 = (24.8, 31.9, 30.86, 40.13)         # unitree_actuators.py @4e08d96 L0203-0207


def flat(v):
    return ("flat", float(v))


def curve(c):
    return ("curve",) + c


CAPSETS = {
    "spec90": {"knee": flat(90)},
    "spec120": {"knee": flat(120)},
    "unitree_curve": {"hip_pitch": curve(N7520_14P3), "hip_yaw": curve(N7520_14P3), "hip_roll": curve(N7520_22P5),
                      "knee": curve(N7520_22P5), "ankle": curve(N5020_16)},
    "rl_lab_flat": {"hip_pitch": flat(88), "hip_yaw": flat(88), "hip_roll": flat(139), "knee": flat(139),
                    "ankle": flat(25)},
    "menagerie139": {},
    "legs300": {"hip": flat(300), "knee": flat(300)},
    "g1cfg": {"hip": flat(300), "knee": flat(300), "ankle": flat(20)},
    "uncapped": {"hip": flat(1e4), "knee": flat(1e4), "ankle": flat(1e4)},
}

# The three free-base runs at a = 0.8 that are recorded for replay and for the video, with their clip windows
# (seconds before the push, seconds after touchdown).
VIDEO_A = 0.8
VIDEO = {"unitree_curve": (0.8, 0.3), "rl_lab_flat": (0.4, 0.3), "g1cfg": (0.4, 0.3)}
REC_HZ = 120.0                        # recorded rows per simulated second; played at 30 fps = quarter speed
FPS = 30

lines = []
ACT = mujoco.mjtObj.mjOBJ_ACTUATOR


def say(s=""):
    lines.append(s)
    print(s)


def find(suffix):
    hits = []
    for root, _, files in os.walk(IN):
        for f in files:
            p = os.path.join(root, f).replace(os.sep, "/")
            if p.endswith(suffix):
                hits.append(p)
    return sorted(hits)[0] if hits else None


def build(path, base):
    tree = ET.parse(path)
    root = tree.getroot()
    parent = {c: p for p in root.iter() for c in p}
    removed = 0
    for g in list(root.iter("geom")):
        if g.get("mesh") is not None or g.get("type") == "mesh":
            parent[g].remove(g)
            removed += 1
    for a in root.iter("asset"):
        for e in list(a):
            if e.get("file") is not None:
                a.remove(e)
    for k in list(root.iter("keyframe")):
        parent[k].remove(k)
    wb = root.find("worldbody")
    pelvis = wb.find("body[@name='pelvis']")
    fj = pelvis.find("freejoint")
    if base == "planar":
        idx = list(pelvis).index(fj)
        pelvis.remove(fj)
        for k, (nm, typ, ax) in enumerate((("base_x", "slide", "1 0 0"), ("base_z", "slide", "0 0 1"),
                                           ("base_pitch", "hinge", "0 1 0"))):
            pelvis.insert(idx + k, ET.Element("joint", {"name": nm, "type": typ, "axis": ax, "limited": "false",
                                                        "armature": "0", "damping": "0", "frictionloss": "0"}))
    wb.insert(0, ET.Element("geom", {"name": "ground", "type": "plane", "size": "0 0 1"}))
    xml = ET.tostring(root, encoding="unicode")
    with open(os.path.join(OUT, "g1_%s.stripped.xml" % base), "w") as f:
        f.write(xml)
    return mujoco.MjModel.from_xml_string(xml), removed


def name(m, kind, i):
    return mujoco.mj_id2name(m, kind, i) or ""


def colliders(m):
    """Every geom that can touch another after stripping (contype or conaffinity set), by type and body."""
    out = {}
    for g in range(m.ngeom):
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue
        typ = {0: "plane", 2: "sphere", 3: "capsule", 4: "ellipsoid", 5: "cylinder", 6: "box"}.get(int(m.geom_type[g]),
                                                                                                  str(m.geom_type[g]))
        body = name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "world"
        out.setdefault(typ, []).append(body)
    return out


def rule_for(n, capset):
    for k, r in CAPSETS[capset].items():
        if k in n:
            return r
    return None


def cap_now(rule, base_cap, qd, effort):
    """The cap this step. For a curve, unitree_actuators.py's _clip_effort: Y1 when the effort and the joint speed
    point the same way, else Y2; full below X1, then linear to 0 at X2."""
    if rule is None:
        return base_cap
    if rule[0] == "flat":
        return rule[1]
    y1, y2, x1, x2 = rule[1:]
    y = y1 if qd * effort > 0 else y2
    if abs(qd) < x1:
        return y
    return max(0.0, y - y / (x2 - x1) * (abs(qd) - x1))


def target(n, a):
    if "hip_pitch" in n:
        return -a
    if "knee" in n:
        return 2 * a
    if "ankle_pitch" in n:
        return -a
    return 0.0


class Rig:
    def __init__(self, path, base):
        self.base = base
        self.m, self.removed = build(path, base)
        m = self.m
        self.acts = {name(m, ACT, i): i for i in range(m.nu)}
        self.leg = {n: i for n, i in self.acts.items() if any(k in n for k in LEG)}
        self.fr0 = {}                   # compiled caps, in joint torque (N*m)
        for n, i in self.acts.items():
            j = m.actuator_trnid[i, 0]
            if m.actuator_forcelimited[i]:
                self.fr0[n] = float(m.actuator_forcerange[i, 1]) * abs(float(m.actuator_gear[i, 0]))
            elif m.jnt_actfrclimited[j]:
                self.fr0[n] = float(m.jnt_actfrcrange[j, 1])
            else:
                self.fr0[n] = 1e4
        for n, i in self.leg.items():
            m.jnt_actfrclimited[m.actuator_trnid[i, 0]] = 0
            m.actuator_forcelimited[i] = 1
        self.gain0 = m.actuator_gainprm.copy()
        self.bias0 = m.actuator_biasprm.copy()
        self.jq = {n: m.jnt_qposadr[m.actuator_trnid[i, 0]] for n, i in self.acts.items()}
        self.jv = {n: m.jnt_dofadr[m.actuator_trnid[i, 0]] for n, i in self.acts.items()}
        self.gear = {n: float(m.actuator_gear[i, 0]) for n, i in self.acts.items()}
        self.pel = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.ground = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.feet = [g for g in range(m.ngeom)
                     if "ankle_roll" in name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
                     and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE]

    def set_kp(self, i, kp):
        self.m.actuator_gainprm[i, 0] = kp
        self.m.actuator_biasprm[i, 1] = -kp

    def pd_effort(self, d, n, i):
        """The joint torque the position actuator would apply this step before its forcerange clip."""
        m, g = self.m, self.gear[n]
        q, qd = d.qpos[self.jq[n]], d.qvel[self.jv[n]]
        f = m.actuator_gainprm[i, 0] * d.ctrl[i] + m.actuator_biasprm[i, 1] * g * q + m.actuator_biasprm[i, 2] * g * qd
        return g * f

    def place(self, d, a):
        m = self.m
        mujoco.mj_resetData(m, d)
        for n, i in self.acts.items():
            j = m.actuator_trnid[i, 0]
            v = target(n, a)
            if m.jnt_limited[j]:
                v = float(np.clip(v, m.jnt_range[j, 0], m.jnt_range[j, 1]))
            d.qpos[self.jq[n]] = v
            d.ctrl[i] = v
        mujoco.mj_kinematics(m, d)
        zmin = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in self.feet)
        dz = 0.0005 - zmin
        if self.base == "planar":
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "base_z")]] += dz
        else:
            d.qpos[2] += dz
        mujoco.mj_forward(m, d)
        mujoco.mj_subtreeVel(m, d)

    def on_ground(self, d):
        return any(self.ground in (d.contact[c].geom1, d.contact[c].geom2) for c in range(d.ncon))

    def run(self, a, capset, clip=None):
        m = self.m
        m.actuator_gainprm[:] = self.gain0
        m.actuator_biasprm[:] = self.bias0
        for n, i in self.leg.items():
            self.set_kp(i, KP_HOLD)
        d = mujoco.MjData(m)
        self.place(d, 0.0)
        com_stand = float(d.subtree_com[0][2])
        self.place(d, a)
        com_crouch0 = float(d.subtree_com[0][2])
        dt = m.opt.timestep
        n_hold, n_after = int(round(HOLD_S / dt)), int(round(AFTER_S / dt))
        rules = {n: rule_for(n, capset) for n in self.leg}
        rec = {"t": [], "com": [], "vz": [], "ground": [], "knee_tau": [], "knee_qd": [], "knee_at_cap": [],
               "tilt": [], "pz": []}
        qrec, trec, next_rec = [], [], 0.0
        lifted = False
        air = 0
        for k in range(n_hold + n_after):
            push = k >= n_hold
            caps = [1e4, 1e4]
            for n, i in self.leg.items():
                tgt = 0.0 if push else target(n, a)
                d.ctrl[i] = tgt
                if push and not lifted:
                    for key, kp in KP_PUSH.items():
                        if key in n:
                            self.set_kp(i, kp)
                qd = d.qvel[self.jv[n]]
                c = cap_now(rules[n], self.fr0[n], qd, self.pd_effort(d, n, i))
                m.actuator_forcerange[i] = [-c / abs(self.gear[n]), c / abs(self.gear[n])]
                if n in KNEES:
                    caps[KNEES.index(n)] = c
            mujoco.mj_step(m, d)
            tau = [abs(float(d.actuator_force[self.acts[kn]])) * abs(self.gear[kn]) for kn in KNEES]
            mujoco.mj_forward(m, d)
            mujoco.mj_subtreeVel(m, d)
            g = self.on_ground(d)
            if push and not lifted:
                air = 0 if g else air + 1
                if air >= AIR_STEPS:
                    lifted = True
                    for n, i in self.leg.items():
                        self.set_kp(i, KP_HOLD)
            rec["t"].append(d.time)
            rec["com"].append(float(d.subtree_com[0][2]))
            rec["vz"].append(float(d.subtree_linvel[0][2]))
            rec["ground"].append(g)
            rec["knee_tau"].append(max(tau))
            rec["knee_qd"].append(max(abs(float(d.qvel[self.jv[kn]])) for kn in KNEES))
            rec["knee_at_cap"].append(any(tau[j] >= caps[j] - 1e-6 and caps[j] < 1e4 for j in range(2)))
            rec["tilt"].append(math.degrees(math.acos(max(-1.0, min(1.0, float(d.xmat[self.pel][8]))))))
            rec["pz"].append(float(d.xpos[self.pel][2]))
            if clip is not None and d.time >= next_rec - 1e-9:
                qrec.append(d.qpos.copy())
                trec.append(d.time)
                next_rec += 1.0 / REC_HZ
        r = {k: np.array(v) for k, v in rec.items()}
        after = slice(n_hold, None)
        gr = r["ground"][after]
        lift_rel = None
        run_len = 0
        for idx, gg in enumerate(gr):
            run_len = 0 if gg else run_len + 1
            if run_len >= AIR_STEPS:
                lift_rel = idx - AIR_STEPS + 1
                break
        out = {"base": self.base, "crouch_a": a, "caps": capset,
               "knee_cap_resolved": round(cap_now(rules[KNEES[0]], self.fr0[KNEES[0]], 0.0, 1.0), 2),
               "com_stand_m": round(com_stand, 4), "com_crouch_start_m": round(com_crouch0, 4),
               "com_end_of_hold_m": round(float(r["com"][n_hold - 1]), 4),
               "held_crouch": bool(r["com"][n_hold - 1] > com_crouch0 - 0.02 and r["tilt"][n_hold - 1] < 20),
               "knee_tau_end_of_hold": round(float(r["knee_tau"][n_hold - 1]), 2),
               "max_tilt_deg": round(float(r["tilt"].max()), 1),
               "fell": bool(r["tilt"].max() > 60 or r["pz"].min() < 0.3)}
        push_tau = r["knee_tau"][after]
        push_cap = r["knee_at_cap"][after]
        push_qd = r["knee_qd"][after]
        end = lift_rel if lift_rel is not None else int(round(0.5 / dt))
        out["peak_knee_Nm_push"] = round(float(push_tau[:end + 1].max()), 2)
        out["knee_at_cap_share_push"] = round(float(push_cap[:end + 1].mean()), 3)
        out["peak_knee_speed_rads_push"] = round(float(push_qd[:end + 1].max()), 2)
        com_after = r["com"][after]
        out["com_apex_above_stand_cm"] = round(100 * float(com_after.max() - com_stand), 2)
        t_push = n_hold * dt
        if lift_rel is None:
            out.update(liftoff=False)
            t_end = t_push + 1.0
        else:
            vz = float(r["vz"][after][lift_rel])
            land = next((j for j in range(lift_rel, len(gr)) if gr[j]), len(gr) - 1)
            out.update(liftoff=True, push_ms=round(1000 * lift_rel * dt, 1), takeoff_vz=round(vz, 3),
                       ballistic_rise_cm=round(100 * max(0.0, vz) ** 2 / (2 * GRAV), 2),
                       flight_ms=round(1000 * (land - lift_rel) * dt, 1),
                       com_apex_in_flight_above_stand_cm=round(100 * float(com_after[lift_rel:land + 1].max()
                                                                           - com_stand), 2),
                       touchdown_s=round(t_push + land * dt, 3))
            out["max_tilt_deg_to_touchdown"] = round(float(r["tilt"][:n_hold + land + 1].max()), 1)
            t_end = t_push + land * dt + (clip[1] if clip else 0.0)
        if clip is None:
            return out, None
        t0 = t_push - clip[0]
        rows = [q for q, t in zip(qrec, trec) if t0 - 1e-9 <= t <= t_end + 1e-9]
        out["clip_rows"] = len(rows)
        return out, np.array(rows, dtype=np.float64)


def write_video(byc):
    """video.json for the three recorded runs: a vertical clip, one part per run, numbers from this run. A clip
    shorter than its narration is replayed from the start (qpos_<caps>_video.npy), so a part can show a jump twice."""
    h = {c: round(byc[c]["com_apex_in_flight_above_stand_cm"]) for c in VIDEO}
    narr = {
        "unitree_curve": "How high can this humanoid jump? In simulation, it depends on whose torque limit you load. "
                         "With the figures in Unitree's own motor file, its center of mass rises %d centimeters."
                         % h["unitree_curve"],
        "rl_lab_flat": "Unitree's training config for this model, 139 at the knee: %d." % h["rl_lab_flat"],
        "g1cfg": "NVIDIA Isaac Lab's stock config, 300: %d centimeters. Same model, same crude push, in simulation, "
                 "not the real robot. The cap is what a policy learns it can ask for." % h["g1cfg"],
    }
    caps = {
        "unitree_curve": ["How high can it jump? Depends on the torque limit.",
                          "Unitree motor-file curve: center of mass +%d cm" % h["unitree_curve"]],
        "rl_lab_flat": ["Unitree 29-DoF training config, knee 139 N*m: +%d cm" % h["rl_lab_flat"]],
        "g1cfg": ["Isaac Lab G1_CFG, hips+knees 300 N*m: +%d cm" % h["g1cfg"],
                  "Simulation, quarter speed. Not the real robot."],
    }
    parts, captions, narration = [], [], []
    t = 0.0
    for c in VIDEO:
        rows = np.load(os.path.join(OUT, "qpos_%s.npy" % c))
        need = int(round((len(narr[c].split()) / 2.3 + 0.6) * FPS))
        if len(rows) < need:                     # a clip shorter than its narration replays instead of freezing
            loop = rows.copy()
            while len(rows) < need:
                rows = np.vstack([rows, loop])
            rows = rows[:need]
        np.save(os.path.join(OUT, "qpos_%s_video.npy" % c), rows)
        dur = len(rows) / FPS
        parts.append({"render": {"model": "unitree_g1", "scene": "scene.xml", "qpos": "qpos_%s_video.npy" % c,
                                 "camera": {"distance": 4.4, "azimuth": 90, "elevation": -5, "lookat": [0, 0, 1.05]}}})
        step = dur / len(caps[c])
        for j, text in enumerate(caps[c]):
            captions.append({"from": round(t + j * step, 2), "to": round(t + (j + 1) * step, 2), "text": text})
        narration.append({"at": round(t + 0.1, 2), "text": narr[c]})
        t += dur
    video = {"name": "g1-jump-caps", "size": "vertical", "fps": FPS, "parts": parts, "captions": captions,
             "narration": narration, "voice": "Charon"}
    with open(os.path.join(OUT, "video.json"), "w") as f:
        json.dump(video, f, indent=1)
    say("\nvideo.json: %d parts, %.1f s, %d narration characters; apex cm used: %s"
        % (len(parts), t, sum(len(x["text"]) for x in narration), json.dumps(h)))


def main():
    path = find("unitree_g1/g1.xml")
    say("# Jump budget: G1 leg torque caps (Menagerie g1.xml, meshes stripped)")
    say("mujoco %s; numpy %s; input %s" % (mujoco.__version__, np.__version__, path))
    results = []
    video = {}
    for base in ("planar", "free"):
        try:
            rig = Rig(path, base)
            m = rig.m
            say("\n## base=%s (removed %d mesh geoms, mass %.3f kg, timestep %g s, integrator %d)"
                % (base, rig.removed, float(m.body_mass.sum()), m.opt.timestep, int(m.opt.integrator)))
            say("collision geoms left: " + "; ".join("%d %s (%s)" % (len(v), k, ", ".join(sorted(set(v))))
                                                     for k, v in sorted(colliders(m).items())))
            say("compiled leg caps N*m: " + ", ".join("%s %g" % (n, rig.fr0[n]) for n in sorted(rig.leg)))
            say("compiled kv (biasprm[2]) knee: %g" % rig.bias0[rig.acts[KNEES[0]], 2])
            say("a | caps | knee cap at 0 rad/s | held | liftoff | push ms | takeoff vz | ballistic rise cm | "
                "apex above stand cm | peak knee Nm | knee at cap % | peak knee rad/s | max tilt | fell")
            for a in DEPTHS:
                for capset in CAPSETS:
                    clip = VIDEO.get(capset) if (base == "free" and a == VIDEO_A) else None
                    try:
                        r, rows = rig.run(a, capset, clip)
                    except Exception:
                        say("ERROR a=%s caps=%s\n%s" % (a, capset, traceback.format_exc()))
                        continue
                    results.append(r)
                    if rows is not None and r.get("liftoff"):
                        np.save(os.path.join(OUT, "qpos_%s.npy" % capset), rows)
                        video[capset] = r
                    say("%.1f | %s | %s | %s | %s | %s | %s | %s | %s | %s | %.0f | %s | %s | %s" % (
                        a, capset, r["knee_cap_resolved"], r["held_crouch"], r["liftoff"], r.get("push_ms", "-"),
                        r.get("takeoff_vz", "-"), r.get("ballistic_rise_cm", "-"), r["com_apex_above_stand_cm"],
                        r["peak_knee_Nm_push"], 100 * r["knee_at_cap_share_push"], r["peak_knee_speed_rads_push"],
                        r["max_tilt_deg"], r["fell"]))
        except Exception:
            say("ERROR base=%s\n%s" % (base, traceback.format_exc()))
    say("\n## free base, CoM apex in flight above standing (cm), and ratios to unitree_curve")
    for a in DEPTHS:
        row = {r["caps"]: r.get("com_apex_in_flight_above_stand_cm") for r in results
               if r["base"] == "free" and r["crouch_a"] == a}
        base_h = row.get("unitree_curve")
        say("a=%.1f: %s" % (a, json.dumps(row)))
        if base_h:
            say("a=%.1f ratios: %s" % (a, json.dumps({k: round(v / base_h, 2) for k, v in row.items() if v})))
    with open(os.path.join(OUT, "results.json"), "w") as f:
        json.dump(results, f, indent=1)
    if all(c in video for c in VIDEO):
        write_video(video)
    else:
        say("not every recorded run lifted off; no video.json written")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        say("ERROR\n" + traceback.format_exc())
    with open(os.path.join(OUT, "jump_budget.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
