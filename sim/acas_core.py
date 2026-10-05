#!/usr/bin/env python3
"""
acas_core.py  -  ACAS logic (from acas_poc.py): ACAS Xp-style collision avoidance for two GA aircraft
=======================================================================================
Honeywell Aerospace x Devils Invent 2026 (ASU Ira A. Fulton Schools of Engineering), PS2. Non-certified prototype logic; thresholds are
configurable starting values, not certified values.

Pipeline (same as the project tree):
    own state + peer state (from P2P packet)
      -> TRACKER          relative state, uncertainty that grows with packet age
      -> THREAT           tau, modified tau (DMOD), CPA, curved-path prediction
      -> ALERT            NONE / PROXIMATE / TA / RA
      -> SELECTOR         look-ahead rollouts x uncertainty samples, perf limits,
                          legality filter (same-sense, 14 CFR 91.113(e), GCAS veto)
      -> COORDINATION     lower ICAO = master proposes, peer acks (L1)
      -> AUTOMATION       3 s countdown -> autopilot engages -> clear of conflict

Run:   python3 acas_poc.py            (all scenarios)
       python3 acas_poc.py --plot     (also saves track plots as PNG)
Edit the CUSTOM SCENARIO block at the bottom to try your own numbers.
Requires: numpy (matplotlib only for --plot)
"""
import math
import sys
import copy
from dataclasses import dataclass
import numpy as np

# ---------------------------------------------------------------- units
KT, FT, NM, G = 0.514444, 0.3048, 1852.0, 9.80665
FPM = FT / 60.0
D2R = math.pi / 180.0

# ---------------------------------------------------------------- configuration
CFG = dict(
    DETECT_NM=5.0,        # packets heard inside this range
    ALERT_NM=3.0,         # TA may trigger by range inside this
    PROX_NM=4.0, PROX_V_FT=1200,     # proximate (filled diamond) inside 4 nm; 4-5 nm shows hollow
    TA_TAU=40.0, RA_TAU=25.0, DMOD_NM=0.3,
    TA_H_NM=0.5, TA_V_FT=600,        # TA protected zone
    RA_H_FT=1500, RA_V_FT=400,       # RA protected zone
    K_SIGMA=2.0,                     # zone inflation: D* = D + k*sigma
    NMAC_H_FT=500, NMAC_V_FT=100,
    COUNTDOWN_S=3.0,
    TURN_BANK_DEG=15.0, MAX_BANK_DEG=20.0, ROLL_RATE_DPS=4.0,
    VS_CMD_FPM=1000.0, VS_ACCEL_G=0.10,
    GROUND_FLOOR_FT=1000.0, GCAS_LOOKAHEAD_S=20.0,
    HORIZON_S=60.0, DT=0.5, N_SAMPLES=40,
    A_UNKNOWN=1.0,                   # m/s^2, unknown-maneuver accel for stale data
)
MANEUVERS = ["MAINTAIN", "TURN_RIGHT", "TURN_LEFT", "CLIMB", "DESCEND"]
CALLOUT = {"MAINTAIN": "MONITOR, MAINTAIN COURSE", "TURN_RIGHT": "TURN RIGHT, TURN RIGHT",
           "TURN_LEFT": "TURN LEFT, TURN LEFT", "CLIMB": "CLIMB, CLIMB",
           "DESCEND": "DESCEND, DESCEND"}
LEVELS = {-1: "NOT DETECTED", 0: "OTHER TRAFFIC", 1: "PROXIMATE", 2: "TA", 3: "RA"}


# ---------------------------------------------------------------- aircraft
@dataclass
class Aircraft:
    name: str
    icao: int
    x: float; y: float; h: float          # m east, m north, m MSL
    hdg: float                            # rad, 0 = north, clockwise
    gs: float                             # m/s
    vs: float = 0.0                       # m/s
    bank: float = 0.0                     # rad, + = right
    max_climb: float = 700 * FPM
    max_desc: float = 1000 * FPM
    ground: float = 0.0                   # terrain elevation, m MSL
    sigma_pos: float = 5.0                # m, GNSS-class position 1-sigma
    sigma_vel: float = 0.3                # m/s
    packet_age: float = 0.0               # s, age of this aircraft's last packet
    bank_cmd: float = None
    vs_cmd: float = None

    def __post_init__(self):
        if self.bank_cmd is None: self.bank_cmd = self.bank
        if self.vs_cmd is None: self.vs_cmd = self.vs

    @property
    def vx(self): return self.gs * math.sin(self.hdg)
    @property
    def vy(self): return self.gs * math.cos(self.hdg)


def make(name, icao, east_nm, north_nm, alt_ft, hdg_deg, speed_kt, vs_fpm=0.0,
         turn_dps=0.0, climb_fpm=700, desc_fpm=1000, ground_ft=0.0,
         sigma_pos=5.0, packet_age=0.0):
    """Build an aircraft from pilot-friendly units."""
    gs = speed_kt * KT
    bank = math.atan(turn_dps * D2R * gs / G)
    return Aircraft(name, icao, east_nm * NM, north_nm * NM, alt_ft * FT, hdg_deg * D2R,
                    gs, vs_fpm * FPM, bank, climb_fpm * FPM, desc_fpm * FPM,
                    ground_ft * FT, sigma_pos, 0.3, packet_age)


def latlon_to_local(lat_ref, lon_ref, lat, lon):
    """Flat-earth conversion, fine within ~10 nm. Returns (east_m, north_m)."""
    north = (lat - lat_ref) * 111320.0
    east = (lon - lon_ref) * 111320.0 * math.cos(lat_ref * D2R)
    return east, north


def step(ac: Aircraft, dt: float):
    """Simple GA kinematics with roll-rate and vertical-acceleration limits."""
    roll = CFG["ROLL_RATE_DPS"] * D2R * dt
    dvs = CFG["VS_ACCEL_G"] * G * dt
    ac.bank += max(-roll, min(roll, ac.bank_cmd - ac.bank))
    ac.vs += max(-dvs, min(dvs, ac.vs_cmd - ac.vs))
    ac.hdg += G * math.tan(ac.bank) / ac.gs * dt
    ac.x += ac.gs * math.sin(ac.hdg) * dt
    ac.y += ac.gs * math.cos(ac.hdg) * dt
    ac.h += ac.vs * dt


# ---------------------------------------------------------------- vectorized look-ahead
def propagate(x, y, h, hdg, gs, vs, bank, cmd=None, delay=0.0, horizon=None):
    """Propagate N copies of an aircraft. cmd=(bank_cmd, vs_cmd) applied after delay.
    Returns arrays (T, N) of x, y, h."""
    horizon = CFG["HORIZON_S"] if horizon is None else horizon
    dt, steps = CFG["DT"], int(horizon / CFG["DT"])
    roll, dvs = CFG["ROLL_RATE_DPS"] * D2R * dt, CFG["VS_ACCEL_G"] * G * dt
    hold_b, hold_v = bank.copy(), vs.copy()
    X, Y, H = [x.copy()], [y.copy()], [h.copy()]
    for k in range(steps):
        bc, vc = (hold_b, hold_v) if (cmd is None or k * dt < delay) else cmd
        bank = bank + np.clip(bc - bank, -roll, roll)
        vs = vs + np.clip(vc - vs, -dvs, dvs)
        hdg = hdg + G * np.tan(bank) / gs * dt
        x = x + gs * np.sin(hdg) * dt
        y = y + gs * np.cos(hdg) * dt
        h = h + vs * dt
        X.append(x); Y.append(y); H.append(h)
    return np.array(X), np.array(Y), np.array(H)


def arrays(ac, n):
    f = lambda v: np.full(n, float(v))
    return [f(ac.x), f(ac.y), f(ac.h), f(ac.hdg), f(ac.gs), f(ac.vs), f(ac.bank)]


def maneuver_cmd(ac, m, n):
    """Map a maneuver to (bank_cmd, vs_cmd) arrays, or None for MAINTAIN."""
    b, v = np.full(n, ac.bank), np.full(n, ac.vs)
    if m == "MAINTAIN":   return None
    if m == "TURN_RIGHT": return (np.full(n, CFG["TURN_BANK_DEG"] * D2R), v)
    if m == "TURN_LEFT":  return (np.full(n, -CFG["TURN_BANK_DEG"] * D2R), v)
    if m == "CLIMB":      return (b, np.full(n, min(CFG["VS_CMD_FPM"] * FPM, ac.max_climb)))
    if m == "DESCEND":    return (b, np.full(n, -min(CFG["VS_CMD_FPM"] * FPM, ac.max_desc)))


# ---------------------------------------------------------------- tracker
def rel_sigma(own, intr):
    """Combined relative uncertainty; grows with the intruder's packet age."""
    age, a = intr.packet_age, CFG["A_UNKNOWN"]
    sp = math.sqrt(own.sigma_pos**2 + intr.sigma_pos**2 + (intr.sigma_vel * age)**2
                   + (0.5 * a * age**2)**2)
    sv = math.sqrt(own.sigma_vel**2 + intr.sigma_vel**2 + (a * age)**2)
    sz = math.sqrt(2) * 25 * FT + 0.5 * 0.3 * age**2
    return sp, sv, sz


# ---------------------------------------------------------------- threat assessment
def assess(own, intr):
    """Everything the decision logic needs, from 'own' looking at 'intr'."""
    rx, ry = intr.x - own.x, intr.y - own.y
    vx, vy = intr.vx - own.vx, intr.vy - own.vy
    rng = math.hypot(rx, ry)
    rr = (rx * vx + ry * vy) / rng                       # range rate (<0 closing)
    dmod = CFG["DMOD_NM"] * NM
    tau = -rng / rr if rr < 0 else math.inf
    if rng < dmod:   tau_mod = 0.0
    elif rr < 0:     tau_mod = -(rng**2 - dmod**2) / (rng * rr)
    else:            tau_mod = math.inf
    vv = vx * vx + vy * vy
    t_cpa = max(0.0, -(rx * vx + ry * vy) / vv) if vv > 1e-9 else 0.0
    d_cpa = math.hypot(rx + vx * t_cpa, ry + vy * t_cpa)  # straight-line miss

    # curved-path prediction: both keep current bank and vertical speed
    A = propagate(*arrays(own, 1)); B = propagate(*arrays(intr, 1))
    dh = np.hypot(A[0] - B[0], A[1] - B[1])[:, 0]
    dv = np.abs(A[2] - B[2])[:, 0]
    k = int(np.argmin(dh))
    sp, sv, sz = rel_sigma(own, intr)
    kk = CFG["K_SIGMA"]
    ta_h, ta_v = CFG["TA_H_NM"] * NM + kk * sp, CFG["TA_V_FT"] * FT + kk * sz
    ra_h, ra_v = CFG["RA_H_FT"] * FT + kk * sp, CFG["RA_V_FT"] * FT + kk * sz
    confl_ta = bool(np.any((dh < ta_h) & (dv < ta_v)))
    confl_ra = bool(np.any((dh < ra_h) & (dv < ra_v)))

    if rng > CFG["DETECT_NM"] * NM:                                   level = -1
    elif confl_ra and (tau_mod <= CFG["RA_TAU"] or rng < dmod):       level = 3
    elif confl_ta and (rng <= CFG["ALERT_NM"] * NM or tau_mod <= CFG["TA_TAU"]): level = 2
    elif rng <= CFG["PROX_NM"] * NM and abs(intr.h - own.h) <= CFG["PROX_V_FT"] * FT: level = 1
    else:                                                              level = 0

    brg = (math.degrees(math.atan2(rx, ry)) - math.degrees(own.hdg) + 360) % 360
    return dict(level=level, range_m=rng, range_rate=rr, tau=tau, tau_mod=tau_mod,
                t_cpa=t_cpa, d_cpa=d_cpa, d_min_curved=float(dh[k]),
                t_min_curved=k * CFG["DT"], dv_at_min=float(dv[k]),
                rel_alt_ft=(intr.h - own.h) / FT, bearing_deg=brg,
                sigma_pos=sp, sigma_vel=sv, sigma_alt=sz, ra_zone_h=ra_h, ta_zone_h=ta_h,
                ra_zone_v=ra_v, ta_zone_v=ta_v, confl_ra=confl_ra, confl_ta=confl_ta,
                rx=rx, ry=ry, vx=vx, vy=vy, dmod=dmod, packet_age=intr.packet_age)


# ---------------------------------------------------------------- maneuver selection
def headon(a, b):
    d = abs((math.degrees(a.hdg - b.hdg) + 180) % 360 - 180)
    return d >= 135.0


def rel_bearing(ac, other):
    b = math.degrees(math.atan2(other.x - ac.x, other.y - ac.y)) - math.degrees(ac.hdg)
    return b % 360


def gives_way(ac, other, is_headon):
    """14 CFR 91.113: head-on -> both; overtaking -> the overtaking aircraft;
    converging -> the aircraft that has the other on its right."""
    if is_headon:
        return True
    b, b2 = rel_bearing(ac, other), rel_bearing(other, ac)
    if (b < 70 or b > 290) and 110 < b2 < 250:   # ac is overtaking other
        return True
    if (b2 < 70 or b2 > 290) and 110 < b < 250:  # other is overtaking ac
        return False
    return 0 < b <= 112.5


def legal(ma, mb, a, b, is_headon):
    if ma == mb and ma in ("CLIMB", "DESCEND"):
        return False, "same vertical sense"
    if is_headon and "TURN_LEFT" in (ma, mb):
        return False, "head-on: 14 CFR 91.113(e) requires turning right"
    if is_headon and {ma, mb} == {"TURN_RIGHT", "MAINTAIN"}:
        return False, "head-on: both aircraft must alter course to the right"
    for m, ac in ((ma, a), (mb, b)):
        if m == "DESCEND":
            rate = min(CFG["VS_CMD_FPM"] * FPM, ac.max_desc)
            agl_after = (ac.h - ac.ground) - rate * CFG["GCAS_LOOKAHEAD_S"]
            if agl_after < CFG["GROUND_FLOOR_FT"] * FT:
                return False, f"GCAS veto: {ac.name} descent would reach {agl_after/FT:.0f} ft AGL"
        if m == "CLIMB" and ac.max_climb < 150 * FPM:
            return False, f"{ac.name} cannot climb"
    return True, ""


def cost(m, ac, give_way=True):
    if m == "MAINTAIN": return 0.0
    c = 0.0 if give_way else 0.5          # prefer the give-way aircraft to maneuver
    if m == "TURN_RIGHT": return c + 1.0
    if m == "TURN_LEFT":  return c + 1.3  # right turns preferred by convention
    if m == "CLIMB": return c + 1.5 + (2.0 if ac.max_climb < 500 * FPM else 0.0)
    return c + 1.2  # DESCEND


def select(a, b, delay, t_now=0.0, prefer=None, fixed=None, allowed=None):
    """Joint maneuver choice. Deterministic seed so both nodes compute the same
    answer from the same shared data (needed for L2 implicit coordination)."""
    master, slave = (a, b) if a.icao < b.icao else (b, a)
    n = CFG["N_SAMPLES"]
    rng = np.random.default_rng(master.icao * 7919 + slave.icao * 31 + int(t_now))
    sp, sv, sz = rel_sigma(master, slave)
    S = arrays(slave, n)
    S[0] = S[0] + rng.normal(0, sp, n); S[1] = S[1] + rng.normal(0, sp, n)
    S[2] = S[2] + rng.normal(0, sz, n)
    vx = slave.vx + rng.normal(0, sv, n); vy = slave.vy + rng.normal(0, sv, n)
    S[3], S[4] = np.arctan2(vx, vy), np.hypot(vx, vy)
    M = arrays(master, n)
    is_ho = headon(master, slave)
    gw_m, gw_s = gives_way(master, slave, is_ho), gives_way(slave, master, is_ho)
    horizon = max(CFG["HORIZON_S"], delay + 45.0)   # always look past the conflict
    nm_h, nm_v = CFG["NMAC_H_FT"] * FT, CFG["NMAC_V_FT"] * FT
    z_h, z_v = CFG["RA_H_FT"] * FT, CFG["RA_V_FT"] * FT
    results, vetoes = [], []
    for i, mm in enumerate(MANEUVERS):
        for j, ms in enumerate(MANEUVERS):
            if fixed and (fixed.get(master.name, mm) != mm or fixed.get(slave.name, ms) != ms):
                continue          # e.g. a pilot who took manual control is fixed to what he is flying
            if allowed and (mm not in allowed.get(master.name, MANEUVERS) or ms not in allowed.get(slave.name, MANEUVERS)):
                continue          # no-message modes: only rule-complementary maneuvers
            ok, why = legal(mm, ms, master, slave, is_ho)
            if not ok:
                vetoes.append((mm, ms, why)); continue
            PM = propagate(*[v.copy() for v in M], cmd=maneuver_cmd(master, mm, n), delay=delay, horizon=horizon)
            PS = propagate(*[v.copy() for v in S], cmd=maneuver_cmd(slave, ms, n), delay=delay, horizon=horizon)
            dh = np.hypot(PM[0] - PS[0], PM[1] - PS[1]); dv = np.abs(PM[2] - PS[2])
            p_nmac = float(np.mean(np.any((dh < nm_h) & (dv < nm_v), axis=0)))
            p_zone = float(np.mean(np.any((dh < z_h) & (dv < z_v), axis=0)))
            score = 1000 * p_nmac + 50 * p_zone + cost(mm, master, gw_m) + cost(ms, slave, gw_s)
            if prefer and prefer.get(master.name) == mm and prefer.get(slave.name) == ms:
                score -= 0.5      # hysteresis: keep the agreed plan unless clearly worse
            results.append(dict(master=mm, slave=ms, p_nmac=p_nmac, p_zone=p_zone,
                                score=score, order=i * 10 + j,
                                med_min_h=float(np.median(dh.min(axis=0)))))
    results.sort(key=lambda r: (r["score"], r["order"]))
    best = results[0]
    plan = {master.name: best["master"], slave.name: best["slave"]}
    return plan, best, results, vetoes, master.name


# ---------------------------------------------------------------- reporting
def fmt_t(x): return "  inf" if math.isinf(x) else f"{x:5.1f}"


def snapshot(a, b, title=""):
    s = assess(a, b)
    print(f"  Snapshot from {a.name}'s view{(' - ' + title) if title else ''}:")
    print(f"    range {s['range_m']/NM:5.2f} nm   closure {-s['range_rate']/KT:6.1f} kt   "
          f"bearing {s['bearing_deg']:5.1f} deg   rel alt {s['rel_alt_ft']:+6.0f} ft")
    print(f"    tau {fmt_t(s['tau'])} s   modified tau {fmt_t(s['tau_mod'])} s   "
          f"straight CPA in {s['t_cpa']:5.1f} s at {s['d_cpa']:6.0f} m")
    print(f"    curved prediction (60 s look-ahead): closest {s['d_min_curved']:6.0f} m horiz / "
          f"{s['dv_at_min']/FT:4.0f} ft vert at t+{s['t_min_curved']:4.1f} s")
    print(f"    uncertainty sigma {s['sigma_pos']:5.1f} m  ->  RA zone {s['ra_zone_h']:5.0f} m, "
          f"TA zone {s['ta_zone_h']:5.0f} m")
    print(f"    ALERT: {LEVELS[s['level']]}")
    return s


def run_encounter(A, B, title, use_acas=True, t_end=150.0, quiet=False, log=None):
    """Closed loop: ACAS at 1 Hz, kinematics at DT. No pilot response is simulated,
    so the countdown always expires and the autopilot engages."""
    A, B = copy.deepcopy(A), copy.deepcopy(B)
    dt = CFG["DT"]; t = 0.0
    state, plan, ra_t, prev = "MONITOR", None, None, None
    min_h, v_at, nmac = math.inf, 0.0, False
    events = []
    def ev(msg):
        events.append(f"    t={t:6.1f}s  {msg}")
    while t <= t_end:
        if use_acas and abs(t - round(t)) < 1e-9:      # 1 Hz decision cycle
            s = assess(A, B)
            if state == "MONITOR":
                if s["level"] != prev and s["level"] in (-1, 0, 1):
                    ev(f"{LEVELS[s['level']]:<14} range {s['range_m']/NM:4.2f} nm  tau {fmt_t(s['tau_mod'])} s")
                if s["level"] == 2 and prev != 2:
                    lead = max(0.0, s["tau_mod"] - CFG["RA_TAU"]) + CFG["COUNTDOWN_S"]
                    plan, best, *_ , mname = select(A, B, lead, t)
                    ev(f"TA  \"TRAFFIC, TRAFFIC\"  range {s['range_m']/NM:4.2f} nm  tau {fmt_t(s['tau_mod'])} s")
                    ev(f"    pre-negotiated (L1, master {mname}): "
                       f"{A.name} {plan[A.name]}, {B.name} {plan[B.name]}")
                if s["level"] == 3:
                    new, best, results, vetoes, mname = select(A, B, CFG["COUNTDOWN_S"], t, prefer=plan)
                    tag = "confirmed" if new == plan else "updated"
                    plan, ra_t, state = new, t, "RA"
                    ev(f"RA  range {s['range_m']/NM:4.2f} nm  tau {fmt_t(s['tau_mod'])} s  "
                       f"(plan {tag}; risk if flown P(NMAC)={best['p_nmac']:.2f})")
                    ev(f"    {A.name}: \"{CALLOUT[plan[A.name]]}\"   "
                       f"{B.name}: \"{CALLOUT[plan[B.name]]}\"   countdown {CFG['COUNTDOWN_S']:.0f} s")
                    if log is not None: log.update(results=results, vetoes=vetoes, best=best)
                prev = s["level"]
            elif state == "RA" and t >= ra_t + CFG["COUNTDOWN_S"]:
                for ac in (A, B):
                    c = maneuver_cmd(ac, plan[ac.name], 1)
                    if c is not None:
                        ac.bank_cmd, ac.vs_cmd = float(c[0][0]), float(c[1][0])
                state = "ENGAGED"
                ev("no pilot response -> AUTOPILOT ENGAGED")
            elif state == "ENGAGED" and s["range_rate"] > 0 and s["range_m"] > s["ra_zone_h"]:
                for ac in (A, B):
                    ac.bank_cmd, ac.vs_cmd = 0.0, 0.0
                state = "CLEAR"
                ev(f"\"CLEAR OF CONFLICT\"  range {s['range_m']/NM:4.2f} nm, control returned")
        step(A, dt); step(B, dt)
        dh, dv = math.hypot(A.x - B.x, A.y - B.y), abs(A.h - B.h)
        if dh < min_h: min_h, v_at = dh, dv
        if dh < CFG["NMAC_H_FT"] * FT and dv < CFG["NMAC_V_FT"] * FT: nmac = True
        if log is not None:
            log.setdefault("trackA", []).append((A.x, A.y)); log.setdefault("trackB", []).append((B.x, B.y))
        t += dt
    if not quiet:
        for e in events: print(e)
    return dict(min_h=min_h, v_at=v_at, nmac=nmac, events=events)


def scenario(title, A, B, expect, plot_name=None):
    print("=" * 92); print(title); print("=" * 92)
    print(f"  Expected: {expect}")
    snapshot(A, B)
    print("  Timeline with ACAS:")
    logA, logB = {}, {}
    with_ = run_encounter(A, B, title, True, log=logA)
    base = run_encounter(A, B, title, False, quiet=True, log=logB)
    def res(r):
        return (f"{r['min_h']:6.0f} m horiz / {r['v_at']/FT:4.0f} ft vert"
                f"{'  NMAC!' if r['nmac'] else ''}")
    print(f"  Result  no ACAS : {res(base)}")
    print(f"          with ACAS: {res(with_)}")
    if "vetoes" in logA and logA["vetoes"]:
        uniq = sorted({v[2] for v in logA["vetoes"]})
        print("  Vetoed options: " + "; ".join(uniq))
    if "results" in logA:
        print("  Top 3 options at RA (master, slave): " + " | ".join(
            f"{r['master']}/{r['slave']} P(NMAC)={r['p_nmac']:.2f} score={r['score']:.1f}"
            for r in logA["results"][:3]))
    if plot_name and "--plot" in sys.argv:
        plot(title, logA, logB, plot_name)
    print()
    return title, base, with_


def plot(title, logA, logB, name):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.5, 7))
    for lg, style, lab in ((logB, "--", "no ACAS"), (logA, "-", "with ACAS")):
        a, b = np.array(lg["trackA"]) / NM, np.array(lg["trackB"]) / NM
        ax.plot(a[:, 0], a[:, 1], style, color="#1F5FA8", label=f"A {lab}")
        ax.plot(b[:, 0], b[:, 1], style, color="#9B2468", label=f"B {lab}")
    ax.set_aspect("equal"); ax.grid(alpha=.3)
    ax.set_xlabel("east (nm)"); ax.set_ylabel("north (nm)")
    ax.set_title(title, fontsize=10); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(name, dpi=130); plt.close(fig)
    print(f"  saved {name}")


# ---------------------------------------------------------------- scenarios
if __name__ == "__main__":
    out = []
    # S1: head-on, same altitude, 100 m lateral offset, 120 kt each
    out.append(scenario("S1  Head-on, co-altitude, 120 kt each, 100 m offset",
        make("A", 0xA1, 0, 0, 4500, 0, 120), make("B", 0xB2, 0.054, 6, 4500, 180, 120),
        "TA near 3 nm, RA at tau ~25 s, complementary maneuver, NMAC avoided",
        "acas_poc_S1_headon.png"))
    # S2: 90-degree crossing, exact collision geometry
    out.append(scenario("S2  90-degree crossing, B from the right, 110 kt each",
        make("A", 0xA1, 0, 0, 4500, 0, 110), make("B", 0xB2, 4, 4, 4500, 270, 110),
        "A has B on its right, so A gives way (turns right, passes behind B)",
        "acas_poc_S2_crossing.png"))
    # S3: parallel, 1 nm apart
    out.append(scenario("S3  Parallel tracks, 1 nm apart, same speed",
        make("A", 0xA1, 0, 0, 4500, 0, 110), make("B", 0xB2, 1.0, 0, 4500, 0, 110),
        "PROXIMATE only, no TA or RA"))
    # S4: head-on but 1,000 ft vertical separation
    out.append(scenario("S4  Head-on, 1,000 ft vertical separation",
        make("A", 0xA1, 0, 0, 4500, 0, 120), make("B", 0xB2, 0.054, 6, 5500, 180, 120),
        "PROXIMATE only; vertical separation already safe"))
    # S5: fast head-on, but 1.2 nm lateral offset
    out.append(scenario("S5  Fast pass, 1.2 nm lateral offset (short tau, large miss)",
        make("A", 0xA1, 0, 0, 4500, 0, 140), make("B", 0xB2, 1.2, 6, 4500, 180, 140),
        "No TA/RA even though tau gets small: miss distance check filters it"))
    # S6: overtaking, slow closure
    out.append(scenario("S6  Overtaking, 25 kt closure, 30 m offset",
        make("A", 0xA1, 0, 0, 4500, 0, 100), make("B", 0xB2, 0.016, -1.0, 4500, 0, 125),
        "tau stays long; DMOD range guard triggers the RA"))
    # S7: low performance, near ground, hot day
    out.append(scenario("S7  Low performance near ground: 250 fpm climb, 1,200 ft AGL",
        make("A", 0xA1, 0, 0, 2300, 0, 100, climb_fpm=250, ground_ft=1100),
        make("B", 0xB2, 0.054, 6, 2300, 180, 100, climb_fpm=250, ground_ft=1100),
        "Descents vetoed by GCAS, weak climb penalized, both turn right",
        "acas_poc_S7_lowperf.png"))
    # S8a/b: same geometry, fresh vs stale data (500 m predicted miss)
    out.append(scenario("S8a Head-on, 500 m predicted miss, FRESH data",
        make("A", 0xA1, 0, 0, 4500, 0, 120), make("B", 0xB2, 0.27, 6, 4500, 180, 120),
        "TA only: 500 m is outside the 457 m RA zone"))
    out.append(scenario("S8b Same geometry, B's packets 8 s OLD",
        make("A", 0xA1, 0, 0, 4500, 0, 120), make("B", 0xB2, 0.27, 6, 4500, 180, 120, packet_age=8.0),
        "Uncertainty inflates the zone past 500 m -> RA: loss of confidence never looks like confidence"))
    # S9: traffic pattern turn (snapshot only)
    print("=" * 92); print("S9  Traffic pattern: B turning right at 3 deg/s toward a parallel track")
    print("=" * 92)
    print("  Expected: straight-line CPA says conflict, curved prediction says safe -> no RA")
    snapshot(make("A", 0xA1, 0, 0, 1800, 0, 80),
             make("B", 0xB2, 1.2, 1.0, 1800, 270, 90, turn_dps=3.0))
    print()

    # ------------------------------------------------------------ CUSTOM SCENARIO
    # Edit and uncomment to try your own numbers:
    # out.append(scenario("CUSTOM",
    #     make("A", 0xA1, east_nm=0, north_nm=0, alt_ft=3500, hdg_deg=45, speed_kt=105),
    #     make("B", 0xB2, east_nm=3, north_nm=2, alt_ft=3600, hdg_deg=270, speed_kt=115, vs_fpm=-300),
    #     "your expectation here"))

    print("=" * 92); print("SUMMARY  (minimum horizontal separation / vertical at that moment)")
    print("=" * 92)
    for title, base, w in out:
        f = lambda r: f"{r['min_h']:6.0f} m /{r['v_at']/FT:5.0f} ft{' NMAC' if r['nmac'] else '     '}"
        print(f"  {title[:52]:<52}  no ACAS {f(base)}   ACAS {f(w)}")


# ==================================================================== multi-aircraft selection
import itertools


def legal_single(m, ac):
    if m == "DESCEND":
        rate = min(CFG["VS_CMD_FPM"] * FPM, ac.max_desc)
        agl_after = (ac.h - ac.ground) - rate * CFG["GCAS_LOOKAHEAD_S"]
        if agl_after < CFG["GROUND_FLOOR_FT"] * FT:
            return False, f"GCAS veto: {ac.name} descent would reach {agl_after / FT:.0f} ft AGL"
    if m == "CLIMB" and ac.max_climb < 150 * FPM:
        return False, f"{ac.name} cannot climb"
    return True, ""


def legal_pair(ma, mb, a, b, is_ho, a_fixed, b_fixed):
    if ma == mb and ma in ("CLIMB", "DESCEND") and not (a_fixed or b_fixed):
        return False, "same vertical sense"
    if is_ho and "TURN_LEFT" in (ma, mb):
        return False, "head-on: 14 CFR 91.113(e) requires turning right"
    if is_ho and not (a_fixed or b_fixed) and {ma, mb} == {"TURN_RIGHT", "MAINTAIN"}:
        return False, "head-on: both aircraft must alter course to the right"
    return True, ""


def select_multi(acs, delay, t_now=0.0, prefer=None, fixed=None, allowed=None):
    """Joint maneuver choice for 2 or more aircraft. acs[0] is the computing aircraft (own state
    exact), the others are estimates from LoRa packets and are sampled with their uncertainty.
    fixed   {name: maneuver}  aircraft that cannot negotiate (transmit-only) or whose pilot took over
    allowed {name: set}       restricted sets used when no messages can be exchanged
    Returns plan {name: maneuver}, best, results, vetoes, master name."""
    fixed, allowed = fixed or {}, allowed or {}
    n = CFG["N_SAMPLES"]
    own = acs[0]
    order = sorted(acs, key=lambda a: a.icao)
    seed = sum(a.icao * (i + 1) * 7919 for i, a in enumerate(order)) + int(t_now)
    rng = np.random.default_rng(seed)
    horizon = max(CFG["HORIZON_S"], delay + 45.0)
    base = {}
    for a in order:
        S = arrays(a, n)
        sp, sv, sz = rel_sigma(own, a)
        e = [rng.normal(0, sp, n), rng.normal(0, sp, n), rng.normal(0, sz, n),
             rng.normal(0, sv, n), rng.normal(0, sv, n)]
        if a is not own:
            S[0], S[1], S[2] = S[0] + e[0], S[1] + e[1], S[2] + e[2]
            vx, vy = a.vx + e[3], a.vy + e[4]
            S[3], S[4] = np.arctan2(vx, vy), np.hypot(vx, vy)
        base[a.name] = S
    names = [a.name for a in acs]
    by = {a.name: a for a in acs}
    opts = {}
    menu = MANEUVERS if len(acs) <= 3 else [m for m in MANEUVERS if m != "TURN_LEFT"]   # keeps 5 aircraft fast
    for a in acs:
        if a.name in fixed:
            opts[a.name] = [fixed[a.name]]
        else:
            opts[a.name] = [m for m in menu if m in allowed.get(a.name, MANEUVERS)]
    traj = {}
    for a in acs:
        for m in set(opts[a.name]) | {"MAINTAIN"}:
            traj[(a.name, m)] = propagate(*[v.copy() for v in base[a.name]], cmd=maneuver_cmd(a, m, n),
                                          delay=delay, horizon=horizon)
    nm_h, nm_v = CFG["NMAC_H_FT"] * FT, CFG["NMAC_V_FT"] * FT
    z_h, z_v = CFG["RA_H_FT"] * FT, CFG["RA_V_FT"] * FT
    ta_h, ta_v = CFG["TA_H_NM"] * NM, CFG["TA_V_FT"] * FT
    pairs = list(itertools.combinations(names, 2))
    conflict, pair_hits, headons = {}, {}, {}
    for i, j in pairs:
        A0, B0 = traj[(i, "MAINTAIN")], traj[(j, "MAINTAIN")]
        dh = np.hypot(A0[0] - B0[0], A0[1] - B0[1]); dv = np.abs(A0[2] - B0[2])
        conflict[(i, j)] = bool(np.any((dh < ta_h) & (dv < ta_v)))
        headons[(i, j)] = headon(by[i], by[j])
        for mi in opts[i]:
            for mj in opts[j]:
                A, B = traj[(i, mi)], traj[(j, mj)]
                dh = np.hypot(A[0] - B[0], A[1] - B[1]); dv = np.abs(A[2] - B[2])
                depth = np.maximum(dh / z_h, dv / z_v).min(axis=0)       # < 1 means inside the zone
                pair_hits[(i, j, mi, mj)] = (np.any((dh < nm_h) & (dv < nm_v), axis=0),
                                             np.any((dh < z_h) & (dv < z_v), axis=0),
                                             depth)
    gw = {}
    for a in acs:
        partners = [b for (i, j) in pairs if conflict[(i, j)] and a.name in (i, j)
                    for b in (i, j) if b != a.name]
        gw[a.name] = any(gives_way(a, by[b], headons[tuple(sorted((a.name, b), key=names.index))])
                         for b in partners) if partners else True
    results, vetoes = [], []
    for k, combo in enumerate(itertools.product(*[opts[nm] for nm in names])):
        plan = dict(zip(names, combo))
        why = ""
        for nm in names:
            if nm not in fixed:
                ok, r = legal_single(plan[nm], by[nm])
                if not ok: why = r; break
        if not why:
            for i, j in pairs:
                if conflict[(i, j)]:
                    ok, r = legal_pair(plan[i], plan[j], by[i], by[j], headons[(i, j)], i in fixed, j in fixed)
                    if not ok: why = r; break
        if why:
            vetoes.append((plan, why)); continue
        nm_any, zn_any, sep = np.zeros(n, bool), np.zeros(n, bool), np.full(n, 9.0)
        for i, j in pairs:
            h1, h2, dep = pair_hits[(i, j, plan[i], plan[j])]
            nm_any |= h1; zn_any |= h2; sep = np.minimum(sep, dep)
        p_nmac, p_zone = float(nm_any.mean()), float(zn_any.mean())
        intrusion = float(np.mean(np.maximum(0.0, 1.0 - sep)))          # how deep into the zone, 0..1
        score = (1000 * p_nmac + 25 * p_zone + 60 * intrusion
                 + sum(cost(plan[nm], by[nm], gw[nm]) for nm in names if nm not in fixed))
        if prefer and all(prefer.get(nm) == plan[nm] for nm in names):
            score -= 0.5
        results.append(dict(plan=plan, p_nmac=p_nmac, p_zone=p_zone, intrusion=intrusion, score=score, order=k))
    if not results:      # nothing legal: fall back to everyone turning right where possible
        plan = {nm: (fixed.get(nm) or "TURN_RIGHT") for nm in names}
        results = [dict(plan=plan, p_nmac=1.0, p_zone=1.0, intrusion=1.0, score=9999, order=0)]
    results.sort(key=lambda r: (r["score"], r["order"]))
    return results[0]["plan"], results[0], results, vetoes, order[0].name
