"""
tcas_node.py - one onboard unit (Heltec Wireless Tracker) per aircraft, any number of peers

Every cycle, all real and recorded for the display:
  own GNSS state -> 33-byte STATE packet (CRC) -> LoRa -> each peer that has a receiver decodes it
  -> tracker per peer (dead-reckoning, uncertainty grows with packet age)
  -> threat logic per peer (TCAS II-style tau / DMOD / predicted miss)
  -> conflict cluster -> joint maneuver search (ACAS X-style) for everyone in the cluster
  -> negotiation over LoRa (12-byte PROPOSE / ACK / COMMIT / MANUAL / CLEAR)
  -> countdown -> autopilot

Coordination levels
  1 explicit    handshake with every negotiating peer succeeded
  2 implicit    peers' states arrive but the handshake did not: rule-complementary maneuvers
  3 independent peer data stale: rule-complementary maneuvers, enlarged zone
  4 one-way     peer only transmits (its packets say it has no receiver): it cannot know about us,
                so we plan alone, holding it to its broadcast intent
A transmit-only unit (can_receive=False) never sees traffic, so it never alerts or maneuvers.
"""
import math
import random
from collections import deque
from acas_core import CFG, CALLOUT, LEVELS, Aircraft, assess, select_multi, step, FT, FPM, G, D2R
import packet as pk
from sim import latlon_to_local

STALE_S = 2.5          # peer state older than this -> Level 3
AP_LAG_S = 1.5         # autopilot roll-in latency added to the countdown in predictions
F_RX = 16              # packet flag bit: unit has a receiver and can negotiate
COORD_NAME = {0: "none", 1: "L1 explicit", 2: "L2 implicit", 3: "L3 independent", 4: "one-way"}


class Link:
    """LoRa channel. Carries raw bytes to a receiver only if that receiver exists.
    up = channel works, coord_up = coordination messages get through, loss = random loss."""
    def __init__(self):
        self.up, self.coord_up, self.loss = True, True, 0.0
        self.sent = self.delivered = self.dropped = 0

    def carry(self, data, receiver, coordination=False):
        self.sent += 1
        ok = (self.up and receiver.can_receive and (self.coord_up or not coordination)
              and random.random() >= self.loss)
        if ok: self.delivered += 1
        else: self.dropped += 1
        return ok


class Node:
    def __init__(self, plane, link, log, msgs, can_receive=True):
        self.plane, self.link, self.log, self.msgs = plane, link, log, msgs
        self.name, self.icao, self.can_receive = plane.name, plane.icao, can_receive
        self.peers = []
        self.rx = {}                     # peer name -> dict(state, time, bytes, decoded, times)
        self.assess = {}                 # peer name -> assessment
        self.primary = None              # most urgent peer
        self.state = "MONITOR"           # MONITOR, RA, ENGAGED, PILOT, CLEAR
        self.plan, self.coord_level, self.cluster = None, 0, []
        self.level, self.ra_t, self.callout, self.say = -1, None, "", []
        self.pilot_cancel, self.manual_peers = False, {}
        self.tx_seq, self.tx_last_t, self.tx_last_bytes, self.crc_fail = 0, -9, b"", 0
        self.sel, self.sel_t, self.coord_t, self.gcas_t, self.threat_t = None, -9, -9, -9, -9
        self.say_t, self.ra_dmin, self.next_tx, self.slot = -9, None, 0.0, 0
        self._neg_cluster = None

    # ---------------------------------------------------------- helpers
    @property
    def assessment(self):
        return self.assess.get(self.primary)

    def peer(self, name):
        return next(p for p in self.peers if p.name == name)

    def flags(self):
        return (1 | 2 | (4 if self.state == "ENGAGED" else 0) | (8 if self.state == "PILOT" else 0)
                | (F_RX if self.can_receive else 0))

    def peer_can_negotiate(self, name):
        d = self.rx.get(name, {}).get("decoded")
        return bool(d and d["flags"] & F_RX)

    def peer_intent(self, name):
        d = self.rx.get(name, {}).get("decoded")
        return (d and d["intent"]) or "MAINTAIN"

    def packet_age(self, t, name=None):
        name = name or self.primary
        r = self.rx.get(name)
        return None if r is None else t - r["time"]

    def last_rx_t(self):
        return max((r["rx_t"] for r in self.rx.values()), default=-9)

    def rx_rate(self, name=None):
        r = self.rx.get(name or self.primary)
        if not r or len(r["times"]) < 2: return 0.0
        return (len(r["times"]) - 1) / max(1e-6, r["times"][-1] - r["times"][0])

    # ---------------------------------------------------------- LoRa
    def transmit(self, t):
        f, p = self.plane.fdm, self.plane
        psi, gs = f["attitude/psi-rad"], f["velocities/vg-fps"] * FT
        intent = self.plan[self.name] if self.plan and self.state in ("RA", "ENGAGED") else "MAINTAIN"
        data = pk.encode_state(self.icao, self.tx_seq, t, p.lat, p.lon, f["position/h-sl-ft"],
                               gs * math.cos(psi), gs * math.sin(psi), f["velocities/h-dot-fps"] * FT,
                               p.turn_rate_dps(), 5.0, 0.3, self.flags(), intent)
        self.tx_seq += 1
        self.tx_last_t, self.tx_last_bytes = t, data
        for q in self.peers:
            if self.link.carry(data, q):
                q.receive(self, data, t)

    def receive(self, sender, data, t):
        d = pk.decode_state(data)
        if d is None:
            self.crc_fail += 1
            return
        x, y = latlon_to_local(d["lat"], d["lon"])
        gs = math.hypot(d["vn"], d["ve"])
        bank = math.atan(d["turn_dps"] * D2R * gs / G) if gs > 1 else 0.0
        st = Aircraft(sender.name, d["icao"], x, y, d["alt_ft"] * FT, math.atan2(d["ve"], d["vn"]), gs,
                      d["vs"], bank, sender.plane.climb_fpm * FPM, sender.plane.desc_fpm * FPM,
                      0.0, d["sig_pos"], d["sig_vel"])
        r = self.rx.setdefault(sender.name, dict(times=deque(maxlen=12)))
        r.update(state=st, time=d["t"], bytes=data, decoded=d, rx_t=t)
        r["times"].append(t)

    def estimate(self, name, t):
        r = self.rx.get(name)
        if r is None:
            return None
        s = r["state"]
        est = Aircraft(**{k: getattr(s, k) for k in s.__dataclass_fields__})
        age = t - r["time"]
        for _ in range(int(age / 0.5)):
            step(est, 0.5)
        est.packet_age = age
        return est

    def peer_estimate(self, t):
        return self.estimate(self.primary, t) if self.primary else None

    def send_coord(self, t, to, typ, man_self, man_peer, note=""):
        data = pk.encode_coord(typ, self.icao, to.icao, int(t), man_self, man_peer)
        ok = self.link.carry(data, to, coordination=True) and pk.decode_coord(data) is not None
        self.msgs.append((t, self.name, to.name, pk.TYPE_NAME[typ], pk.hexdump(data), ok,
                          f"{man_self or ''}/{man_peer or ''} {note}".strip(" /")))
        self.coord_t = t
        if ok: to.coord_t = t
        return ok

    # ---------------------------------------------------------- events
    def event(self, t, text, speak=None):
        self.log.append((t, self.name, text))
        if speak:
            self.say.append(speak); self.say_t = t

    def _set_plan(self, plan, level, t, how):
        self.plan, self.coord_level = plan, level
        self.event(t, f"COORD {COORD_NAME[level]} {how}: " + ", ".join(f"{k} {v}" for k, v in plan.items()))

    def _select(self, acs, delay, t, mode, fixed=None, allowed=None):
        plan, best, results, vetoes, master = select_multi(acs, delay, t, prefer=self.plan,
                                                           fixed=fixed, allowed=allowed)
        self.sel = dict(t=t, delay=delay, mode=mode, plan=plan, best=best, results=results,
                        vetoes=vetoes, master=master, n=CFG["N_SAMPLES"], pairs=len(results),
                        names=[a.name for a in acs], fixed=dict(fixed or {}))
        self.sel_t = t
        if any("GCAS" in v[1] for v in vetoes):
            self.gcas_t = t
        return plan

    def rule_set(self, names):
        """Maneuvers complementary by rule when no messages can be exchanged: everyone may turn
        right; vertically the lowest ID may descend, the highest may climb."""
        icao = {self.name: self.icao, **{p.name: p.icao for p in self.peers}}
        order = sorted(names, key=lambda nm: icao[nm])
        rs = {nm: {"TURN_RIGHT"} for nm in order}
        rs[order[0]].add("DESCEND"); rs[order[-1]].add("CLIMB")
        return rs

    # ---------------------------------------------------------- decision cycle (1 Hz)
    def tick(self, t):
        if not self.can_receive or not self.rx:
            self.level = -1                  # a transmit-only unit never learns about traffic
            return
        own = self.plane.state()
        ests = {nm: self.estimate(nm, t) for nm in self.rx}
        self.assess = {nm: assess(own, e) for nm, e in ests.items()}
        self.threat_t = t
        key = lambda nm: (self.assess[nm]["level"], -self.assess[nm]["tau_mod"])
        self.primary = max(self.assess, key=key)
        s = self.assess[self.primary]
        lvl = s["level"]
        if self.state == "MONITOR":
            self.cluster = sorted(nm for nm, a in self.assess.items() if a["level"] >= 2)
            if lvl != self.level and lvl in (-1, 0, 1):
                self.event(t, f"{LEVELS[lvl]} {self.primary} range {s['range_m'] / 1852:.2f} nm")
            if lvl == 2 and self.level != 2:
                self.event(t, f"TA {self.primary} range {s['range_m'] / 1852:.2f} nm tau {s['tau_mod']:.0f} s",
                           speak="TRAFFIC")
            if lvl == 2 and self._is_master() and self.cluster != self._neg_cluster:
                # pre-negotiate during the TA, and again whenever another aircraft joins the conflict
                lead = max(0.0, s["tau_mod"] - CFG["RA_TAU"]) + CFG["COUNTDOWN_S"]
                self._negotiate(own, ests, lead, t, "pre-negotiated")
                self._neg_cluster = list(self.cluster)
            if lvl == 3:
                self._resolve(own, ests, t)
            self.level = lvl
        if self.state in ("RA", "ENGAGED") and self.pilot_cancel:
            # USER button: the pilot flies; the system keeps alerting and showing the advisory
            self.state = "PILOT"
            self.event(t, "USER button: pilot took control -> MANUAL sent", speak="PILOT")
            self._send_manual(t)
        elif self.state == "RA":
            if t >= self.ra_t + CFG["COUNTDOWN_S"]:
                self.state = "ENGAGED"
                self.plane.command(self.plan[self.name])
                self.event(t, f"no pilot response -> AUTOPILOT ENGAGED {self.plan[self.name]}", speak="ENGAGED")
        if self.state in ("ENGAGED", "PILOT"):
            members = [nm for nm in self.cluster if nm in self.assess] or [self.primary]
            if all(self.assess[nm]["range_rate"] > 0 and self.assess[nm]["range_m"] > self.assess[nm]["ra_zone_h"]
                   for nm in members):
                if self.state == "ENGAGED":
                    self.plane.command("RESUME")        # a pilot who took over keeps flying
                self.state, self.level = "CLEAR", 0
                self.event(t, "CLEAR OF CONFLICT", speak="CLEAR")
                for nm in members:
                    if self.peer_can_negotiate(nm):
                        self.send_coord(t, self.peer(nm), pk.T_CLEAR, None, None)

    def _coop(self, names):
        return [nm for nm in names if self.peer_can_negotiate(nm)]

    def _is_master(self):
        coop = self._coop(self.cluster)
        icao = {p.name: p.icao for p in self.peers}
        return all(self.icao < icao[nm] for nm in coop)

    def _fixed(self, names):
        fx = {nm: self.peer_intent(nm) for nm in names if not self.peer_can_negotiate(nm)}
        fx.update({nm: m for nm, m in self.manual_peers.items() if nm in names})
        return fx

    def _negotiate(self, own, ests, delay, t, how):
        names = self.cluster or [self.primary]
        acs = [own] + [ests[nm] for nm in names]
        coop, fixed = self._coop(names), self._fixed(names)
        if not coop:
            plan = self._select(acs, delay, t, "one-way, peers held to broadcast intent", fixed)
            self._set_plan(plan, 4, t, f"{how}: {', '.join(names)} cannot receive, we act alone")
            return
        plan = self._select(acs, delay, t, "L1 full search", fixed)
        ok = True
        for nm in coop:
            q = self.peer(nm)
            ok = ok and self.send_coord(t, q, pk.T_PROPOSE, plan[self.name], plan[nm], how) \
                    and q.send_coord(t, self, pk.T_ACK, plan[nm], plan[self.name]) \
                    and self.send_coord(t, q, pk.T_COMMIT, plan[self.name], plan[nm])
        if ok:
            self._set_plan(plan, 1, t, f"{how} (PROPOSE->ACK->COMMIT)")
            for nm in coop:
                q = self.peer(nm)
                q.plan, q.coord_level = plan, 1
                if q.state == "RA": q.callout = CALLOUT[plan[nm]]
        else:
            plan = self._select(acs, delay, t, "L2 rule set", fixed, self.rule_set([self.name] + names))
            self._set_plan(plan, 2, t, f"{how} (handshake lost)")

    def _resolve(self, own, ests, t):
        delay = CFG["COUNTDOWN_S"] + AP_LAG_S
        names = self.cluster or [self.primary]
        acs = [own] + [ests[nm] for nm in names]
        coop, fixed = self._coop(names), self._fixed(names)
        stale = any(ests[nm].packet_age > STALE_S for nm in names)
        if stale:
            plan = self._select(acs, delay, t, "L3 rule set", fixed, self.rule_set([self.name] + names))
            self._set_plan(plan, 3, t, "(peer data stale)")
        elif not coop or self._is_master():
            self._negotiate(own, ests, delay, t, "RA")
        elif not (self.plan and self.coord_level == 1 and all(nm in self.plan for nm in names)):
            plan = self._select(acs, delay, t, "L2 rule set", fixed, self.rule_set([self.name] + names))
            self._set_plan(plan, 2, t, "(no PROPOSE received yet)")
        self.state, self.ra_t = "RA", t
        self.ra_dmin = self.assessment["d_min_curved"]
        self.callout = CALLOUT[self.plan[self.name]]
        self.event(t, f"RA \"{self.callout}\" countdown {CFG['COUNTDOWN_S']:.0f} s", speak=self.plan[self.name])

    def _send_manual(self, t):
        for nm in self._coop(self.cluster):
            q = self.peer(nm)
            if self.send_coord(t, q, pk.T_MANUAL, "MAINTAIN", None, "pilot in control"):
                q.manual_peers[self.name] = "MAINTAIN"
                q.event(t, f"MANUAL received from {self.name}: re-planning around it")
                own = q.plane.state()
                names = q.cluster or [self.name]
                ests = {n2: q.estimate(n2, t) for n2 in names}
                plan = q._select([own] + [ests[n2] for n2 in names], AP_LAG_S, t, "re-plan, pilot fixed",
                                 q._fixed(names))
                q._set_plan(plan, 1, t, "re-plan after MANUAL")
                q.callout = CALLOUT[plan[q.name]]
                if q.state == "ENGAGED":
                    q.plane.command(plan[q.name])
