"""
run_demo.py - Honeywell Aerospace x Devils Invent 2026 (ASU Ira A. Fulton Schools of Engineering), PS2 demo: Cessna 172 Skyhawks with LoRa P2P collision avoidance

  python run_demo.py                      # start menu, FlightGear + dashboard
  python run_demo.py --no-fg              # dashboard only
  python run_demo.py --headless --all     # fast test of every scenario, text only
  python run_demo.py --headless --scenario S2 --aircraft 5 --offset 0
"""
import argparse
import itertools
import math
import time
from acas_core import NM, FT, CFG
from sim import Plane, FGMultiplayer
from tcas_node import Node, Link

NAMES = "ABCDE"
ICAO = {n: 0xA1 + 0x11 * i for i, n in enumerate(NAMES)}
COLLISION_H_M, COLLISION_V_M = 11.0, 3.0     # one Cessna 172 wingspan / airframe height

# start-up parameters (editable in the start menu): name -> (default, min, max, step, label)
PARAMS = {
    "aircraft":  (3, 3, 5, 1, "Aircraft in scenario 2"),
    "cruise":    (124, 90, 130, 2, "Cruise speed (kt TAS)"),
    "offset":    (0, 0, 400, 25, "Lateral offset (m), 0 = direct collision course"),
    "countdown": (3, 1, 8, 1, "Pilot response window (s)"),
    "loss":      (0, 0, 60, 10, "LoRa packet loss (%)"),
}
TITLES = {
    "S1": "Head-on: two Skyhawks, both units negotiate",
    "S2": "Converging traffic: {n} Skyhawks heading for one point",
    "S3": "One-way link: Skyhawk B transmits only (no receiver), only A can act",
}


def default_params():
    return {k: v[0] for k, v in PARAMS.items()}


def build(key, p):
    """Aircraft list: (name, east_nm, north_nm, alt_ft, hdg_deg, kt, climb_fpm, desc_fpm, has_receiver)."""
    kt, off = p["cruise"], p["offset"] / NM
    if key in ("S1", "S3"):
        return [("A", 0, 0, 4500, 0, kt, 400, 700, True),
                ("B", off, 6.0, 4500, 180, kt, 400, 700, key == "S1")]
    n = p["aircraft"]
    cx, cy, R = 0.0, 3.0, 3.0           # everyone reaches (0, 3 nm) at the same moment
    out = []
    for i in range(n):
        brg = math.radians(180 + i * 360 / n)          # A starts due south of the point
        e, nn = cx + R * math.sin(brg), cy + R * math.cos(brg)
        hdg = (math.degrees(brg) + 180) % 360
        lat_off = off if i == 1 else 0.0               # offset applies to B
        out.append((NAMES[i], e + lat_off * math.cos(math.radians(hdg)), nn - lat_off * math.sin(math.radians(hdg)),
                    4500, hdg, kt, 400, 700, True))
    return out


class Demo:
    FRAME = 0.1            # seconds of sim per update
    TX_PERIOD = 0.2        # 5 Hz state packets in GNSS-timed TDMA slots
    MP_WALL_PERIOD = 0.1   # FlightGear multiplayer: 10 per real second at any sim speed

    def __init__(self, key, fg=False, acas=True, t_max=200.0, params=None):
        self.key, self.params = key, dict(params or default_params())
        CFG["COUNTDOWN_S"] = float(self.params["countdown"])
        specs = build(key, self.params)
        self.title = TITLES[key].format(n=len(specs))
        self.acas, self.t_max = acas, t_max
        self.link, self.log, self.msgs, self.history = Link(), [], [], []
        self.link.loss = self.params["loss"] / 100.0
        self.planes, self.nodes = [], []
        for i, (nm, e, n, alt, hdg, kt, cl, de, rx) in enumerate(specs):
            pl = Plane(nm, ICAO[nm], e, n, alt, hdg, kt, climb_fpm=cl, desc_fpm=de,
                       fg_port=5550 if (fg and i == 0) else None)
            self.planes.append(pl)
            self.nodes.append(Node(pl, self.link, self.log, self.msgs, can_receive=rx))
        for i, nd in enumerate(self.nodes):
            nd.peers = [q for q in self.nodes if q is not nd]
            nd.slot = i
            nd.next_tx = i * self.TX_PERIOD / len(self.nodes)
        self.A, self.B = self.planes[0], self.planes[1]
        self.nA, self.nB = self.nodes[0], self.nodes[1]
        self.mp = [FGMultiplayer(f"ACFT-{p.name}") for p in self.planes[1:]] if fg else []
        self._mp_wall = 0.0
        self.t, self.next_tick = 0.0, 0.0
        self.min_h, self.v_at, self.nmac, self.done_at = math.inf, 0.0, False, None
        self.nmac_t, self.collision = None, None
        self.pair_min = {}
        self.paused = False

    # ---------------------------------------------------------- pilot of Skyhawk A
    def pilot_takeover(self):
        if not self.nA.pilot_cancel:
            self.nA.pilot_cancel = True
            self.A.command("MANUAL")
            self.log.append((self.t, "A", "USER button: pilot flying (arrow keys)"))

    def pilot_input(self, roll, pitch):
        if self.nA.pilot_cancel and not self.A.crashed:
            self.A.set_input(roll, pitch)

    # ---------------------------------------------------------- simulation
    def link_state(self):
        if not self.link.up: return "lost"
        if any(not n.can_receive for n in self.nodes): return "oneway"
        if not self.link.coord_up: return "coordlost"
        return "ok"

    def update(self):
        if self.paused:
            return
        if self.collision is None:
            for nd in self.nodes:
                if self.t >= nd.next_tx - 1e-9:
                    nd.transmit(self.t)
                    nd.next_tx += self.TX_PERIOD
            if self.acas and self.t >= self.next_tick - 1e-9:
                for nd in sorted(self.nodes, key=lambda q: q.icao):
                    nd.tick(self.t)
                self.next_tick += 1.0
        for pl in self.planes:
            pl.step(self.FRAME)
        self.t += self.FRAME
        if self.mp and time.monotonic() >= self._mp_wall:
            for mp, pl in zip(self.mp, self.planes[1:]):
                if not pl.crashed:              # a crashed aircraft disappears from FlightGear
                    mp.send(pl)
            self._mp_wall = time.monotonic() + self.MP_WALL_PERIOD
        st = {p.name: p.state() for p in self.planes}
        opening = True
        for a, b in itertools.combinations(st, 2):
            sa, sb = st[a], st[b]
            dh, dv = math.hypot(sa.x - sb.x, sa.y - sb.y), abs(sa.h - sb.h)
            if dh < self.pair_min.get((a, b), (math.inf,))[0]:
                self.pair_min[(a, b)] = (dh, dv)
            if dh < self.min_h:
                self.min_h, self.v_at = dh, dv
            if dh < CFG["NMAC_H_FT"] * FT and dv < CFG["NMAC_V_FT"] * FT and not self.nmac:
                self.nmac, self.nmac_t = True, self.t
                self.log.append((self.t, "-", f"NEAR MIDAIR COLLISION {a}-{b}: {dh:.0f} m / {dv / FT:.0f} ft"))
            if dh < COLLISION_H_M and dv < COLLISION_V_M and self.collision is None:
                self.collision = dict(t=self.t, names=(a, b), x=(sa.x + sb.x) / 2, y=(sa.y + sb.y) / 2,
                                      h=(sa.h + sb.h) / 2, wall=time.monotonic())
                for p in self.planes:
                    if p.name in (a, b):
                        p.crashed = True
                self.log.append((self.t, "-", f"MIDAIR COLLISION {a}-{b}"))
                self.done_at = self.t
            if not ((sa.x - sb.x) * (sa.vx - sb.vx) + (sa.y - sb.y) * (sa.vy - sb.vy) > 0 and dh > 3000):
                opening = False
        if not self.history or self.t - self.history[-1][0] >= 0.5 - 1e-9:
            self.history.append((self.t, self.link_state(),
                                 {n.name: (n.level if n.state == "MONITOR" else 3 if n.state in ("RA", "ENGAGED") else 0,
                                           n.state, n.sel_t, n.coord_t, n.gcas_t, n.coord_level)
                                  for n in self.nodes}))
        if self.done_at is None and opening and self.t > 30:
            self.done_at = self.t

    @property
    def finished(self):
        return self.t >= self.t_max or (self.done_at is not None and self.t > self.done_at + 10)

    def toggle_link(self):
        self.link.up = not self.link.up
        self.log.append((self.t, "-", "LoRa LINK " + ("RESTORED" if self.link.up else "LOST")))

    def toggle_coord(self):
        self.link.coord_up = not self.link.coord_up
        self.log.append((self.t, "-", "COORD MSGS " + ("RESTORED" if self.link.coord_up else "LOST")))

    def result(self):
        closest = sorted(self.pair_min.items(), key=lambda kv: kv[1][0])[:3]
        parts = [f"{a}-{b} {d:5.0f} m / {v / FT:3.0f} ft" for (a, b), (d, v) in closest]
        tag = "   COLLISION!" if self.collision else ("   NMAC!" if self.nmac else "")
        return "   ".join(parts) + tag


def headless(keys, params):
    for k in keys:
        print("=" * 100); d0 = Demo(k, params=params); print(f"{k}  {d0.title}"); print("=" * 100)
        base = Demo(k, acas=False, params=params)
        while not base.finished:
            base.update()
        d = Demo(k, params=params)
        while not d.finished:
            d.update()
        for t, who, txt in d.log:
            if "COORD" in txt or "COLLISION" in txt or "CLEAR" in txt or "ENGAGED" in txt:
                print(f"  t={t:6.1f}s  [{who}] {txt}")
        print(f"  RESULT  no system: {base.result()}")
        print(f"          with system: {d.result()}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="S1", choices=list(TITLES))
    ap.add_argument("--no-fg", action="store_true", help="do not send to FlightGear")
    ap.add_argument("--headless", action="store_true", help="fast run, text output only")
    ap.add_argument("--all", action="store_true", help="with --headless: run every scenario")
    ap.add_argument("--sim-speed", type=float, default=1.0, help="starting time multiplier")
    for k, (dflt, lo, hi, stp, lab) in PARAMS.items():
        ap.add_argument(f"--{k}", type=int, default=dflt, help=f"{lab} [{lo}..{hi}]")
    args = ap.parse_args()
    params = {k: max(PARAMS[k][1], min(PARAMS[k][2], getattr(args, k))) for k in PARAMS}
    if args.headless:
        headless(list(TITLES) if args.all else [args.scenario], params)
        return
    import ui
    ui.run(Demo, TITLES, args.scenario, fg=not args.no_fg, speed=args.sim_speed, params=params)


if __name__ == "__main__":
    main()
