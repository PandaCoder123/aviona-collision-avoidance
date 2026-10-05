"""
ui.py - live demo dashboard (1600x920, drawn 1:1 for sharp text)

  LEFT    Hardware, always visible, one column per Skyhawk: UC6580 GNSS, ESP32-S3, SX1262 LoRa,
          TFT, audio out, USER button, autopilot servos; last LoRa packet; timeline
  CENTER  Cockpit traffic display (radar, callouts, voice, countdown ring) and the live
          calculations: only the equations deciding the outcome right now
  RIGHT   Third-person view; ESP32-S3 software modules that appear only while running

Keys: P start/pause, + / - speed (0.1x..16x), 0 = 1x, N step 1 s, S slow-motion during RA,
      SPACE = USER button on Skyhawk A, L cut LoRa link, K cut coordination messages,
      R restart, 1-3 scenario, ESC quit.
"""
import math
import os
import sys
import time
import numpy as np
import pygame
from acas_core import NM, FT, CFG, CALLOUT, LEVELS
from tcas_node import COORD_NAME

CW, CH = 1600, 920
BG, PANEL, CARD = (8, 12, 16), (13, 19, 26), (21, 30, 40)
GREY, DIM, WHITE = (120, 130, 140), (48, 56, 66), (238, 242, 246)
CYAN, AMBER, RED, GREEN = (0, 200, 225), (255, 176, 0), (238, 64, 52), (52, 206, 104)
VIOLET = (172, 124, 255)
AC_COL = {"A": (92, 156, 255), "B": (216, 84, 172), "C": (255, 150, 70), "D": (90, 220, 170), "E": (225, 205, 120)}
SPEEDS = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0]
LORA_SENS, LORA_TX, F_MHZ = -117.0, 21.0, 915.0
SHORT = {"TURN_RIGHT": "RIGHT", "TURN_LEFT": "LEFT", "CLIMB": "CLIMB", "DESCEND": "DESCEND", "MAINTAIN": "HOLD"}
PHRASES = {"TRAFFIC": "Traffic, traffic", "TURN_RIGHT": "Turn right, turn right",
           "TURN_LEFT": "Turn left, turn left", "CLIMB": "Climb, climb",
           "DESCEND": "Descend, descend", "MAINTAIN": "Monitor, maintain course",
           "ENGAGED": "Autopilot engaged", "PILOT": "Pilot in control", "CLEAR": "Clear of conflict"}


def make_dpi_aware():
    """Windows: draw at physical pixels instead of being stretched by display scaling (blurry)."""
    if sys.platform == "win32":
        try:
            import ctypes
            try:
                ctypes.windll.shcore.SetProcessDpiAwareness(2)
            except Exception:
                ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass


# ------------------------------------------------------------------ audio
class Audio:
    def __init__(self):
        self.ok, self.sounds, self.queue = False, {}, []
        try:
            pygame.mixer.init(22050, -16, 1, 512)
            self.ok = True
        except pygame.error:
            return
        folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sounds")
        os.makedirs(folder, exist_ok=True)
        missing = [k for k in PHRASES if not os.path.exists(os.path.join(folder, k + ".wav"))]
        if missing:
            try:
                import pyttsx3
                eng = pyttsx3.init(); eng.setProperty("rate", 175)
                for k in missing:
                    eng.save_to_file(PHRASES[k], os.path.join(folder, k + ".wav"))
                eng.runAndWait()
            except Exception:
                pass
        for key in PHRASES:
            path = os.path.join(folder, key + ".wav")
            try:
                self.sounds[key] = pygame.mixer.Sound(path) if os.path.exists(path) else self._tone(key)
            except Exception:
                self.sounds[key] = self._tone(key)
        self.beep = self._make(self._sine(1000, 0.12))
        sr = pygame.mixer.get_init()[0]
        tt = np.linspace(0, 2.2, int(sr * 2.2), False)
        rng = np.random.default_rng(7)
        boom = (rng.normal(0, 1, len(tt)) * np.exp(-tt * 2.5) + 0.8 * np.sin(2 * np.pi * 55 * tt) * np.exp(-tt * 1.6))
        self.boom = self._make(boom / np.max(np.abs(boom)) * 0.9 * 32767)
        self.chan = pygame.mixer.Channel(0)

    def _sine(self, f, dur):
        t = np.linspace(0, dur, int(pygame.mixer.get_init()[0] * dur), False)
        return 0.4 * np.sin(2 * np.pi * f * t) * 32767

    @staticmethod
    def _make(wave):
        ch = pygame.mixer.get_init()[2]
        wave = wave.astype(np.int16)
        if ch > 1:
            wave = np.repeat(wave[:, None], ch, axis=1).copy()
        return pygame.sndarray.make_sound(wave)

    def _tone(self, key):
        f = {"TRAFFIC": 700, "CLEAR": 500, "ENGAGED": 900, "PILOT": 600}.get(key, 1200)
        w = self._sine(f, 0.6)
        return self._make(w * (np.sin(2 * np.pi * 4 * np.linspace(0, 0.6, len(w))) > 0))

    def say(self, key):
        if self.ok and key in self.sounds: self.queue.append(self.sounds[key])

    def tick_beep(self):
        if self.ok and not self.chan.get_busy(): self.beep.play()

    def play_boom(self):
        if self.ok:
            self.queue.clear(); self.chan.stop()
            pygame.mixer.Channel(1).play(self.boom)

    def busy(self):
        return self.ok and self.chan.get_busy()

    def update(self, mute=False):
        if not self.ok: return
        if mute: self.queue.clear(); return
        if self.queue and not self.chan.get_busy(): self.chan.play(self.queue.pop(0))


# ------------------------------------------------------------------ helpers
def fspl_db(d_m):
    return 32.44 + 20 * math.log10(F_MHZ) + 20 * math.log10(max(d_m, 1.0) / 1000.0)


def dashed_circle(surf, color, c, r, seg=40, width=1):
    if r < 2: return
    for i in range(0, seg, 2):
        a0, a1 = 2 * math.pi * i / seg, 2 * math.pi * (i + 1) / seg
        pygame.draw.line(surf, color, (c[0] + r * math.cos(a0), c[1] + r * math.sin(a0)),
                         (c[0] + r * math.cos(a1), c[1] + r * math.sin(a1)), width)


def plane_icon(surf, color, pos, hdg, size=10):
    pts = [(0, -size), (size * 0.7, size * 0.8), (0, size * 0.4), (-size * 0.7, size * 0.8)]
    c, s = math.cos(hdg), math.sin(hdg)
    pygame.draw.polygon(surf, color, [(pos[0] + x * c - y * s, pos[1] + x * s + y * c) for x, y in pts])


def num(v, nd=1):
    if v is None: return "-"
    if isinstance(v, float) and math.isinf(v): return "∞"
    return f"{v:,.{nd}f}".replace("-", "−")


class Dash:
    def __init__(self, canvas):
        self.c = canvas
        pygame.font.init()
        sans, mono = "segoeui,arial,dejavusans", "consolas,dejavusansmono,couriernew"
        F = lambda n, b=False: pygame.font.SysFont(sans, n, bold=b)
        self.f12, self.f13, self.f14, self.f14b = F(13), F(14), F(15), F(15, True)
        self.f16b, self.f20b, self.f22b, self.f34b = F(17, True), F(21, True), F(24, True), F(36, True)
        self.fsub = F(11)
        self.mono = pygame.font.SysFont(mono, 14)
        self.f48b = F(48, True)
        self.appear = {}
        self.boom_id, self.sparks = None, []

    # ---------------------------------------------------------- text
    def text(self, txt, pos, font=None, color=WHITE, center=False, right=False, maxw=None):
        font, txt = font or self.f14, str(txt)
        if maxw and font.size(txt)[0] > maxw:
            while len(txt) > 3 and font.size(txt + "..")[0] > maxw:
                txt = txt[:-1]
            txt += ".."
        img = font.render(txt, True, color)
        r = img.get_rect()
        if center: r.center = pos
        elif right: r.topright = pos
        else: r.topleft = pos
        self.c.blit(img, r)
        return r

    def math(self, s, pos, font, color=WHITE, hi=AMBER):
        """Small math typesetter: _{sub}  ^{sup}  \\dot{x}  [[highlighted]]."""
        x, y = pos
        base_h = font.get_height()
        i, buf, col = 0, "", color

        def flush():
            nonlocal x, buf
            if buf:
                img = font.render(buf, True, col); self.c.blit(img, (x, y)); x += img.get_width(); buf = ""
        while i < len(s):
            if s.startswith("_{", i) or s.startswith("^{", i):
                flush()
                j = s.index("}", i)
                img = self.fsub.render(s[i + 2:j], True, col)
                dy = base_h - img.get_height() + 2 if s[i] == "_" else -2
                self.c.blit(img, (x, y + dy)); x += img.get_width()
                i = j + 1
            elif s.startswith("\\dot{", i):
                flush()
                j = s.index("}", i)
                img = font.render(s[i + 5:j], True, col)
                self.c.blit(img, (x, y))
                pygame.draw.circle(self.c, col, (x + img.get_width() // 2 + 1, y + 3), 2)
                x += img.get_width(); i = j + 1
            elif s.startswith("[[", i):
                flush(); col = hi; i += 2
            elif s.startswith("]]", i):
                flush(); col = color; i += 2
            else:
                buf += s[i]; i += 1
        flush()
        return x

    def explosion(self, center, age, scale=1.0):
        """Fireball, sparks and smoke, driven by real seconds since the collision."""
        if age < 0 or age > 6:
            return
        cx, cy = center
        surf = pygame.Surface(self.c.get_size(), pygame.SRCALPHA)
        for (ang, spd, life, sz, gray) in self.sparks:          # smoke first (behind)
            if gray and age < life:
                r = (8 + spd * age * 0.35) * scale
                a = int(150 * (1 - age / life))
                pygame.draw.circle(surf, (90, 90, 95, a), (cx + math.cos(ang) * spd * age * 0.25 * scale,
                                                          cy + math.sin(ang) * spd * age * 0.25 * scale - age * 6), r)
        if age < 1.2:                                              # fireball
            r = (12 + 70 * min(age / 0.5, 1.0)) * scale
            a = int(230 * max(0.0, 1 - age / 1.2))
            pygame.draw.circle(surf, (255, 120, 20, a), (cx, cy), r)
            pygame.draw.circle(surf, (255, 220, 120, a), (cx, cy), r * 0.55)
            pygame.draw.circle(surf, (255, 255, 230, a), (cx, cy), r * 0.25)
        for (ang, spd, life, sz, gray) in self.sparks:             # sparks / debris
            if not gray and age < life:
                d = spd * age * scale
                a = int(255 * (1 - age / life))
                pygame.draw.circle(surf, (255, 180 - int(120 * age / life), 40, a),
                                   (cx + math.cos(ang) * d, cy + math.sin(ang) * d + 9 * age * age), sz * scale)
        self.c.blit(surf, (0, 0))

    def new_explosion(self, key):
        if self.boom_id == key:
            return False
        rng = np.random.default_rng(int(abs(hash(key))) % 2**32)
        self.boom_id = key
        self.sparks = [(rng.uniform(0, 2 * math.pi), rng.uniform(30, 120), rng.uniform(0.8, 2.2),
                        rng.uniform(2, 4), False) for _ in range(70)]
        self.sparks += [(rng.uniform(0, 2 * math.pi), rng.uniform(20, 60), rng.uniform(3, 5.5),
                         0, True) for _ in range(18)]
        return True

    def menu(self, sel, key, params, PARAMS, TITLES):
        self.c.fill(BG)
        self.text("PS2  -  LoRa P2P COLLISION AVOIDANCE FOR GENERAL AVIATION", (CW // 2, 70), self.f34b, WHITE, center=True)
        self.text("Cessna 172 Skyhawks  -  Heltec Wireless Tracker (ESP32-S3 + SX1262 LoRa + UC6580 GNSS)",
                  (CW // 2, 112), self.f16b, GREY, center=True)
        rows = [("Scenario", f"{key}:  {TITLES[key].format(n=params['aircraft'])}")] + \
               [(lab, f"{params[k]}") for k, (_, _, _, _, lab) in PARAMS.items()]
        x0, y0, w = 330, 170, 940
        for i, (lab, val) in enumerate(rows):
            y = y0 + i * 58
            on = i == sel
            self.rrect((x0, y, w, 46), CARD if on else PANEL, AMBER if on else DIM, 2, 8)
            self.text(lab, (x0 + 20, y + 13), self.f16b, AMBER if on else WHITE)
            self.text(("◄  " if on else "") + val + ("  ►" if on else ""), (x0 + w - 20, y + 12), self.f16b,
                      WHITE, right=True, maxw=w - 360)
        y = y0 + len(rows) * 58 + 20
        notes = ["↑ / ↓ choose a setting     ← / → change it     ENTER start     ESC quit",
                 "Aircraft count applies to scenario 2.  Lateral offset 0 m = the aircraft would hit without the system.",
                 "FlightGear: start launch_flightgear.bat first; Skyhawk A is the FlightGear cockpit, the others fly as multiplayer traffic.",
                 "In the simulation: SPACE = USER button (pilot takes control), then fly Skyhawk A with the arrow keys."]
        for i, n in enumerate(notes):
            self.text(n, (CW // 2, y + i * 26), self.f14 if i else self.f16b, GREY if i else CYAN, center=True)

    def rrect(self, rect, fill, border=None, w=2, rad=7):
        pygame.draw.rect(self.c, fill, rect, border_radius=rad)
        if border: pygame.draw.rect(self.c, border, rect, w, border_radius=rad)

    def header(self, title, x, y, w, color=AMBER, sub=None):
        r = self.text(title, (x, y), self.f16b, color, maxw=w)
        if sub: self.text(sub, (x + w, y + 3), self.f12, GREY, right=True, maxw=max(10, w - r.width - 12))
        pygame.draw.line(self.c, DIM, (x, y + 25), (x + w, y + 25))

    # ============================================================== LEFT: hardware
    def hardware(self, demo, audio, x0, y0, w, h):
        self.rrect((x0, y0, w, h), PANEL, rad=0)
        x, W = x0 + 12, w - 24
        self.header("HARDWARE  (always on)", x, y0 + 10, W, WHITE)
        self.text("Heltec Wireless Tracker in each Skyhawk + its autopilot", (x, y0 + 40), self.f12, GREY, maxw=W)
        nodes = demo.nodes
        N = len(nodes)
        self.compact = N > 3
        cw = (W - 6 * (N - 1)) // N
        for k, nd in enumerate(nodes):
            lab = (f"Skyhawk {nd.name}" if not self.compact else nd.name) + ("" if nd.can_receive else "  TX only")
            self.text(lab, (x + k * (cw + 6) + 2, y0 + 60), self.f14b, AC_COL[nd.name], maxw=cw)
        rows = [("UC6580 GNSS", "3D fix"), ("ESP32-S3 MCU", "P2P + collision code"),
                ("SX1262 LoRa 915 MHz", "SF7 BW500 21 dBm"), ("TFT display 0.96 in", "traffic + advisory"),
                ("Audio alert out", "intercom / buzzer"), ("USER button", "pilot override"),
                ("Autopilot servos", "HDG / ALT hold")]
        y = y0 + 84
        for name, role in rows:
            r = self.text(name, (x, y), self.f14b, WHITE, maxw=W * 0.6)
            self.text(role, (x + W, y + 1), self.f12, GREY, right=True, maxw=W - r.width - 10)
            for k, nd in enumerate(nodes):
                st, txt = self._hw_state(demo, audio, nd, name)
                self._cell(x + k * (cw + 6), y + 20, cw, 26, st, txt)
            y += 53
        y += 2
        n = demo.nA
        src = n.primary or (next(iter(n.rx)) if n.rx else None)
        self.header(f"LAST LoRa PACKET  A ← {src or '-'}  (33 bytes)", x, y, W, AMBER)
        y += 32
        r = n.rx.get(src) if src else None
        if r:
            hx = [f"{b:02X}" for b in r["bytes"]]
            for i in range(3):
                self.text(" ".join(hx[i * 11:(i + 1) * 11]), (x, y + i * 17), self.mono, CYAN, maxw=W)
            d = r["decoded"]
            hdg = math.degrees(math.atan2(d["ve"], d["vn"])) % 360
            self.text(f"seq {d['seq']}  {d['alt_ft']:,} ft  {math.hypot(d['vn'], d['ve']) / 0.514444:.0f} kt  "
                      f"hdg {hdg:.0f}°  turn {d['turn_dps']:+.1f}°/s", (x, y + 54), self.f12, WHITE, maxw=W)
            rxcap = "yes" if d["flags"] & 16 else "NO (transmit-only)"
            self.text(f"intent {d['intent']}   receiver: {rxcap}   CRC ok", (x, y + 71), self.f12,
                      WHITE if d["flags"] & 16 else AMBER, maxw=W)
        else:
            self.text("no packet received yet", (x, y), self.f14, GREY)
        self.timeline(demo, x, y + 100, W, 184)

    def _cell(self, x, y, w, h, st, txt):
        col = {"off": DIM, "on": GREEN, "active": VIOLET, "warn": AMBER, "alert": RED, "fail": RED, "info": CYAN}[st]
        fill = {"active": (46, 32, 74), "alert": (72, 22, 18), "warn": (62, 46, 10),
                "fail": (54, 12, 12), "off": (17, 22, 28)}.get(st, CARD)
        self.rrect((x, y, w, h), fill, col, 2, 5)
        pygame.draw.circle(self.c, col, (x + 8, y + h // 2), 3, 0 if st != "off" else 1)
        self.text(txt, (x + 15, y + 5), self.f12, WHITE if st != "off" else GREY, maxw=w - 18)

    def _hw_state(self, demo, audio, n, name):
        st, txt = self._hw_state_full(demo, audio, n, name)
        if not getattr(self, "compact", False):
            return st, txt
        short = {"1 Hz cycle": "1 Hz", "computing": "calc", "guiding": "guide", "no signal": "no sig",
                 "no receiver": "no RX", "listening": "listen", "no traffic data": "no data",
                 "traffic picture": "ok", "speaking": "talk", "not pressed": "-", "PRESSED": "ON",
                 "HDG+ALT": "hold", "HDG+ALT hold": "hold", "pilot flying": "pilot", "TX only": "TX only"}
        if txt in short: return st, short[txt]
        if name.startswith("UC6580"): return st, f"{n.plane.fdm['position/h-sl-ft'] / 1000:.1f}k ft"
        if txt.startswith("RX "): return st, txt.replace(" dBm", "").replace("RX ", "RX")
        if txt.startswith("listen"): return st, "listen"
        if name.startswith("Autopilot") and st == "alert": return st, txt[0] + txt[txt.index(" "):]
        return st, txt.replace("TX 33 B slot", "TX").replace("ARMED ", "ARM ").replace("RA ", "")[:9]

    def _hw_state_full(self, demo, audio, n, name):
        t, f = demo.t, n.plane.fdm
        if n.plane.crashed:
            return "fail", "destroyed"
        if name.startswith("UC6580"):
            return "on", f"{f['position/h-sl-ft']:,.0f} ft {f['velocities/vg-fps'] * 0.5925:.0f} kt"
        if name.startswith("ESP32"):
            if not n.can_receive: return "on", "TX only"
            if t - n.sel_t < 1.5 or t - n.coord_t < 1.5: return "active", "computing"
            if n.state in ("RA", "ENGAGED"): return "active", "guiding"
            return "on", "1 Hz cycle"
        if name.startswith("SX1262"):
            if not demo.link.up: return "fail", "no signal"
            if t - n.tx_last_t < 0.12: return "active", f"TX 33 B slot {n.slot}"
            if not n.can_receive: return "off", "no receiver"
            if t - n.last_rx_t() < 0.12 and n.primary:
                return "active", f"RX {LORA_TX - fspl_db(self._dist(demo, n.name, n.primary)):.0f} dBm"
            if n.primary:
                return "on", f"listen +{LORA_TX - fspl_db(self._dist(demo, n.name, n.primary)) - LORA_SENS:.0f} dB"
            return "on", "listening"
        if name.startswith("TFT"):
            if not n.can_receive: return "off", "no traffic data"
            if n.state in ("RA", "ENGAGED") and n.plan: return "alert", "RA " + SHORT[n.plan[n.name]]
            if n.state == "MONITOR" and n.level == 2: return "warn", "TRAFFIC"
            if n.state == "CLEAR": return "on", "clear"
            return "on", "traffic picture"
        if name.startswith("Audio"):
            speaking = audio.busy() if n is demo.nA else (t - n.say_t < 1.5)
            if speaking: return ("alert" if n.state in ("RA", "ENGAGED") else "warn"), "speaking"
            return "off", "quiet"
        if name.startswith("USER"):
            return ("active", "PRESSED") if n.pilot_cancel else ("off", "not pressed")
        if name.startswith("Autopilot"):
            if n.state == "RA": return "warn", f"ARMED {max(0, n.ra_t + CFG['COUNTDOWN_S'] - t):.1f} s"
            if n.state == "ENGAGED":
                return "alert", f"{SHORT[n.plane.maneuver]} {math.degrees(f['attitude/phi-rad']):+.0f}°"
            if n.state == "PILOT" or n.pilot_cancel: return "info", "pilot flying"
            if n.plane.crashed: return "fail", "crashed"
            return "on", "HDG+ALT hold" if len(demo.nodes) < 3 else "HDG+ALT"
        return "off", ""

    @staticmethod
    def _dist(demo, a, b):
        pa = next(p for p in demo.planes if p.name == a).state()
        pb = next(p for p in demo.planes if p.name == b).state()
        return math.hypot(pa.x - pb.x, pa.y - pb.y)

    def timeline(self, demo, x, y, w, h):
        self.header("TIMELINE", x, y, w, AMBER)
        rows = ["LoRa link", "Negotiation"] + [f"Skyhawk {n.name}" for n in demo.nodes]
        top, rh, lx = y + 32, 17, x + 92
        tw = w - 92
        tmax = max(demo.t, 60.0)
        X = lambda t: lx + t / tmax * tw
        for i, r in enumerate(rows):
            self.text(r, (x, top + i * rh - 1), self.f12, GREY)
        hist = demo.history
        for j in range(1, len(hist)):
            t0, t1, hc = hist[j - 1][0], hist[j][0], hist[j]
            x0_, wd = int(X(t0)), int(X(t1)) - int(X(t0)) + 1

            def bar(row, col, frac=1.0, off=0.0):
                pygame.draw.rect(self.c, col, (x0_, top + row * rh + 2 + (rh - 5) * off, wd, max(2, (rh - 5) * frac)))
            bar(0, {"ok": GREEN, "oneway": AMBER, "coordlost": AMBER, "lost": RED}[hc[1]])
            cl = hc[2]["A"][5]
            if cl: bar(1, {1: GREEN, 2: AMBER, 3: RED, 4: AMBER}[cl])
            for k, nd in enumerate(demo.nodes):
                lvl, stt = hc[2][nd.name][0], hc[2][nd.name][1]
                col = {3: RED, 2: AMBER, 1: CYAN}.get(lvl)
                if col: bar(2 + k, col, 0.6)
                if stt in ("ENGAGED", "PILOT"): bar(2 + k, VIOLET if stt == "ENGAGED" else WHITE, 0.4, 0.6)
        n = len(rows)
        pygame.draw.line(self.c, WHITE, (X(demo.t), top - 2), (X(demo.t), top + n * rh))
        yy = top + n * rh + 6
        lx2 = x
        for lab, col in (("near", CYAN), ("TA", AMBER), ("RA", RED), ("autopilot flying", VIOLET)):
            pygame.draw.rect(self.c, col, (lx2, yy + 4, 10, 8))
            lx2 = self.text(lab, (lx2 + 14, yy), self.f12, GREY).right + 10
        self.text(f"{tmax:.0f} s", (x + w, yy), self.f12, GREY, right=True)
        lx2 = x
        lx2 = self.text("link:", (lx2, yy + 17), self.f12, GREY).right + 8
        for lab, col in (("two-way", GREEN), ("one-way", AMBER), ("lost", RED)):
            pygame.draw.rect(self.c, col, (lx2, yy + 21, 10, 8))
            lx2 = self.text(lab, (lx2 + 14, yy + 17), self.f12, GREY).right + 10

    # ============================================================== CENTER: radar
    def radar(self, demo, x0, y0, w, h, speed, slow):
        self.rrect((x0, y0, w, h), BG, rad=0)
        n = demo.nA
        self.text("COCKPIT TRAFFIC DISPLAY  -  Skyhawk A", (x0 + 14, y0 + 10), self.f16b, GREY)
        self.text(f"t {demo.t:6.1f} s   SPEED {speed:g}x{'  SLOW-MO' if slow else ''}", (x0 + w - 14, y0 + 10),
                  self.f16b, CYAN, right=True)
        cx, cy, k = x0 + w // 2, y0 + 318, 36.0
        for r_nm, lab, col in ((2, "2 nm", DIM), (3, "3 nm ALERT", AMBER), (5, "5 nm DETECT", DIM)):
            dashed_circle(self.c, col, (cx, cy), r_nm * k, seg=72)
            self.text(lab, (cx + r_nm * k * 0.71 + 4, cy + r_nm * k * 0.71 - 4), self.f12, col)
        own = demo.A.state()
        for d in range(0, 360, 30):
            a = math.radians(d) - own.hdg
            r0, r1 = 5 * k, 5 * k + (10 if d % 90 == 0 else 5)
            pygame.draw.line(self.c, GREY, (cx + r0 * math.sin(a), cy - r0 * math.cos(a)),
                             (cx + r1 * math.sin(a), cy - r1 * math.cos(a)), 2)
            if d % 90 == 0:
                self.text("NESW"[d // 90], (cx + (r1 + 12) * math.sin(a), cy - (r1 + 12) * math.cos(a)),
                          self.f14b, GREY, center=True)
        plane_icon(self.c, WHITE, (cx, cy), 0, 14)
        if n.state == "RA":
            frac = max(0.0, (n.ra_t + CFG["COUNTDOWN_S"] - demo.t) / CFG["COUNTDOWN_S"])
            pygame.draw.arc(self.c, AMBER, (cx - 34, cy - 34, 68, 68), math.pi / 2, math.pi / 2 + 2 * math.pi * frac, 5)
        co, si = math.cos(-own.hdg), math.sin(-own.hdg)
        rot = lambda dx, dy: (dx * co + dy * si, -dx * si + dy * co)
        for nm in n.rx:
            est = n.estimate(nm, demo.t)
            s = n.assess.get(nm)
            if est is None or s is None or s["level"] < 0: continue
            lvl = 3 if (n.state in ("RA", "ENGAGED") and nm in n.cluster) else s["level"]
            rx, ry = rot(est.x - own.x, est.y - own.y)
            p = (cx + rx / NM * k, cy - ry / NM * k)
            vx, vy = rot(est.vx - own.vx, est.vy - own.vy)
            q = (p[0] + vx * 30 / NM * k, p[1] - vy * 30 / NM * k)
            col = {3: RED, 2: AMBER}.get(lvl, CYAN)
            if math.hypot(p[0] - cx, p[1] - cy) < 5.6 * k:
                pygame.draw.line(self.c, col, p, q, 1)
                self._intruder(p, lvl, (est.h - own.h) / FT, est.vs / FT * 60, nm)
        if n.state in ("RA", "ENGAGED", "PILOT") and n.plan and not demo.collision:
            man = n.plan[n.name]
            self.text(CALLOUT[man], (cx, y0 + 50), self.f34b, RED, center=True)
            if man in ("TURN_RIGHT", "TURN_LEFT"):
                d = 1 if man == "TURN_RIGHT" else -1
                pygame.draw.line(self.c, GREEN, (cx - 46 * d, y0 + 88), (cx + 36 * d, y0 + 88), 8)
                pygame.draw.polygon(self.c, GREEN, [(cx + 60 * d, y0 + 88), (cx + 32 * d, y0 + 73), (cx + 32 * d, y0 + 103)])
            elif man != "MAINTAIN":
                self.text("▲ CLIMB ▲" if man == "CLIMB" else "▼ DESCEND ▼", (cx, y0 + 88), self.f20b, GREEN, center=True)
        elif n.state == "MONITOR" and n.level == 2:
            self.text("TRAFFIC, TRAFFIC", (cx, y0 + 50), self.f34b, AMBER, center=True)
        elif n.state == "CLEAR":
            self.text("CLEAR OF CONFLICT", (cx, y0 + 50), self.f34b, GREEN, center=True)
        msg, col = None, WHITE
        if n.state == "RA":
            msg, col = f"AUTOPILOT IN {max(0, n.ra_t + CFG['COUNTDOWN_S'] - demo.t):.1f} s   -   SPACE = USER button: I have control", AMBER
        elif n.state == "ENGAGED":
            msg, col = (f"AUTOPILOT FLYING {n.plan[n.name].replace('_', ' ')}   bank "
                        f"{math.degrees(demo.A.fdm['attitude/phi-rad']):+.0f}°   VS {demo.A.fdm['velocities/h-dot-fps'] * 60:+.0f} fpm"), GREEN
        if n.pilot_cancel and not demo.A.crashed:
            r_, p_ = demo.A.input_roll, demo.A.input_pitch
            inp = " ".join(x for x in (("◄ LEFT" if r_ < 0 else "RIGHT ►" if r_ > 0 else ""),
                                       ("▲ CLIMB" if p_ > 0 else "▼ DESCEND" if p_ < 0 else "")) if x) or "wings level"
            msg, col = f"PILOT FLYING   ← → bank   ↑ ↓ climb / descend      stick: {inp}", CYAN
        if msg:
            self.text(msg, (cx, y0 + h - 14), self.f16b, col, center=True, maxw=w - 30)
        if demo.collision:
            c = demo.collision
            age = time.monotonic() - c["wall"]
            if "A" in c["names"]:
                self.explosion((cx, cy), age, 1.0)
            if int(age * 3) % 2 == 0 or age > 3:
                self.text("MIDAIR COLLISION", (cx, y0 + 52), self.f48b, RED, center=True)
            self.text(f"Skyhawk {c['names'][0]} and Skyhawk {c['names'][1]} collided at t = {c['t']:.1f} s",
                      (cx, y0 + 92), self.f16b, WHITE, center=True)
        elif demo.nmac_t is not None and demo.t - demo.nmac_t < 4:
            self.text("NEAR MIDAIR COLLISION", (cx, y0 + 130), self.f20b, AMBER, center=True)

    def _intruder(self, p, level, rel_ft, vs_fpm, name):
        r = 9
        dia = [(p[0], p[1] - r), (p[0] + r, p[1]), (p[0], p[1] + r), (p[0] - r, p[1])]
        if level <= 0: pygame.draw.polygon(self.c, CYAN, dia, 2); col = CYAN
        elif level == 1: pygame.draw.polygon(self.c, CYAN, dia); col = CYAN
        elif level == 2: pygame.draw.circle(self.c, AMBER, p, r); col = AMBER
        else: pygame.draw.rect(self.c, RED, (p[0] - r, p[1] - r, 2 * r, 2 * r)); col = RED
        tag = "00" if abs(rel_ft) < 50 else f"{'+' if rel_ft > 0 else '−'}{abs(rel_ft) / 100:02.0f}"
        self.text(tag, (p[0] - 13, p[1] - 30 if rel_ft >= 0 else p[1] + 11), self.f16b, col)
        if abs(vs_fpm) >= 500:
            self.text("↑" if vs_fpm > 0 else "↓", (p[0] + 13, p[1] - 11), self.f16b, col)
        self.text(f"Skyhawk {name}", (p[0] + 13, p[1] + 4), self.f12, col)

    # ============================================================== CENTER: calculations
    def calculations(self, demo, x0, y0, w, h):
        self.rrect((x0, y0, w, h), PANEL, rad=0)
        x, W = x0 + 12, w - 24
        self.header("LIVE CALCULATIONS", x, y0 + 8, W, WHITE, "only what decides the outcome right now")
        cards = self._pick(demo)[:4]
        cw, chh = (W - 12) // 2, (h - 50 - 12) // 2
        for i, cd in enumerate(cards):
            self._card(x + (i % 2) * (cw + 12), y0 + 42 + (i // 2) * (chh + 12), cw, chh, cd)

    def _card(self, x, y, w, h, cd):
        acc = cd["accent"]
        self.rrect((x, y, w, h), CARD, None, rad=8)
        self.c.set_clip((x, y, w, h))
        pygame.draw.rect(self.c, acc, (x, y, 5, h))
        vw = 0
        if cd.get("verdict"):
            txt, col = cd["verdict"]
            vw = self.text(txt, (x + w - 12, y + 8), self.f14b, col, right=True, maxw=w * 0.42).width + 10
        self.text(cd["title"], (x + 16, y + 9), self.f12, acc, maxw=w - 28 - vw)
        self.math(cd["big"], (x + 16, y + 28), self.f22b)
        yy, bottom = y + 62, y + h - (32 if cd.get("bar") else 6)
        for line, col in cd["lines"]:
            if yy + 17 > bottom: break
            self.math(line, (x + 16, yy), self.f13, col)
            yy += 19
        if cd.get("bar"):
            v, vmax, marks = cd["bar"]
            bx, by, bw = x + 16, y + h - 13, w - 32
            pygame.draw.rect(self.c, DIM, (bx, by, bw, 6), border_radius=3)
            pygame.draw.rect(self.c, acc, (bx, by, int(bw * max(0, min(1, v / vmax))), 6), border_radius=3)
            for m, lab, col in marks:
                mx = bx + bw * min(1, m / vmax)
                pygame.draw.line(self.c, col, (mx, by - 4), (mx, by + 10), 2)
                self.text(lab, (mx + 4, by - 16), self.f12, col)
        self.c.set_clip(None)

    def _pick(self, demo):
        n, t = demo.nA, demo.t
        s = n.assessment
        if demo.collision:
            c = demo.collision
            why = ("pilot took control and did not avoid" if n.pilot_cancel else
                   "collision avoidance was not active" if not demo.acas else "")
            return [self._c_result(demo),
                    dict(title="WHAT HAPPENED", accent=RED, big=f"{c['names'][0]} and {c['names'][1]} collided",
                         lines=[(f"{demo.min_h:.0f} m apart horizontally, {demo.v_at:.1f} m vertically", WHITE),
                                ("contact: < 11 m (wingspan) and < 3 m vertical", GREY),
                                (why, AMBER)], verdict=("COLLISION", RED))]
        if s is None or (n.level == -1 and n.state == "MONITOR"):
            return [self._c_link(demo), self._c_detect(demo)]
        cards = []
        oneway = [nm for nm in n.rx if not n.peer_can_negotiate(nm) and n.assess.get(nm, {}).get("level", -1) >= 1]
        if oneway and n.state != "CLEAR":
            cards.append(self._c_oneway(n, oneway[0]))
        if n.gcas_t > 0 and t - n.gcas_t < 6:
            cards.append(self._c_gcas(n))
        if s["sigma_pos"] > 20:
            cards.append(self._c_sigma(s))
        others = [nm for nm in n.assess if nm != n.primary and n.assess[nm]["level"] >= 1] if len(n.assess) > 1 else []
        if n.state == "MONITOR":
            if s["range_m"] < s["dmod"] * 1.6 and s["range_rate"] < 0 and s["tau"] > CFG["RA_TAU"]:
                cards.append(self._c_dmod(s))
            cards += [self._c_tau(s, n.primary), self._c_miss(s, "RA" if n.level == 2 else "TA", n.primary)]
            if others: cards.append(self._c_others(n, others))
            if n.level == 2 and n.sel: cards.append(self._c_resolution(n, pre=True))
            cards.append(self._c_link(demo))
        elif n.state == "RA":
            cards += [self._c_ra_tests(s, n.primary), self._c_resolution(n), self._c_countdown(n, t)]
            if others: cards.append(self._c_others(n, others))
            cards.append(self._c_tau(s, n.primary))
        elif n.state == "ENGAGED":
            cards += [self._c_working(n, s), self._c_clear_test(s), self._c_resolution(n), self._c_tau(s, n.primary)]
        elif n.state == "PILOT":
            cards += [self._c_pilot(), self._c_working(n, s), self._c_clear_test(s)]
        elif n.state == "CLEAR":
            cards += [self._c_clear_test(s), self._c_result(demo), self._c_link(demo)]
        seen, out = set(), []
        for cd in cards:
            if cd["title"] not in seen:
                seen.add(cd["title"]); out.append(cd)
        return out

    # ---- equation cards (math markup: _{sub} ^{sup} \dot{r} [[highlight]])
    def _c_link(self, demo):
        d = self._dist(demo, "A", demo.nA.primary or "B")
        L = fspl_db(d); pr = LORA_TX - L; m = pr - LORA_SENS
        s = demo.nA.assessment
        dop = abs(s["range_rate"]) * F_MHZ * 1e6 / 3e8 if s else 0.0
        ok = demo.link.up and m > 0
        return dict(title="LoRa P2P LINK  (SX1262, 915 MHz)", accent=GREEN if ok else RED,
                    big=f"P_{{r}} = {num(pr)} dBm    margin [[{m:.0f} dB]]",
                    lines=[(f"FSPL = 32.44 + 20 log_{{10}}(915) + 20 log_{{10}}({d / 1000:.2f}) = {L:.1f} dB", WHITE),
                           ("margin = P_{r} − (−117 dBm sensitivity, SF7/BW500)", GREY),
                           (f"Doppler = |\\dot{{r}}|·f / c = {dop:.0f} Hz  (LoRa tolerates ±125 kHz)", GREY)],
                    verdict=("LINK OK" if ok else "LINK LOST", GREEN if ok else RED))

    def _c_detect(self, demo):
        d = self._dist(demo, "A", "B") / NM
        return dict(title="DETECTION", accent=GREY, big=f"range {d:.2f} nm",
                    lines=[("LoRa packets arrive 5 times per second", WHITE),
                           ("tracking starts inside 5 nm, alerting inside 3 nm", GREY)],
                    verdict=("tracking" if d <= CFG["DETECT_NM"] else "outside 5 nm", CYAN if d <= 5 else GREY))

    def _c_tau(self, s, nm):
        tm = s["tau_mod"]
        col = RED if tm <= CFG["RA_TAU"] else AMBER if tm <= CFG["TA_TAU"] else CYAN
        return dict(title=f"TIME TO GO vs Skyhawk {nm}  (TCAS II-style τ)", accent=col,
                    big=f"τ_{{mod}} = [[{num(tm)} s]]",
                    lines=[(f"\\dot{{r}} = (r·v) / |r| = {num(s['range_rate'])} m/s  ({'closing' if s['range_rate'] < 0 else 'opening'})", WHITE),
                           (f"τ = −|r| / \\dot{{r}} = {s['range_m']:,.0f} / {abs(s['range_rate']):.1f} = {num(s['tau'])} s", GREY)],
                    bar=(min(60, 60 - (tm if not math.isinf(tm) else 60)), 60,
                         [(60 - CFG["TA_TAU"], "TA 40 s", AMBER), (60 - CFG["RA_TAU"], "RA 25 s", RED)]))

    def _c_miss(self, s, zone, nm):
        D = s["ra_zone_h"] if zone == "RA" else s["ta_zone_h"]
        base = 457 if zone == "RA" else 926
        inside = s["confl_ra"] if zone == "RA" else s["confl_ta"]
        return dict(title=f"WILL THEY HIT?  A vs {nm}, {zone} zone", accent=RED if inside else GREEN,
                    big=f"miss [[{s['d_min_curved']:,.0f} m]]  vs  zone {D:,.0f} m",
                    lines=[(f"curved path from LoRa turn rate: closest in {s['t_min_curved']:.0f} s", WHITE),
                           (f"Δh at closest = {s['dv_at_min'] / FT:.0f} ft", WHITE),
                           (f"zone = {base} + 2σ = {base} + 2×{s['sigma_pos']:.1f} = {D:,.0f} m", GREY)],
                    verdict=("CONFLICT" if inside else "PASSES CLEAR", RED if inside else GREEN))

    def _c_sigma(self, s):
        return dict(title="UNCERTAINTY GROWTH  (no fresh LoRa packets)", accent=AMBER,
                    big=f"σ = [[{s['sigma_pos']:,.0f} m]]   age {s['packet_age']:.1f} s",
                    lines=[("σ = √(σ_{A}² + σ_{B}² + (σ_{v}·age)² + (½·a·age²)²)", WHITE),
                           (f"RA zone = 457 + 2σ = {s['ra_zone_h']:,.0f} m", WHITE),
                           ("position dead-reckoned from the last packet", GREY)],
                    verdict=("alert earlier", AMBER))

    def _c_dmod(self, s):
        return dict(title="RANGE GUARD  (slow closure, DMOD)", accent=RED,
                    big=f"r = {s['range_m']:,.0f} m  <  DMOD {s['dmod']:.0f} m",
                    lines=[(f"closing only {abs(s['range_rate']):.1f} m/s → τ = {num(s['tau'])} s looks safe", WHITE),
                           ("τ_{mod} = −(r² − DMOD²) / (r·\\dot{r}) → 0", WHITE)],
                    verdict=("→ alert", RED))

    def _c_ra_tests(self, s, nm):
        a = s["tau_mod"] <= CFG["RA_TAU"] or s["range_m"] < s["dmod"]
        yes = lambda b: "[[YES]]" if b else "no"
        return dict(title=f"RA DECISION  (A vs Skyhawk {nm})", accent=RED, big="RA ISSUED",
                    lines=[(f"time:   τ_{{mod}} = {num(s['tau_mod'])} s ≤ 25 s  →  {yes(a)}", WHITE),
                           (f"space:  miss {s['d_min_curved']:,.0f} m < zone {s['ra_zone_h']:,.0f} m  →  {yes(s['confl_ra'])}", WHITE),
                           (f"height: Δh {s['dv_at_min'] / FT:.0f} ft < {s['ra_zone_v'] / FT:.0f} ft  →  "
                            f"{yes(s['dv_at_min'] < s['ra_zone_v'])}", WHITE)],
                    verdict=("→ RA", RED))

    def _c_resolution(self, n, pre=False):
        sel = n.sel
        plan = sel["plan"]
        best = sel["best"]
        runner = sel["results"][1] if len(sel["results"]) > 1 else None
        vals = set(plan.values())
        big = (f"{', '.join(plan)}: all {SHORT[vals.pop()]}" if len(vals) == 1 and len(plan) > 2
               else "  ".join(f"{k}:{SHORT[v]}" for k, v in plan.items()) if len(plan) > 3
               else "   ".join(f"{k} {SHORT[v]}" for k, v in plan.items()))
        lines = [(f"scored {sel['pairs']} legal plans × {sel['n']} uncertainty samples", GREY),
                 (f"chosen: P(collision) = [[{best['p_nmac']:.2f}]],  P(zone) = {best['p_zone']:.2f}", WHITE)]
        if runner:
            nb = " ".join(f"{k} {SHORT[v]}" for k, v in runner["plan"].items())
            lines.append((f"next best {nb}" + (f":  P(zone) {runner['p_zone']:.2f}" if len(runner["plan"]) < 3 else ""), GREY))
        lvl = COORD_NAME[n.coord_level]
        return dict(title=("PRE-NEGOTIATED PLAN" if pre else "CHOSEN MANEUVERS") + "  (ACAS X-style search)",
                    accent=VIOLET, big=big, lines=lines,
                    verdict=(lvl, GREEN if n.coord_level == 1 else AMBER))

    def _c_countdown(self, n, t):
        left = max(0.0, n.ra_t + CFG["COUNTDOWN_S"] - t)
        return dict(title="PILOT RESPONSE WINDOW", accent=AMBER, big=f"[[{left:.1f} s]]",
                    lines=[("pilot may press the USER button (SPACE)", WHITE),
                           (f"else the autopilot flies {n.plan[n.name].replace('_', ' ')}", WHITE)],
                    bar=(CFG["COUNTDOWN_S"] - left, CFG["COUNTDOWN_S"], []), verdict=("counting", AMBER))

    def _c_working(self, n, s):
        ok = not s["confl_ra"]
        was = n.ra_dmin
        return dict(title="IS IT WORKING?  (predicted closest distance)", accent=GREEN if ok else AMBER,
                    big=(f"miss {was:,.0f} m  →  [[{s['d_min_curved']:,.0f} m]]" if was is not None
                         else f"miss [[{s['d_min_curved']:,.0f} m]]"),
                    lines=[(f"now {s['range_m']:,.0f} m apart; at closest they will be {s['d_min_curved']:,.0f} m", WHITE),
                           (f"was {was:,.0f} m when the RA fired;  zone {s['ra_zone_h']:,.0f} m" if was is not None
                            else f"zone {s['ra_zone_h']:,.0f} m", GREY),
                           (f"bank {math.degrees(n.plane.fdm['attitude/phi-rad']):+.0f}°   VS {n.plane.fdm['velocities/h-dot-fps'] * 60:+.0f} fpm", GREY)],
                    verdict=("WORKING" if ok else "converging", GREEN if ok else AMBER))

    def _c_clear_test(self, s):
        a, b = s["range_rate"] > 0, s["range_m"] > s["ra_zone_h"]
        yes = lambda c: "[[YES]]" if c else "no"
        return dict(title="CLEAR-OF-CONFLICT TEST", accent=GREEN if a and b else GREY,
                    big="CLEAR" if a and b else "not yet",
                    lines=[(f"\\dot{{r}} = {s['range_rate']:+.1f} m/s > 0 (opening)  →  {yes(a)}".replace("-", "−"), WHITE),
                           (f"r = {s['range_m']:,.0f} m > zone {s['ra_zone_h']:,.0f} m  →  {yes(b)}", WHITE)],
                    verdict=("hand back" if a and b else "keep going", GREEN if a and b else GREY))

    def _c_gcas(self, n):
        agl = n.plane.fdm["position/h-agl-ft"]
        rate = min(CFG["VS_CMD_FPM"], n.plane.desc_fpm)
        after = agl - rate * CFG["GCAS_LOOKAHEAD_S"] / 60
        return dict(title="GCAS TERRAIN CHECK", accent=RED, big="descent vetoed",
                    lines=[(f"h_{{AGL}} − VS × 20 s = {agl:,.0f} − {rate:.0f} × 20/60 = [[{after:,.0f} ft]]", WHITE),
                           (f"{after:,.0f} ft < {CFG['GROUND_FLOOR_FT']:,.0f} ft floor", RED)],
                    verdict=("→ turn instead", AMBER))

    def _c_oneway(self, n, nm):
        hz = n.rx_rate(nm)
        return dict(title="ONE-WAY LoRa LINK", accent=AMBER, big=f"Skyhawk {nm} cannot hear A",
                    lines=[(f"A hears {nm} at {hz:.1f} Hz → A tracks and predicts it", WHITE),
                           (f"{nm}'s receiver flag = [[0]] → no handshake", WHITE),
                           (f"A plans alone; {nm} assumed to {SHORT[n.peer_intent(nm)]} (its intent)", GREY)],
                    verdict=("A acts alone", AMBER))

    def _c_others(self, n, others):
        tracked = sorted(n.assess, key=lambda nm: n.assess[nm]["tau_mod"])
        conflict = [nm for nm in tracked if n.assess[nm]["level"] >= 2 or nm in n.cluster]
        lines = []
        for nm in tracked[:3]:
            a = n.assess[nm]
            lines.append((f"Skyhawk {nm}:  τ_{{mod}} = {num(a['tau_mod'])} s,  miss {a['d_min_curved']:,.0f} m  →  "
                          f"[[{LEVELS[a['level']]}]]", WHITE))
        lines.append(("every aircraft in conflict is solved in one search", GREY))
        return dict(title=f"TRAFFIC  (A tracks {len(tracked)} aircraft)", accent=CYAN,
                    big=f"A + {len(conflict)} aircraft in conflict",
                    lines=lines, verdict=("joint plan", CYAN))

    def _c_pilot(self):
        return dict(title="PILOT TOOK CONTROL", accent=CYAN, big="MANUAL sent over LoRa",
                    lines=[("Skyhawk A autopilot stays off", WHITE),
                           ("the other units re-plan with A held to its course", WHITE)],
                    verdict=("pilot in charge", CYAN))

    def _c_result(self, demo):
        bad = demo.collision or demo.nmac
        return dict(title="RESULT", accent=RED if bad else GREEN,
                    big=f"closest [[{demo.min_h:,.0f} m]] / {demo.v_at / FT:.0f} ft",
                    lines=[("NMAC: < 152 m (500 ft) apart and < 30 m (100 ft) vertical", GREY)],
                    verdict=("COLLISION" if demo.collision else "NMAC" if demo.nmac else "NO COLLISION",
                             RED if bad else GREEN))

    # ============================================================== RIGHT
    def observer(self, demo, x0, y0, w, h):
        self.rrect((x0, y0, w, h), BG, rad=0)
        self.text(f"THIRD-PERSON VIEW   {demo.key}", (x0 + 12, y0 + 8), self.f16b, GREY, maxw=w - 24)
        self.text(demo.title, (x0 + 12, y0 + 31), self.f12, GREY, maxw=w - 24)
        mx, my, mw, mh = x0 + 10, y0 + 52, w - 20, h - 60
        self.rrect((mx, my, mw, mh), PANEL, rad=6)
        tracks = [p.track for p in demo.planes]
        pts = [q for tr in tracks for q in tr]
        if not pts: return
        xs, ys = [q[1] for q in pts], [q[2] for q in pts]
        cxm, cym = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        span = max((max(xs) - min(xs)) * mh / mw, max(ys) - min(ys), 2 * NM) * 1.15
        k = mh / span
        tf = lambda x, y: (mx + mw / 2 + (x - cxm) * k, my + mh / 2 - (y - cym) * k)
        self.c.set_clip((mx, my, mw, mh))
        g0 = math.floor((cym - span) / NM)
        for i in range(int(2 * span / NM) + 3):
            gy = tf(cxm, (g0 + i) * NM)[1]; gx = tf((math.floor((cxm - span) / NM) + i) * NM, cym)[0]
            pygame.draw.line(self.c, (24, 32, 42), (mx, gy), (mx + mw, gy))
            pygame.draw.line(self.c, (24, 32, 42), (gx, my), (gx, my + mh))
        for p, tr in zip(demo.planes, tracks):
            if len(tr) > 1:
                pygame.draw.lines(self.c, AC_COL[p.name], False, [tf(q[1], q[2]) for q in tr[::3] + [tr[-1]]], 2)
        n = demo.nA
        for nm in n.rx:
            est, s = n.estimate(nm, demo.t), n.assess.get(nm)
            if est is None or s is None or s["level"] < 1: continue
            col = RED if (n.state in ("RA", "ENGAGED") and nm in n.cluster) else (AMBER if s["level"] == 2 else GREY)
            dashed_circle(self.c, col, tf(est.x, est.y), min(s["ra_zone_h"] * k, 900), seg=50, width=2)
        for p in demo.planes:
            st = p.state(); q = tf(st.x, st.y)
            if p.crashed: continue
            plane_icon(self.c, AC_COL[p.name], q, st.hdg, 11)
            self.text(f"Skyhawk {p.name}  {st.h / FT:,.0f} ft", (q[0] + 13, q[1] - 8), self.f12, AC_COL[p.name])
        if demo.collision:
            c = demo.collision
            self.explosion(tf(c["x"], c["y"]), time.monotonic() - c["wall"], 0.7)
        self.c.set_clip(None)
        self.text("1 nm grid   dashed circle: A's protected zone around each threat", (mx + 6, my + mh - 18),
                  self.f12, GREY, maxw=mw - 12)

    def software(self, demo, x0, y0, w, h):
        self.rrect((x0, y0, w, h), PANEL, rad=0)
        x, W = x0 + 12, w - 24
        self.header("SOFTWARE ON ESP32-S3  (Skyhawk A)", x, y0 + 8, W, WHITE)
        self.text("modules appear only while they are running", (x, y0 + 37), self.f12, GREY)
        n, t = demo.nA, demo.t
        cards = []
        live = {nm: a for nm, a in n.assess.items() if a["level"] >= 0}
        if live:
            cards.append(("tracker", "Tracker", CYAN,
                          [f"{nm}: {a['range_m'] / NM:4.2f} nm  brg {a['bearing_deg']:3.0f}°  σ {a['sigma_pos']:4.0f} m  age {a['packet_age']:.1f} s"
                           for nm, a in live.items()]))
            alerting = {nm: a for nm, a in live.items() if a["level"] >= 1}
            if alerting or n.state != "MONITOR":
                worst = 3 if n.state in ("RA", "ENGAGED") else max(a["level"] for a in live.values())
                cards.append(("threat", "Collision logic  (TCAS II-style τ / DMOD alerting)",
                              {3: RED, 2: AMBER}.get(worst, CYAN),
                              [f"{nm}: {LEVELS[a['level']]:<10} τmod {num(a['tau_mod'], 0):>4} s  miss {a['d_min_curved']:6,.0f} m"
                               for nm, a in alerting.items()] or ["monitoring"]))
        if n.sel and t - n.sel_t < 6:
            sel = n.sel
            rows = []
            for r in sel["results"][:4]:
                if len(r["plan"]) > 3:
                    rows.append(" ".join(f"{k}:{SHORT[v][0]}" for k, v in r["plan"].items())
                                + f"   P(coll) {r['p_nmac']:.2f}" + ("  ◄" if r is sel["best"] else ""))
                else:
                    rows.append("  ".join(f"{k} {SHORT[v]:<7}" for k, v in r["plan"].items())
                                + f" P(coll) {r['p_nmac']:.2f}" + ("  ◄" if r is sel["best"] else ""))
            reasons = sorted({v[1] for v in sel["vetoes"]})
            if sel["fixed"]:
                rows.append("held (cannot negotiate): " + ", ".join(f"{k} {SHORT[v]}" for k, v in sel["fixed"].items()))
            if reasons:
                rows.append(f"rejected {len(sel['vetoes'])}: " + "; ".join(reasons))
            cards.append(("selector", "Maneuver selector  (ACAS X-style cost search)", VIOLET, rows))
        oneway = [nm for nm in n.rx if not n.peer_can_negotiate(nm)]
        if t - n.coord_t < 6 or n.state == "RA" or (oneway and n.level >= 2):
            msgs = [m for m in demo.msgs if t - m[0] < 8 and "A" in (m[1], m[2])][-4:]
            rows = [f"{m[1]}→{m[2]} {m[3]:<7} {m[4][:23]}  {'ok' if m[5] else 'LOST'}" for m in msgs]
            if oneway:
                rows.append(f"{', '.join(oneway)} has no receiver: no handshake possible")
            rows.append("level: " + COORD_NAME[n.coord_level])
            cards.append(("coord", "Negotiation over LoRa  (12-byte messages)",
                          GREEN if n.coord_level == 1 else AMBER, rows))
        if n.gcas_t > 0 and t - n.gcas_t < 6:
            cards.append(("gcas", "GCAS terrain monitor", RED, ["descent would go below 1,000 ft AGL → rejected"]))
        if n.state in ("RA", "ENGAGED", "PILOT"):
            txt = {"RA": f"countdown {max(0, n.ra_t + CFG['COUNTDOWN_S'] - t):.1f} s, then autopilot",
                   "ENGAGED": f"autopilot flying {n.plane.maneuver.replace('_', ' ')}, bank limit 20°",
                   "PILOT": "pilot in control, automation off"}[n.state]
            cards.append(("auto", "Automation + safety filter",
                          AMBER if n.state == "RA" else RED if n.state == "ENGAGED" else CYAN, [txt]))
        now = time.monotonic()
        keys = {c[0] for c in cards}
        for kk in list(self.appear):
            if kk not in keys: del self.appear[kk]
        self.c.set_clip((x0, y0, w, h))
        y = y0 + 58
        if not cards:
            self.text("monitoring  -  no module active", (x, y + 4), self.f14, GREY)
        for key, title, col, rows in cards:
            self.appear.setdefault(key, now)
            slide = max(0.0, 1 - (now - self.appear[key]) / 0.3)
            lines = []
            for r in rows:
                lines += self._wrap(r, self.mono, W - 24)
            hh = 30 + 17 * len(lines)
            if y + hh > y0 + h - 6: break
            xx = x + int(slide * 60)
            self.rrect((xx, y, W, hh), CARD, col, 2, 7)
            self.text(title, (xx + 12, y + 6), self.f14b, col, maxw=W - 24)
            for i, r in enumerate(lines):
                self.text(r, (xx + 12, y + 26 + 17 * i), self.mono, WHITE, maxw=W - 24)
            y += hh + 8
        self.c.set_clip(None)

    @staticmethod
    def _wrap(txt, font, maxw):
        words, lines, cur = txt.split(" "), [], ""
        for wd in words:
            test = cur + (" " if cur else "") + wd
            if font.size(test)[0] <= maxw or not cur: cur = test
            else: lines.append(cur); cur = wd
        if cur: lines.append(cur)
        return lines


# ------------------------------------------------------------------ main loop
def run(Demo, titles, key, fg=True, speed=1.0, snapshots=None, snapshot_dir=".", params=None):
    import run_demo
    make_dpi_aware()
    pygame.init()
    info = pygame.display.Info()
    sw, sh = (info.current_w, info.current_h) if info.current_w > 0 else (CW, CH)
    size = (CW, CH) if (sw >= CW and sh * 0.94 >= CH) else \
        (int(CW * min(sw * 0.97 / CW, sh * 0.9 / CH)), int(CH * min(sw * 0.97 / CW, sh * 0.9 / CH)))
    win = pygame.display.set_mode(size, pygame.RESIZABLE)
    pygame.display.set_caption("PS2 LoRa P2P collision avoidance  -  Cessna 172 Skyhawks")
    canvas = pygame.Surface((CW, CH))
    dash, audio, clock = Dash(canvas), Audio(), pygame.time.Clock()
    keys = list(titles)
    params = dict(params or run_demo.default_params())
    PARAMS = run_demo.PARAMS
    in_menu, msel = snapshots is None, 0
    demo = Demo(key, fg=fg, params=params)
    demo.paused = False
    si = min(range(len(SPEEDS)), key=lambda i: abs(SPEEDS[i] - speed))
    auto_slow, acc, said, last_beep = False, 0.0, 0, -1
    snaps = sorted(snapshots or [])
    running = True
    while running:
        step_once = False
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.VIDEORESIZE:
                win = pygame.display.set_mode(ev.size, pygame.RESIZABLE)
            elif ev.type == pygame.KEYDOWN and in_menu:
                pk_ = list(PARAMS)
                if ev.key == pygame.K_ESCAPE: running = False
                elif ev.key == pygame.K_UP: msel = (msel - 1) % (len(pk_) + 1)
                elif ev.key == pygame.K_DOWN: msel = (msel + 1) % (len(pk_) + 1)
                elif ev.key in (pygame.K_LEFT, pygame.K_RIGHT):
                    d = 1 if ev.key == pygame.K_RIGHT else -1
                    if msel == 0:
                        key = keys[(keys.index(key) + d) % len(keys)]
                    else:
                        k = pk_[msel - 1]; _, lo, hi, stp, _ = PARAMS[k]
                        params[k] = max(lo, min(hi, params[k] + d * stp))
                elif ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    demo, said, acc = Demo(key, fg=fg, params=params), 0, 0.0
                    dash.appear.clear(); in_menu = False
            elif ev.type == pygame.KEYDOWN:
                ch = ev.unicode.lower() if ev.unicode else ""
                if ev.key == pygame.K_ESCAPE: running = False
                elif ev.key == pygame.K_SPACE: demo.pilot_takeover()
                elif ch == "m": in_menu = True
                elif ch == "p": demo.paused = not demo.paused
                elif ch in ("+", "="): si = min(si + 1, len(SPEEDS) - 1)
                elif ch in ("-", "_"): si = max(si - 1, 0)
                elif ch == "0": si = SPEEDS.index(1.0)
                elif ch == "s": auto_slow = not auto_slow
                elif ch == "n": step_once = True
                elif ch == "l": demo.toggle_link()
                elif ch == "k": demo.toggle_coord()
                elif ch == "r" or (ch.isdigit() and 1 <= int(ch) <= len(keys)):
                    key = keys[int(ch) - 1] if ch.isdigit() else key
                    demo, said, acc = Demo(key, fg=fg, params=params), 0, 0.0
                    dash.appear.clear()
        if in_menu:
            dash.menu(msel, key, params, PARAMS, titles)
        else:
            if demo.nA.pilot_cancel:                       # arrow keys fly Skyhawk A
                kp = pygame.key.get_pressed()
                demo.pilot_input((1 if kp[pygame.K_RIGHT] else 0) - (1 if kp[pygame.K_LEFT] else 0),
                                 (1 if kp[pygame.K_UP] else 0) - (1 if kp[pygame.K_DOWN] else 0))
            sp = SPEEDS[si]
            in_ra = any(nd.state == "RA" or (nd.state == "ENGAGED" and demo.t - (nd.ra_t or 0) < 8) for nd in demo.nodes)
            slow = auto_slow and in_ra
            if slow: sp = min(sp, 0.25)
            if snapshots is not None: acc += 1.0
            elif step_once and demo.paused: acc += 1.0
            elif not demo.paused: acc += sp / 30.0
            while acc >= demo.FRAME - 1e-9 and not demo.finished:
                demo.update(); acc -= demo.FRAME
                if demo.collision: acc = min(acc, demo.FRAME)
            while said < len(demo.nA.say):
                audio.say(demo.nA.say[said]); said += 1
            if demo.collision and dash.new_explosion(id(demo.collision)):
                audio.play_boom()
            if demo.nA.state == "RA" and int(demo.t) != last_beep and sp <= 2 and not demo.collision:
                last_beep = int(demo.t); audio.tick_beep()
            audio.update(mute=sp > 2)
            canvas.fill(BG)
            dash.hardware(demo, audio, 0, 0, 380, CH - 22)
            dash.radar(demo, 380, 0, 700, 545, sp, slow)
            dash.calculations(demo, 380, 545, 700, CH - 567)
            dash.observer(demo, 1080, 0, 520, 430)
            dash.software(demo, 1080, 430, 520, CH - 452)
            if demo.paused:
                dash.text("PAUSED  -  P resume   N step 1 s   + / - speed", (730, 160), dash.f20b, AMBER, center=True)
            if demo.finished:
                dash.text("Scenario complete  -  R restart   1-3 scenario   M menu", (730, 512), dash.f16b, GREEN, center=True)
            dash.text("P pause   +/- speed   0 real time   N step   S slow-mo   SPACE USER button (then arrow keys)   "
                      "L cut LoRa   K cut coord   R restart   1-3 scenario   M menu   ESC quit",
                      (CW // 2, CH - 11), dash.f12, GREY, center=True)
        ws = win.get_size()
        win.fill(BG)
        if ws[0] >= CW and ws[1] >= CH:
            win.blit(canvas, ((ws[0] - CW) // 2, (ws[1] - CH) // 2))      # 1:1, sharp
        else:
            sc = min(ws[0] / CW, ws[1] / CH)
            img = pygame.transform.smoothscale(canvas, (int(CW * sc), int(CH * sc)))
            win.blit(img, ((ws[0] - img.get_width()) // 2, (ws[1] - img.get_height()) // 2))
        pygame.display.flip()
        if snaps and not in_menu and (demo.t >= snaps[0] or demo.finished):
            pygame.image.save(canvas, os.path.join(snapshot_dir, f"dash_{demo.key}_t{int(snaps[0])}.png"))
            snaps.pop(0)
            if not snaps: running = False
        if snapshots is None:
            clock.tick(30)
    pygame.quit()
