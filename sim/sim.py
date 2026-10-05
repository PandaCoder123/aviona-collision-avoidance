"""
sim.py - JSBSim aircraft + FlightGear links
  Plane        : one JSBSim c172x, flown through its own heading/altitude-hold autopilot
  FGMultiplayer: sends an aircraft to FlightGear as a multiplayer target (plane B)
Plane A is shown in FlightGear through JSBSim's built-in native-FDM output (UDP 5550).
"""
import math
import os
import socket
import struct
import tempfile
import jsbsim
from acas_core import Aircraft, FT, KT, FPM, NM, D2R

REF_LAT, REF_LON = 37.60, -122.25          # over San Francisco Bay (default FG scenery)
M_PER_DEG_LAT = 111320.0


def local_to_latlon(east_m, north_m):
    lat = REF_LAT + north_m / M_PER_DEG_LAT
    lon = REF_LON + east_m / (M_PER_DEG_LAT * math.cos(REF_LAT * D2R))
    return lat, lon


def latlon_to_local(lat, lon):
    north = (lat - REF_LAT) * M_PER_DEG_LAT
    east = (lon - REF_LON) * M_PER_DEG_LAT * math.cos(REF_LAT * D2R)
    return east, north


class Plane:
    """A JSBSim Cessna 172 flown by its autopilot. Maneuvers use 'carrot' setpoints:
    a heading target 15 deg ahead gives a steady ~15 deg bank; an altitude target
    100 ft away gives the autopilot's maximum climb or descent."""
    MANEUVER_THROTTLE = {"CLIMB": 1.0, "DESCEND": 0.45}

    def __init__(self, name, icao, east_nm, north_nm, alt_ft, hdg_deg, speed_kt,
                 climb_fpm=400, desc_fpm=700, fg_port=None):
        self.name, self.icao = name, icao
        self.label = f"Skyhawk {name}"            # Cessna 172 Skyhawk
        self.climb_fpm, self.desc_fpm = climb_fpm, desc_fpm
        self.fdm = jsbsim.FGFDMExec(None)
        self.fdm.set_debug_level(0)
        self.fdm.set_output_path(tempfile.gettempdir())      # model's own CSV log -> temp folder
        self.fdm.load_model("c172x")
        if fg_port:
            path = os.path.join(tempfile.gettempdir(), f"fg_out_{name}.xml")
            with open(path, "w") as f:
                f.write(f'<?xml version="1.0"?>\n<output name="127.0.0.1" type="FLIGHTGEAR" '
                        f'port="{fg_port}" protocol="UDP" rate="30"></output>\n')
            self.fdm.set_output_directive(path)
        lat, lon = local_to_latlon(east_nm * NM, north_nm * NM)
        f = self.fdm
        f["ic/lat-geod-deg"], f["ic/long-gc-deg"] = lat, lon
        f["ic/h-sl-ft"], f["ic/psi-true-deg"], f["ic/vt-kts"] = alt_ft, hdg_deg, speed_kt
        f.run_ic()
        f["propulsion/set-running"] = -1
        f["fcs/throttle-cmd-norm"], f["fcs/mixture-cmd-norm"] = 0.75, 1.0
        f["simulation/do_simple_trim"] = 1
        self.cruise_throttle = f["fcs/throttle-cmd-norm"]
        self.dt = f.get_delta_t()
        self.t = 0.0
        self.hold_heading = f["attitude/psi-deg"]
        self.hold_alt_agl = f["position/h-agl-ft"]
        f["ap/heading_setpoint"], f["ap/altitude_setpoint"] = self.hold_heading, self.hold_alt_agl
        f["ap/heading_hold"], f["ap/altitude_hold"] = 1, 1
        self.maneuver = "MAINTAIN"
        self.track = []
        self.crashed = False
        self.hold_track = hdg_deg                  # GPS course hold (LNAV-style)
        self.line = (*latlon_to_local(self.lat, self.lon), hdg_deg)
        self.input_roll = self.input_pitch = 0     # pilot stick input in MANUAL (-1, 0, +1)

    # ---------------------------------------------------------- control
    def track_deg(self):
        f = self.fdm
        return math.degrees(math.atan2(f["velocities/v-east-fps"], f["velocities/v-north-fps"])) % 360

    def command(self, maneuver):
        """MAINTAIN, TURN_RIGHT, TURN_LEFT, CLIMB, DESCEND, RESUME (hold present track/altitude)
        or MANUAL (pilot flies with the stick input)."""
        f = self.fdm
        if maneuver in ("RESUME", "MANUAL"):
            self._new_line()
            self.hold_alt_agl = f["position/h-agl-ft"]
            self.input_roll = self.input_pitch = 0
        self.maneuver = "MAINTAIN" if maneuver == "RESUME" else maneuver
        f["fcs/throttle-cmd-norm"] = self.MANEUVER_THROTTLE.get(self.maneuver, self.cruise_throttle)

    def set_input(self, roll, pitch):
        """Pilot stick in MANUAL: roll -1 left / +1 right, pitch +1 climb / -1 descend.
        Releasing the stick holds the track / altitude reached at that moment."""
        f = self.fdm
        if roll == 0 and self.input_roll != 0:
            self._new_line()
        if pitch == 0 and self.input_pitch != 0:
            self.hold_alt_agl = f["position/h-agl-ft"]
        self.input_roll, self.input_pitch = roll, pitch
        thr = {1: 1.0, -1: 0.45}.get(pitch, self.cruise_throttle)
        f["fcs/throttle-cmd-norm"] = thr

    def _new_line(self):
        self.hold_track = self.track_deg()
        self.line = (*latlon_to_local(self.lat, self.lon), self.hold_track)

    def _course_correction(self):
        """Cross-track error from the held course line -> heading correction (like GPS LNAV)."""
        x, y = latlon_to_local(self.lat, self.lon)
        x0, y0, trk = self.line
        th = math.radians(trk)
        e = (x - x0) * math.cos(th) - (y - y0) * math.sin(th)     # + = right of the line
        return -max(-20.0, min(20.0, 0.08 * e))

    def _update_setpoints(self):
        f, m = self.fdm, self.maneuver
        psi = f["attitude/psi-deg"]
        crab = ((psi - self.track_deg() + 180) % 360) - 180
        roll = self.input_roll if m == "MANUAL" else 0
        pitch = self.input_pitch if m == "MANUAL" else 0
        if m == "TURN_RIGHT" or roll > 0:   f["ap/heading_setpoint"] = (psi + (15 if roll == 0 else 25)) % 360
        elif m == "TURN_LEFT" or roll < 0:  f["ap/heading_setpoint"] = (psi - (15 if roll == 0 else 25)) % 360
        else:                               f["ap/heading_setpoint"] = (self.hold_track + crab + self._course_correction()) % 360
        agl = f["position/h-agl-ft"]
        if m == "CLIMB" or pitch > 0:       f["ap/altitude_setpoint"] = agl + 100
        elif m == "DESCEND" or pitch < 0:   f["ap/altitude_setpoint"] = agl - 100
        else:                               f["ap/altitude_setpoint"] = self.hold_alt_agl
        if m in ("CLIMB", "DESCEND"):
            self.hold_alt_agl = agl          # level off where we are when the maneuver ends

    def step(self, seconds):
        if self.crashed:
            return
        n = max(1, int(round(seconds / self.dt)))
        for i in range(n):
            if i % 12 == 0:
                self._update_setpoints()
            self.fdm.run()
        self.t += n * self.dt
        x, y = latlon_to_local(self.lat, self.lon)
        self.track.append((self.t, x, y, self.fdm["position/h-sl-ft"]))

    # ---------------------------------------------------------- state
    @property
    def lat(self): return self.fdm["position/lat-geod-deg"]
    @property
    def lon(self): return self.fdm["position/long-gc-deg"]

    def state(self):
        """Current state as an acas_core Aircraft (what the GNSS/IMU/baro estimator would give)."""
        f = self.fdm
        x, y = latlon_to_local(self.lat, self.lon)
        return Aircraft(self.name, self.icao, x, y, f["position/h-sl-ft"] * FT,
                        f["attitude/psi-rad"], f["velocities/vg-fps"] * FT,
                        f["velocities/h-dot-fps"] * FT, f["attitude/phi-rad"],
                        self.climb_fpm * FPM, self.desc_fpm * FPM,
                        f["position/terrain-elevation-asl-ft"] * FT)

    def turn_rate_dps(self):
        return self.fdm["velocities/psidot-rad_sec"] / D2R

    def attitude(self):
        f = self.fdm
        return (f["attitude/psi-rad"], f["attitude/theta-rad"], f["attitude/phi-rad"],
                (f["velocities/u-fps"] * FT, f["velocities/v-fps"] * FT, f["velocities/w-fps"] * FT))


# -------------------------------------------------------------------- FlightGear multiplayer
WGS84_A, WGS84_E2 = 6378137.0, 6.69437999014e-3


def geod_to_ecef(lat_deg, lon_deg, alt_m):
    lat, lon = lat_deg * D2R, lon_deg * D2R
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    return ((n + alt_m) * math.cos(lat) * math.cos(lon),
            (n + alt_m) * math.cos(lat) * math.sin(lon),
            (n * (1 - WGS84_E2) + alt_m) * math.sin(lat))


def q_mul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw)


def q_lonlat(lon, lat):
    """SimGear SGQuat::fromLonLatRad: earth-centered frame -> local horizontal (NED)."""
    zd2, yd2 = 0.5 * lon, -0.25 * math.pi - 0.5 * lat
    sz, sy, cz, cy = math.sin(zd2), math.sin(yd2), math.cos(zd2), math.cos(yd2)
    return (cz * cy, -sz * sy, cz * sy, sz * cy)


def q_euler(yaw, pitch, roll):
    """SimGear SGQuat::fromYawPitchRoll."""
    zd2, yd2, xd2 = 0.5 * yaw, 0.5 * pitch, 0.5 * roll
    sz, sy, sx = math.sin(zd2), math.sin(yd2), math.sin(xd2)
    cz, cy, cx = math.cos(zd2), math.cos(yd2), math.cos(xd2)
    return (cx*cy*cz + sx*sy*sz, sx*cy*cz - cx*sy*sz, cx*sy*cz + sx*cy*sz, cx*cy*sz - sx*sy*cz)


def angle_axis(q):
    w, x, y, z = q
    nrm = math.sqrt(w*w + x*x + y*y + z*z)
    ang = math.acos(max(-1.0, min(1.0, w / nrm)))
    s = math.sin(ang)
    if abs(s) < 1e-12:
        return (0.0, 0.0, 0.0)
    k = 2 * ang / (nrm * s)
    return (x * k, y * k, z * k)


class FGMultiplayer:
    """Minimal FlightGear multiplayer POS_DATA sender (protocol 1.1, no property block).
    FlightGear must be started with --multiplay=in,10,127.0.0.1,<port>."""
    MAGIC, VERSION, POS_DATA_ID = 0x46474653, 0x00010001, 7
    MODEL = "Aircraft/c172p/Models/c172p.xml"

    def __init__(self, callsign="ACFT-B", host="127.0.0.1", port=5000):
        self.callsign, self.addr = callsign, (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @classmethod
    def packet(cls, callsign, t, lat, lon, alt_m, yaw, pitch, roll, uvw):
        q = q_mul(q_lonlat(lon * D2R, lat * D2R), q_euler(yaw, pitch, roll))
        body = (struct.pack(">96s", cls.MODEL.encode())
                + struct.pack(">dd", t, 0.1)
                + struct.pack(">3d", *geod_to_ecef(lat, lon, alt_m))
                + struct.pack(">3f", *angle_axis(q))
                + struct.pack(">3f", *uvw)
                + struct.pack(">9f", *([0.0] * 9))
                + struct.pack(">I", 0))
        hdr = (struct.pack(">6I", cls.MAGIC, cls.VERSION, cls.POS_DATA_ID, 32 + len(body), 100, 0)
               + callsign.encode()[:7].ljust(8, b"\0"))
        return hdr + body

    def send(self, plane: Plane):
        yaw, pitch, roll, uvw = plane.attitude()
        pkt = self.packet(self.callsign, plane.t, plane.lat, plane.lon,
                          plane.fdm["position/h-sl-ft"] * FT, yaw, pitch, roll, uvw)
        try:
            self.sock.sendto(pkt, self.addr)
        except OSError:
            pass
