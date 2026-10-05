"""
packet.py - the bytes that actually go over the P2P link

STATE (33 bytes, big-endian)
  B  type/version      0x11
  3s ICAO address      24-bit
  H  sequence number
  I  time (ms)
  i  latitude   (1e-7 deg)       i  longitude (1e-7 deg)
  H  altitude   (ft + 1000)
  h  vel north  (cm/s)           h  vel east  (cm/s)      h  vertical speed (cm/s)
  b  turn rate  (0.1 deg/s)
  B  position sigma (0.5 m)      B  velocity sigma (0.05 m/s)
  B  flags  bit0 GNSS ok, bit1 autopilot available, bit2 automation engaged, bit3 pilot manual
  B  intent (maneuver code)
  H  CRC-16/CCITT

COORDINATION (12 bytes)
  B type (0x21 PROPOSE, 0x22 ACK, 0x23 COMMIT, 0x24 MANUAL, 0x25 CLEAR)
  3s from ICAO   3s to ICAO   B encounter id   B maneuver for sender   B maneuver for receiver
  H CRC-16/CCITT
"""
import struct

STATE_FMT = ">B3sHIiiHhhhbBBBB"
COORD_FMT = ">B3s3sBBB"
T_STATE, T_PROPOSE, T_ACK, T_COMMIT, T_MANUAL, T_CLEAR = 0x11, 0x21, 0x22, 0x23, 0x24, 0x25
TYPE_NAME = {T_STATE: "STATE", T_PROPOSE: "PROPOSE", T_ACK: "ACK", T_COMMIT: "COMMIT",
             T_MANUAL: "MANUAL", T_CLEAR: "CLEAR"}
MANEUVER_CODE = {"MAINTAIN": 0, "TURN_RIGHT": 1, "TURN_LEFT": 2, "CLIMB": 3, "DESCEND": 4, None: 255}
CODE_MANEUVER = {v: k for k, v in MANEUVER_CODE.items()}


def crc16(data: bytes, crc=0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _clip(v, lo, hi):
    return max(lo, min(hi, int(round(v))))


def encode_state(icao, seq, t, lat, lon, alt_ft, vn, ve, vs, turn_dps, sig_pos, sig_vel,
                 flags, intent):
    body = struct.pack(STATE_FMT, T_STATE, icao.to_bytes(3, "big"), seq & 0xFFFF,
                       int(t * 1000) & 0xFFFFFFFF, int(round(lat * 1e7)), int(round(lon * 1e7)),
                       _clip(alt_ft + 1000, 0, 65535), _clip(vn * 100, -32768, 32767),
                       _clip(ve * 100, -32768, 32767), _clip(vs * 100, -32768, 32767),
                       _clip(turn_dps * 10, -128, 127), _clip(sig_pos * 2, 0, 255),
                       _clip(sig_vel * 20, 0, 255), flags & 0xFF, MANEUVER_CODE.get(intent, 255))
    return body + struct.pack(">H", crc16(body))


def decode_state(data: bytes):
    if len(data) != 33 or crc16(data[:-2]) != struct.unpack(">H", data[-2:])[0]:
        return None
    (typ, icao, seq, tms, lat, lon, alt, vn, ve, vs, tr, sp, sv, flags, intent) = \
        struct.unpack(STATE_FMT, data[:-2])
    if typ != T_STATE:
        return None
    return dict(icao=int.from_bytes(icao, "big"), seq=seq, t=tms / 1000.0, lat=lat / 1e7,
                lon=lon / 1e7, alt_ft=alt - 1000, vn=vn / 100, ve=ve / 100, vs=vs / 100,
                turn_dps=tr / 10, sig_pos=sp / 2, sig_vel=sv / 20, flags=flags,
                intent=CODE_MANEUVER.get(intent))


def encode_coord(typ, frm, to, enc_id, man_sender, man_receiver):
    body = struct.pack(COORD_FMT, typ, frm.to_bytes(3, "big"), to.to_bytes(3, "big"),
                       enc_id & 0xFF, MANEUVER_CODE.get(man_sender, 255),
                       MANEUVER_CODE.get(man_receiver, 255))
    return body + struct.pack(">H", crc16(body))


def decode_coord(data: bytes):
    if len(data) != 12 or crc16(data[:-2]) != struct.unpack(">H", data[-2:])[0]:
        return None
    typ, frm, to, enc, ms, mr = struct.unpack(COORD_FMT, data[:-2])
    return dict(type=TYPE_NAME.get(typ, hex(typ)), frm=int.from_bytes(frm, "big"),
                to=int.from_bytes(to, "big"), enc=enc, man_sender=CODE_MANEUVER.get(ms),
                man_receiver=CODE_MANEUVER.get(mr))


def hexdump(data: bytes):
    return " ".join(f"{b:02X}" for b in data)


if __name__ == "__main__":
    p = encode_state(0xA1, 7, 12.5, 37.6, -122.25, 4500, 56.5, -0.3, 0.1, 0.0, 5, 0.3, 3, "TURN_RIGHT")
    print(len(p), hexdump(p)); print(decode_state(p))
    c = encode_coord(T_PROPOSE, 0xA1, 0xB2, 73, "TURN_RIGHT", "TURN_RIGHT")
    print(len(c), hexdump(c)); print(decode_coord(c))
    bad = bytearray(p); bad[10] ^= 0x01
    print("corrupted packet rejected:", decode_state(bytes(bad)) is None)
