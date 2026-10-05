# Setup and run guide

Cessna 172 Skyhawks (124 KTAS) fly in JSBSim, a 6-DoF flight model running inside Python. Each
aircraft carries a **Heltec Wireless Tracker** (ESP32-S3 + SX1262 LoRa + UC6580 GNSS). The units talk
directly over **LoRa P2P at 915 MHz**: 33-byte state packets 5 times a second in GNSS-timed slots,
plus 12-byte coordination messages. Each unit predicts conflicts, negotiates a joint maneuver with
every aircraft involved, warns the pilot, and if the pilot does not respond within the countdown,
flies the maneuver through the aircraft's autopilot.

## 1. Install (Windows, once)

1. **Python 3.10 - 3.12** from python.org. In the installer tick **"Add python.exe to PATH"**.
2. Open the `sim` folder in File Explorer, click the address bar, type `cmd`, press Enter.
3. `pip install -r requirements.txt`  (installs jsbsim, numpy, pygame, pyttsx3)
4. **FlightGear** from flightgear.org (Windows installer), default location
   `C:\Program Files\FlightGear ...`. The default install includes the San Francisco Bay scenery the
   demo flies over and the Cessna 172P Skyhawk model. No extra downloads needed.
5. Test without graphics: `python run_demo.py --headless --all`
   You should see three scenarios, each "COLLISION!" without the system and a safe pass with it.

## 2. Run with FlightGear

1. Double-click **`launch_flightgear.bat`**. Wait until the Skyhawk cockpit is visible (1-2 min).
   - If Windows Firewall asks about FlightGear or Python, click **Allow** (local UDP ports 5550 and 5000).
   - If the script cannot find FlightGear, open it in Notepad and set `FGFS` to the full path of `fgfs.exe`.
2. In the command window: **`python run_demo.py`**
3. In the start menu choose the scenario and parameters (arrow keys), press **Enter**.
4. Arrange windows: FlightGear on one side, the dashboard on the other (or two monitors).
5. In FlightGear press **`V`** to cycle views: the cockpit view and chase view show the other
   Skyhawks approaching (they are multiplayer aircraft named ACFT-B ... ACFT-E).
   Check *Multiplayer > Pilot List* to confirm they are being received.

What FlightGear shows and how:
| Aircraft | How it reaches FlightGear |
|---|---|
| Skyhawk A (your cockpit) | JSBSim native-FDM output, UDP port 5550, 30 updates per simulated second |
| Skyhawks B-E | Multiplayer position packets from Python, UDP port 5000, 10 per real second |

Without FlightGear: `python run_demo.py --no-fg`

## 3. Start menu parameters

| Parameter | Range | Default | Meaning |
|---|---|---|---|
| Scenario | S1 - S3 | S1 | see below |
| Aircraft in scenario 2 | 3 - 5 | 3 | Skyhawks converging on one point |
| Cruise speed | 90 - 130 kt | 124 | all aircraft |
| Lateral offset | 0 - 400 m | 0 | 0 = aircraft would hit without the system |
| Pilot response window | 1 - 8 s | 3 | countdown before the autopilot takes over |
| LoRa packet loss | 0 - 60 % | 0 | random loss on the link |

The same parameters work from the command line, e.g.
`python run_demo.py --headless --scenario S2 --aircraft 5 --loss 30`.

## 4. Scenarios

| | Scenario | Shows |
|---|---|---|
| S1 | Head-on, two Skyhawks | Hollow diamond at 5 nm, filled at 4 nm, TA at 3 nm, plan agreed over LoRa, both turn right |
| S2 | 3-5 Skyhawks converging on one point | Everyone solved together; plan re-negotiated as aircraft join; all turn right (roundabout) |
| S3 | One-way: Skyhawk B transmits only | A hears B and acts alone; B never learns about A |

## 5. Keys during the simulation

| Key | Action |
|---|---|
| `P` | Pause / resume |
| `+` / `-`, `0` | Speed 0.1x ... 16x, back to real time |
| `N` | While paused: advance exactly 1 s |
| `S` | Auto slow-motion during the RA |
| `SPACE` | USER button: the pilot takes control of Skyhawk A |
| Arrow keys | After SPACE: left/right bank 25 deg, up/down climb/descend; release = hold course/altitude |
| `L` / `K` | Cut / restore the LoRa link / only coordination messages |
| `R`, `1`-`3`, `M`, `ESC` | Restart, other scenario, back to start menu, quit |

If aircraft come within one wingspan (11 m) and 3 m vertically, it is a **midair collision**: both
aircraft are destroyed (explosion, sound), disappear from FlightGear, and the dashboard explains why.
Try it: scenario S3, press SPACE when the RA appears and do not steer.

## 6. Files

| File | What it does |
|---|---|
| `run_demo.py` | Entry point, scenarios, parameters, collision detection, FlightGear multiplayer |
| `acas_core.py` | Collision logic: tau / DMOD / CPA, alerting, joint maneuver search |
| `tcas_node.py` | Onboard unit: LoRa link, negotiation levels, countdown, pilot override |
| `packet.py` | The real bytes: 33-byte STATE packet, 12-byte coordination messages, CRC-16 |
| `sim.py` | JSBSim Skyhawks, GPS course hold, maneuvers, pilot stick, FlightGear outputs |
| `ui.py` | Start menu, dashboard, audio, explosion |
| `launch_flightgear.bat` | Starts FlightGear configured for this demo |

## 7. Troubleshooting

- **FlightGear cockpit does not move:** start FlightGear first, do not use `--no-fg`, nothing else on UDP 5550.
- **Other Skyhawks not visible:** check *Multiplayer > Pilot List*. The multiplayer packets are built to the
  FlightGear protocol but were not tested against a real FlightGear before delivery. If they are missing,
  send the text from FlightGear's console window. The dashboard still shows every aircraft.
- **Choppy FlightGear at slow speeds:** at 0.1x FlightGear only gets 3 cockpit updates per second; use 0.5x or more.
- **Blurry text:** the dashboard is 1600x920 and DPI-aware; on smaller screens it must be scaled down.
- **No voice:** `pip install pyttsx3`, delete the `sounds` folder, restart (tones are used otherwise).

## 8. Verified / not verified

- Verified (Linux, JSBSim 1.3.1): every scenario and parameter extreme in JSBSim physics; collisions without
  the system, safe passes with it; 5-aircraft search in about 0.05 s; JSBSim emits FlightGear native-FDM
  packets; multiplayer packet structure and orientation math; dashboard rendering; CRC round trip.
- Not verified here (no FlightGear or display): FlightGear actually showing the aircraft, Windows audio.

Thresholds are configurable starting values for a non-certified prototype (`CFG` in `acas_core.py`).
