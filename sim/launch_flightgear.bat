@echo off
REM ---------------------------------------------------------------------------
REM  Starts FlightGear as the visual for the PS2 TCAS demo.
REM    Aircraft A  : driven by JSBSim in Python  (native-fdm, UDP 5550)
REM    Aircraft B  : injected by Python as multiplayer traffic (UDP 5000)
REM  Start this FIRST, wait until the cockpit appears, then run:
REM      python run_demo.py      and press P in the demo window.
REM ---------------------------------------------------------------------------
set FGFS=
for /d %%D in ("C:\Program Files\FlightGear*") do if exist "%%D\bin\fgfs.exe" set FGFS=%%D\bin\fgfs.exe
if "%FGFS%"=="" (
  echo Could not find fgfs.exe under C:\Program Files\FlightGear*
  echo Edit this file and set FGFS to the full path of fgfs.exe
  pause
  exit /b 1
)
echo Using %FGFS%
"%FGFS%" ^
  --aircraft=c172p ^
  --fdm=null ^
  --native-fdm=socket,in,60,,5550,udp ^
  --lat=37.60 --lon=-122.25 --altitude=4500 --heading=0 ^
  --timeofday=noon ^
  --disable-ai-traffic ^
  --disable-real-weather-fetch ^
  --disable-terrasync ^
  --callsign=ACFT-A ^
  --multiplay=out,10,127.0.0.1,5002 ^
  --multiplay=in,10,127.0.0.1,5000 ^
  --geometry=1280x720
