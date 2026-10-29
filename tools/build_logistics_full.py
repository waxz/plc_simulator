"""
build_logistics_full.py - builds logistics_full.json, a complete logistics
system with every kind of machine, sensor and device the simulator has,
run by the PLC (plc_mock.py's program, plc_logic.py) over TCP:

  sorting line (logistics_demo.json)      induct, barcode scanner, accumulation
                                          (3 zones), DWS, laser banner, pusher +
                                          wheel diverters into chutes, turntable to
                                          the no-read bin, guard door + light
                                          curtain + e-stop on a safety relay
  pallet loop (machines_demo.json, below) roller loop with four 90-degree
                                          transfers, pallets, stoppers, a 6-axis
                                          robot loading parts (pick sensor), an
                                          elevator to an upper level
  tote line (above)                       straight and curved roller conveyor,
                                          RFID reader, curved belt
  operator panel                          START / STOP / RESET / E-STOP, RUN and
                                          FAULT lamps

    python tools/build_logistics_full.py            # writes logistics_full.json
    python main.py logistics_full.json              # the simulator (TCP 127.0.0.1:5001)
    python plc_mock.py logistics_full.json --autostart
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from config_parser import (Belt, ControlButton, LedIndicator, Scanner, Sensor, SimulatorConfig,  # noqa: E402
                           Sink, Source)

PALLET_AREA_DY = -12.0       # the pallet loop sits below the sorting line
OUT = os.path.join(HERE, "logistics_full.json")


def _load(name: str, committed: bool = False) -> dict:
    """A demo layout as data (the committed version: the working copy may
    hold someone's edits)."""
    if committed:
        text = subprocess.run(["git", "show", f"HEAD:{name}"], cwd=HERE, capture_output=True, text=True,
                              check=True).stdout
        return json.loads(text)
    with open(os.path.join(HERE, name), encoding="utf-8") as f:
        return json.load(f)


def _shift(data: dict, dx: float, dy: float) -> dict:
    for key, items in data.items():
        if not isinstance(items, list):
            continue
        for item in items:
            if isinstance(item, dict) and "x" in item and "y" in item:
                item["x"] += dx
                item["y"] += dy
                for px, py in (("pickX", "pickY"), ("placeX", "placeY")):
                    if px in item:
                        item[px] += dx
                        item[py] += dy
    return data


def _merge(*layouts: dict) -> dict:
    merged: dict = {}
    for data in layouts:
        for key, value in data.items():
            if isinstance(value, list):
                merged.setdefault(key, [])
                ids = {c["id"] for c in merged[key] if isinstance(c, dict) and "id" in c}
                for c in value:
                    if isinstance(c, dict) and c.get("id") in ids:
                        raise ValueError(f"id {c['id']!r} twice in {key}")
                    merged[key].append(c)
            else:
                merged.setdefault(key, value)
    return merged


def _add(data: dict, key: str, *models) -> None:
    data.setdefault(key, []).extend(m.model_dump() for m in models)


def build() -> SimulatorConfig:
    sorting = _load("logistics_demo.json")
    pallets = _shift(_load("machines_demo.json", committed=True), 0.0, PALLET_AREA_DY)
    data = _merge(pallets, sorting)          # the pallet demo's simulation settings (physicsDt, ...)

    # -- operator panel: START / STOP first (the PLC picks its buttons by label)
    panel = [ControlButton(id="start", x=0.4, y=-2.2, width=0.4, height=0.3, rotation=0.0, label="START",
                           color="#16a34a"),
             ControlButton(id="stop", x=1.0, y=-2.2, width=0.4, height=0.3, rotation=0.0, label="STOP",
                           color="#64748b", normallyClosed=True)]
    data["buttons"] = [b.model_dump() for b in panel] + data.get("buttons", [])
    _add(data, "leds",
         LedIndicator(id="lamp_run", x=0.4, y=-2.8, radius=0.15, label="RUN", color="#22c55e"),
         LedIndicator(id="lamp_fault", x=1.0, y=-2.8, radius=0.15, label="FAULT", color="#ef4444"))

    # -- sorting line: barcode scanner at the induct, a laser banner before the
    # diverters (a parcel profile), a light curtain on the safety relay
    _add(data, "scanners", Scanner(id="bcr_1", x=4.5, y=0.0, width=0.4, height=1.2, scannerType="barcode",
                                   label="Barcode"))
    _add(data, "sensors",
         Sensor(id="banner_1", x=12.7, y=0.0, width=0.1, height=1.0, rotation=0.0, label="Profile banner",
                color="#f97316", sensorType="laser_banner", laserCount=16, beamDistance=0.06),
         Sensor(id="curtain_1", x=27.0, y=1.5, width=0.1, height=1.2, rotation=90.0, label="Light curtain",
                color="#facc15", sensorType="laser_banner", laserCount=24, beamDistance=0.05))
    for circuit in data.get("safety", []):
        circuit["curtainIds"] = sorted(set(circuit.get("curtainIds", [])) | {"curtain_1"})

    # -- tote line: roller conveyor, a curved roller turning left, a belt up, a
    # curved belt turning right, out; an RFID reader on the rollers
    y0 = 8.0
    _add(data, "sources", Source(id="totes", x=0.6, y=y0, rotation=0.0, interval=6.0, minWidth=0.5, maxWidth=0.5,
                                 minHeight=0.4, maxHeight=0.4, label="Totes", barcodeStart=500000))
    _add(data, "belts",
         Belt(id="tote_in", type="linear", x=2.0, y=y0, length=4.0, beltWidth=1.0, rotation=0.0, speed=0.5,
              directionAngle=0.0, color="#4b5563", surface="roller"),
         Belt(id="tote_curve", type="curved", shape="arc", x=4.0, y=y0 + 1.5, beltWidth=1.0, rotation=0.0,
              radius=1.5, startAngle=270.0, endAngle=360.0, speed=0.5, directionAngle=90.0, color="#4b5563",
              surface="roller"),
         Belt(id="tote_up", type="linear", x=5.5, y=y0 + 3.0, length=3.0, beltWidth=1.0, rotation=90.0, speed=0.5,
              directionAngle=0.0, color="#3b82f6"),
         Belt(id="tote_turn", type="curved", shape="arc", x=7.0, y=y0 + 4.5, beltWidth=1.0, rotation=0.0,
              radius=1.5, startAngle=90.0, endAngle=180.0, speed=0.5, directionAngle=-90.0, color="#3b82f6"),
         Belt(id="tote_out", type="linear", x=8.5, y=y0 + 6.0, length=3.0, beltWidth=1.0, rotation=0.0, speed=0.5,
              directionAngle=0.0, color="#3b82f6"))
    _add(data, "scanners", Scanner(id="rfid_1", x=2.5, y=y0, width=0.4, height=1.2, scannerType="rfid",
                                   label="RFID"))
    _add(data, "sensors", Sensor(id="s_tote", x=9.0, y=y0 + 6.0, width=0.1, height=1.0, rotation=0.0,
                                 label="Tote out", color="#ff0000"))
    _add(data, "sinks", Sink(id="tote_store", x=10.5, y=y0 + 6.0, width=1.0, height=1.2, label="Tote store"))

    # -- the PLC link: TCP, the simulator listening on 127.0.0.1:5001
    settings = data.setdefault("mqttSettings", {})
    settings.update(enabled=True, transport="tcp", tcpBindHost="127.0.0.1", tcpServerHost="127.0.0.1",
                    tcpPort=5001, endianness="LE")
    data["background"] = None
    return SimulatorConfig.model_validate(data)


def main() -> int:
    config = build()
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(config.model_dump_json(indent=2) + "\n")
    counts = {k: len(v) for k, v in config.model_dump().items() if isinstance(v, list) and v}
    print(f"wrote {os.path.relpath(OUT, HERE)}: " + ", ".join(f"{n} {k}" for k, n in counts.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
