"""
build_complete_plant.py - builds complete_plant.json: the whole plant, every
kind of equipment the simulator has, with the designer's newer layers on
top of logistics_full.json (tools/build_logistics_full.py):

  sorting line      induct, barcode scanner, accumulation (3 zones), DWS,
                    profile banner, pusher + pop-up wheel diverters into
                    chutes, turntable to the no-read bin, guard door + light
                    curtain + e-stop on a safety relay; the infeed and the
                    sorter run in Hz through their drive trains (inverter ->
                    motor -> gearbox -> drum)
  pallet loop       roller loop with four 90-degree chain transfers, pallets,
                    stoppers, a 6-axis robot loading parts, an elevator to the
                    upper level - transfers and elevator on drive trains with
                    inverter ramps
  tote line         straight and curved roller conveyor, RFID reader, curved
                    belt
  operator panel    a control panel: START, STOP (NC), RESET; RUN and FAULT
                    lamps; a horn (control_panel.py)
  PLC tags          one tag table per area (Sorting, Pallet loop, Tote line,
                    Operator panel, System) - the designer's PLC Tags dock and
                    the AX generator's Sim_Tags

Run by the PLC (plc_mock.py's program, plc_logic.py) over TCP:

    python tools/build_complete_plant.py          # writes complete_plant.json
    python launcher.py                             # pick complete_plant.json, Mock PLC, Run
    (or: python main.py complete_plant.json  +  python plc_mock.py complete_plant.json --autostart)
"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "tools"))

import build_logistics_full  # noqa: E402
import control_panel  # noqa: E402
import plc_tags  # noqa: E402
from config_parser import ControlPanel, DriveConfig, PlcTag, SimulatorConfig  # noqa: E402

OUT = os.path.join(HERE, "complete_plant.json")

SORTING = {"induct", "infeed", "acc_1", "acc_2", "acc_3", "scan", "sorter", "branch_1", "branch_2", "branch_3",
           "tt_belt", "reject", "acc", "dws_1", "bcr_1", "banner_1", "curtain_1", "div_1", "div_2", "div_3", "tt_1",
           "chute_1", "chute_2", "chute_3", "reject_bin", "door_1", "safety_1", "estop", "reset"}
TOTES = {"totes", "tote_in", "tote_curve", "tote_up", "tote_turn", "tote_out", "rfid_1", "s_tote", "tote_store"}


def _area(component: str, pallet_ids: set) -> str:
    if component.startswith("op_panel"):
        return "Operator panel"
    if component == "clock":
        return "System"
    if component in SORTING:
        return "Sorting"
    if component in TOTES:
        return "Tote line"
    if component in pallet_ids:
        return "Pallet loop"
    return ""


def build() -> SimulatorConfig:
    config = build_logistics_full.build()

    # -- the operator panel: one enclosure with its devices, instead of loose
    # buttons and lamps (the PLC still finds START / STOP / RUN / FAULT by label)
    config.buttons = [b for b in config.buttons if b.id not in ("start", "stop")]
    config.leds = [lamp for lamp in config.leds if lamp.id not in ("lamp_run", "lamp_fault")]
    config.controlPanels = [ControlPanel(id="op_panel", x=0.9, y=-2.6, width=1.6, height=0.9, label="OPERATOR",
                                         elements=control_panel.default_elements())]
    control_panel.refresh(config)

    # -- drive trains: the sorting belts commanded in Hz (the PLC sends the
    # frequency for the design speed); the machines' drives with ramps
    belts = {b.id: b for b in config.belts}
    for belt_id in ("infeed", "sorter"):
        belt = belts[belt_id]
        belt.commandFormat = "frequency"
        belt.drive = DriveConfig(drumDiameter=0.1, rampUpTime=1.0, rampDownTime=0.8)   # gear ratio: derived
    for elevator in config.elevators:
        elevator.drive = DriveConfig(drumDiameter=0.2, rampUpTime=1.0, rampDownTime=1.0)
    for table in config.turntables:
        table.drive = DriveConfig(transmissionRatio=8.0, rampUpTime=0.5, rampDownTime=0.5)
    for transfer in config.transfers:
        transfer.drive = DriveConfig(drumDiameter=0.08, rampUpTime=0.3, rampDownTime=0.3)

    # -- PLC tags: a tag table per area
    machines = build_logistics_full._load("machines_demo.json", committed=True)
    pallet_ids = {c["id"] for items in machines.values() if isinstance(items, list)
                  for c in items if isinstance(c, dict) and "id" in c}
    tags, _ = plc_tags.tags_of(config)
    config.plcTags = [PlcTag(key=t.key, table=_area(t.component, pallet_ids)) for t in tags
                      if _area(t.component, pallet_ids)]
    return SimulatorConfig.model_validate(config.model_dump())


def main() -> int:
    config = build()
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(config.model_dump_json(indent=2) + "\n")
    counts = {k: len(v) for k, v in config.model_dump().items() if isinstance(v, list) and v}
    print(f"wrote {os.path.relpath(OUT, HERE)}: " + ", ".join(f"{n} {k}" for k, n in counts.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
