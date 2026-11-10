# The designer's equipment model

How the designer describes equipment, not only where it stands: what a
conveyor, a roller line or an elevator is made of, layer by layer, from the
mechanics to the motor to the PLC signal. This document compares the approach
with established virtual commissioning tools, describes what is built, and
plans the next steps.

## 1. How virtual commissioning tools structure a machine

| Tool | How a machine is modelled | How it reaches the PLC |
|---|---|---|
| Siemens NX MCD + SIMIT | A digital twin in three layers: automation (PLC logic), electrical and behaviour (drives, sensors, periphery: SIMIT), physical and kinematic (rigid bodies, joints, collisions: MCD) | Signals named like the PLC tags, mapped (auto-mapping by name) through a signal adapter |
| SIMIT drive models (SINAMICS DriveSim) | A drive and its motor as one validated model, linked to the drive documentation | PROFIdrive telegrams, as the real drive |
| Emulate3D (Rockwell) | Catalog components (smart conveyors) with property categories; a separate Motor / motor-controller aspect can replace a conveyor's built-in motor | Tags, browsed and bound per component |
| Visual Components | A component = geometry + behaviours (paths, sensors, servos) linked by signals; properties per behaviour (a path's speed in mm/s) | Signals connected to PLC variables (OPC UA, vendor connectors) |
| FACTORY I/O | Parts from a catalog (sensors, conveyors, stations); each part brings its own I/O points | One driver screen: sensors on one side, actuators on the other, addresses in the middle |

The common idea: a machine is not one bag of fields. It is **mechanics**
(what physically moves), a **drive** (what moves it), **signals** (what it
reports and accepts) and the **PLC side** (where those signals go), and each
tool lets a user configure each layer where it belongs. The tools differ in
weight: MCD and SIMIT are engineering tools with full physics and validated
drive models; FACTORY I/O is a ready-made catalog with fixed I/O.

## 2. What the designer does

This simulator sits between the two ends: a catalog and a 2D layout like
FACTORY I/O, with configurable mechanics, drives and signals per component in
the spirit of the layered tools, and a generated PLC interface (the AX
project's generated types) instead of manual tag mapping.

### Layers in the property panel

Every component's properties are grouped by layer, in the order a machine is
built, in both the designer and the simulator window's inspector
(`hmi_style.PROPERTY_GROUP_ORDER`):

| Group | Holds | Examples |
|---|---|---|
| Identity | what it is | id, label, sensor type |
| Layout | where it stands, its footprint | x, y, rotation, level, length, width |
| Mechanics | the mechanical design | belt or roller surface, roller pitch, drum / roller diameter, chain ratio, gear ratio; lift levels; robot reach |
| Devices | the devices built into it | a control panel's push buttons, lamps and buzzers |
| Drive | how it is driven | drive type (setpoint m/s, VFD, inverter in Hz), motor preset, motor nameplate, max frequency, ramps, encoder, and the computed result |
| Motion | how it moves | design speed, direction, lift speed, cycle times |
| Signal | what a sensor reports | delays, light-on / dark-on, beams, packing |
| Operation | what it does | parcel sizes, buttons, hold times, linked devices |
| Display | how it looks | colour |
| PLC I/O | where the signals go | command source, every command and feedback signal with its PLC type |

### The drive train (belts and roller conveyors)

A belt commanded in Hz (drive type "Inverter, Hz") shows its train, from the
surface back to the motor: drum / roller diameter, chain, gearbox (Mechanics),
then the motor and the inverter (Drive). A **Motor** preset fills the
nameplate (1450 rpm at 50 Hz, ...). The **Result** row computes the chain
while you type, for example
`50 Hz → motor 1450 rpm → 72.5 rpm at the drum → 0.380 m/s; design 0.3 m/s = 39.5 Hz`,
and turns into a warning when the design speed needs more than the inverter's
maximum frequency. Values left unset show what the simulation derives.

### Composed equipment: the control panel

A control panel is equipment made of devices, as an Emulate3D component
with aspects or a Visual Components component with behaviours: one
enclosure whose push buttons, lamps and buzzers are its properties (the
Devices table), not components placed one by one. Each device becomes an
ordinary button or lamp (`<panel>_<name>`, control_panel.py), so the PLC
exchange, the generated ST and the simulator need nothing new; the layout
file keeps only the panel with its devices.

## 3. Next steps

Each step is usable on its own.

1. **Drive units as components.** A drive unit (motor, gearbox, inverter)
   as its own library item, assignable to one or several conveyors: one motor
   driving several roller zones by chain, as on real roller lines, and a
   catalog of the units actually used (as Emulate3D's motor aspect).
2. **Machines with drive trains.** Done: an elevator's lift speed comes
   from its hoist (motor, gearbox, chain, drum), a turntable's turn speed
   from its slewing drive (motor, gearbox, ring / pinion, in deg/s) and a
   transfer's chain speed from its sprocket, all with ramped acceleration
   and braking and the same Drive rows and Result.
3. **Sensors with an electrical layer.** Supply and output type (PNP / NPN,
   NO / NC), response time, and the wiring to an input module channel.
4. **I/O table: addresses and tags.** Done: every signal as a PLC tag, as
   TIA Portal's PLC tags editor - tag tables, editable names and comments,
   %I / %Q addresses, filter, sort, CSV and an .xlsx for import into TIA
   Portal. Next: the tag names in the generated ST, and a link to the frame
   picture (`tools/frame_map.py`).
5. **Templates of whole stations.** Done: a selected group saved to the
   library as one station and placed again with new ids, its internal links
   and belt connections pointing to the new copies (component_library.py).
   Next: parameters asked on placing (a lane's length, its name prefix).

## Sources

- [Siemens NX MCD course notes (FH Aachen)](https://i40.fh-aachen.de/courses/dta/basics/mcd_basic/index.html)
- [NX Open SignalMappingConnectionBuilder](https://docs.plm.automation.siemens.com/data_services/resources/nx/12/nx_api/custom/en_US/nxopen_python_ref/NXOpen.Mechatronics.SignalMappingConnectionBuilder.html)
- [Siemens virtual commissioning webinar, part 1](https://assets.new.siemens.com/siemens/assets/api/uuid:cc65aa3b-b2d4-4b77-9fa6-84185abd1575/2021-1005-virtual-commissioning-webinar-part1.pdf)
- [SIMATIC Machine Simulator slides](https://assets.new.siemens.com/siemens/assets/api/uuid:41efc594-38ae-4d51-9744-2240a67bfe92/SIMATIC-Machine-Simulator--SalesSlides-2021-01-26-EN.pdf)
- [SINAMICS DriveSim Basic](https://qualitymag.com/articles/98312-sinamics-drivesim-basic)
- [Emulate3D mechatronic updates (Rockwell)](https://www.rockwellautomation.com/content/dam/rockwell-automation/documents/pdf/events/roklive-emea-2024/digital-engineering-summit/mechatronic-updates.pdf)
- [Visual Components: behaviors](https://help.visualcomponents.com/4.9/Premium/en/English/Component%20Modeling/Behaviors/Behaviors.htm)
- [FACTORY I/O: controlling with a PLC](https://docs.factoryio.com/getting-started/controlling-with-a-plc/)
- [FACTORY I/O: parts](https://docs.factoryio.com/manual/parts/)
