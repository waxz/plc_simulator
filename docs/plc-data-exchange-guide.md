# PLC data exchange guide

This guide covers how the simulator and a PLC exchange data, and how to work
with that data on each side:

- **The wire:** frames, byte order and packing.
- **The definition:** what a layout puts in the frames, and how to add to it.
- **The mock PLC (`plc_mock.py`):** how it extracts the simulator's data and
  composes its commands.
- **A real PLC (Siemens SIMATIC AX, ST):** the same with generated types and
  serializers.

`logistics_full.json` (every kind of machine, sensor and device) is the
running example. For the build and CI workflow of the AX side see
[plc-exchange-workflow.md](plc-exchange-workflow.md).

> Status. Everything Python-side here runs and is tested
> (`tools/verify_exchange.py`, `test_device_kinds.py`, `test_st_export.py`).
> The generated ST has not been compiled yet: `apax build` stops on an
> expired SIMATIC AX license token. Build and test it on the real compiler
> before relying on it.

---

## 1. The big picture

```
 simulator (main.py)                                   PLC (plc_mock.py, or the AX PLC)
 TCP server, 127.0.0.1:5001
        |  IO_MAPPING "input"  frame every 10 ms  --------->  sensors, buttons, device feedback, clock
        |  <---------  IO_MAPPING "output" frame every scan   lamps, belt speeds, device commands
        |  <---------  LAYOUT_CHECK (once per connection)     "are we using the same frames?"
        |  LAYOUT_CHECK reply  ------------------------------>  OK / MISMATCH
        |  <---------  METRIC_LAYOUT_n (optional)             the PLC's own measurements
```

- **The simulator is the plant.** It owns the physics and reports what its
  sensors and devices see. The PLC owns the logic. Nothing moves until the
  PLC commands it: at start every PLC-driven command is at rest.
- **"Input" and "output" are named from the PLC's side.** Input is
  simulator -> PLC; output is PLC -> simulator. Inside the Python mapping
  code the same frames are built with `direction="out"` (input) and
  `direction="in"` (output), because there the names are from the
  simulator's side.
- **Both frames come from the layout file alone.** The simulator, the mock
  PLC and the code generator all call the same function
  (`mqtt_data_exchange.io_topics`). The same layout file on both sides
  therefore means the same bytes.

---

## 2. The wire

### 2.1 Frame

Every message is one frame (`tcp_framed_transport.TcpFrameCodec`, and
`MagicFrameCodec` in the AX project):

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `12 34 56 78` |
| 4 | 1 | node id (0 by default; tells several PLCs apart) |
| 5 | 1 | frame type (below) |
| 6 | 4 | payload length, UDINT, in the exchange's byte order (`LE`) |
| 10 | n | payload |
| 10+n | 4 | magic again (confirms the frame boundary) |

The receiver scans for the magic, checks the trailer, and resynchronizes by
one byte on anything that does not fit, so a broken frame never loses the
ones behind it. Payloads can be at most 2048 bytes.

| Frame type | Value | Direction | Payload |
|---|---|---|---|
| `IO_MAPPING` | 1 | both | the input frame (sim -> PLC) or the output frame (PLC -> sim) |
| `METRIC_LAYOUT_1/2/3` | 2 / 3 / 5 | PLC -> sim | the PLC's own measurements (web dashboard) |
| `LAYOUT_CHECK` | 7 | both | layout fingerprints and sizes (section 2.4) |
| `INPUT_IMAGE` | 8 | PLC -> monitor | the input image the PLC worked on (Foxglove bridge) |
| `JSON` | 9 | PLC -> monitor | a UTF-8 JSON object, published as is on `/plc/json/<topic>` (Foxglove bridge) |
| `RECORD_SCHEMA` | 10 | PLC -> monitor | a record's field names and types, once per connection (`plc_records.py`) |
| `RECORD_VALUES` | 11 | PLC -> monitor | that record's values only; the bridge publishes them as JSON on `/plc/json/<topic>` |

### 2.2 Payload packing

The payload is a flat byte image with no alignment, built from a mapping:
a list of `(type, "component/field[index]")` entries (`io_layout.py`).

- **BIT:** one bit. Consecutive bits share a byte, bit 0 first. A bit run is
  padded to a whole byte.
- **BYTE / UINT16 / UINT32 / UINT64 / FLOAT:** 1 / 2 / 4 / 8 / 4 bytes, in
  the exchange's byte order (`mqttSettings.endianness`, `LE`). On the PLC
  these are BYTE / UINT / UDINT / LINT / REAL.
- **Order:**
  1. every single bit of every component first;
  2. then each multi-bit component, such as a laser banner's beams, in
     bytes of its own;
  3. then every other value, component by component.

  A device's bits are therefore all near the start of the frame and its
  values further on. The generated serializers and `plc_mock.py` handle
  this for you.

For a graph of both frames (byte and bit map per component, with the
mapping checks), run `python tools/frame_map.py <layout>`. To see any
layout's exact byte map as text:

```bash
python io_layout.py logistics_full.json          # byte.bit / type / field, both frames
python io_layout.py logistics_full.json --st     # + plain STRUCTs
```

```
input  (simulator -> PLC): 165 bytes, layout 083EBF55
  byte.bit   type    PLC    field
  0.0        bit     BOOL   start/sensor_states[0]
  0.4        bit     BOOL   s_high/sensor_states[0]
  0.7-1.0    bit     BOOL   st_feed/{isUp, occupied}
  1.3-2.2    bit     BOOL   robot_1/{busy, holding, partAtPick, placeReady, ready, atHome, inPosition, fault}
  8.0-9.7    bit     BOOL   banner_1/sensor_states[0..15]  (16 bits)
  53         uint32  DWORD  dws_1/code
  157        uint64  LINT   clock/nanoseconds
output (PLC -> simulator): 247 bytes, layout A188665C
  0.0        bit     BOOL   lamp_run/isOn
  0.2        bit     BOOL   st_feed/release
  0.4-1.0    bit     BOOL   robot_1/{start, enable, hold, faultReset, gripperClose}
```

### 2.3 Timing

- **Simulator -> PLC:** the simulator sends the input frame to every
  connected client every 10 ms (`interval` of the input channel).
- **PLC -> simulator:** the PLC sends its output frame once per scan. The
  simulator applies each frame as it arrives.
- **Stale frames:** both sides keep only the newest frame of each kind, so
  a slow peer gets current data, not a backlog.
- **Clock:** the input frame ends with `clock`, the simulator's time in
  nanoseconds (LINT). A PLC can use it to time-stamp, or to set its own
  clock.

### 2.4 Layout check

After connecting, the PLC sends a `LAYOUT_CHECK` frame with a 13-byte
payload:

| Offset | Type | Field |
|---|---|---|
| 0 | UDINT | input frame fingerprint |
| 4 | UINT | input frame size |
| 6 | UDINT | output frame fingerprint |
| 10 | UINT | output frame size |
| 12 | BYTE | status: 0 request, 1 match, 2 mismatch |

The simulator compares the payload with its own layout and answers with a
`LAYOUT_CHECK` frame of its own values, status 1 or 2. It also logs
`I/O layout check with node 0: OK`. A fingerprint is a CRC-32 of the frame's
offsets, types and names. The PLC does not compute it: it sends the
constants generated for it (`SimIoContract.SIM_INPUT_FINGERPRINT`, ...).

A mismatch means the two sides were built from different layout files.
Regenerate the PLC side from the layout the simulator runs.

---

## 3. Defining the exchange

### 3.1 What a layout puts in the frames

`io_topics(config)` lists the components of each frame, in this order:

| Frame | Components, in order |
|---|---|
| input (sim -> PLC) | every button, every sensor, every device's feedback (kind by kind, see 3.3), `clock` |
| output (PLC -> sim) | every lamp (LED), every belt / roller conveyor, every device's commands |

Each component's fields depend on its kind:

| Component | Input (feedback) | Output (commands) |
|---|---|---|
| IR sensor, push button | its output BOOL (`sensor_states`) | - |
| Laser banner / light curtain | one BOOL per beam (`ARRAY[0..n-1] OF BOOL`) | - |
| Sensor / button with `packFormat` byte/uint16/uint32/float | one register (BYTE / UINT / UDINT / REAL) | - |
| Lamp (LED) | - | `isOn` BOOL (`packFormat` byte: `mode` BYTE - 0 off, 1 on, 2/3 blink) |
| Belt / roller conveyor, `commandFormat` "speed" | - | `speed` REAL (m/s, signed), `directionAngle` REAL |
| Belt / roller conveyor, `commandFormat` "vfd" | - | `run` BOOL, `reverse` BOOL, `speed` REAL |
| Belt / roller conveyor, `commandFormat` "frequency" | - | `run` BOOL, `reverse` BOOL, `frequency` REAL (Hz to the inverter) |
| Belt / roller conveyor with `feedback` on (the belt's own property) | `running`, `atSpeed`, `reversing`, `fault` BOOL; `actualSpeed` REAL, `encoderCount` UDINT; a "frequency" belt also `outputFrequency` REAL (Hz), `motorSpeed` REAL (rpm) | - |
| Machines and logistics devices | see 3.3 | see 3.3 |

A belt commanded in Hz is driven through its drive train (`drivetrain.py`):
the PLC converts the surface speed it wants into the inverter frequency with
the drive's nameplate data, as on a real line. With the train in the layout's
`drive` settings:

```
// m/s per Hz: motor rpm per Hz, through the gearbox and the chain, at the drum
speedPerHz := motorRatedSpeed / motorRatedFrequency / (gearRatio * transmissionRatio) / 60.0 * 3.14159265 * drumDiameter;
cmd.run := running;
cmd.reverse := speed < 0.0;
cmd.frequency := ABS(speed) / speedPerHz;     // the inverter caps it at maxFrequency
```

A "frequency" belt without `gearRatio` and `drumDiameter` runs its design
speed (`speed` in the layout, else 1 m/s) at the rated frequency, so then
`cmd.frequency := ABS(speed) / designSpeed * motorRatedFrequency`. The
feedback's `outputFrequency` and `motorSpeed` are what the inverter reports.

Ids are the component ids of the layout (`div_1`, `infeed`, ...). In the
generated ST each component is a STRUCT member with its id as the name, so
choose ids that are valid identifiers.

### 3.2 Exchange settings

These are in the layout's `mqttSettings`, or a separate file passed with
`--exchange-config` to `main.py`, `plc_mock.py` and `st_export.py`:

```json
{ "transport": "tcp", "enabled": true,
  "tcpBindHost": "127.0.0.1", "tcpServerHost": "127.0.0.1", "tcpPort": 5001,
  "endianness": "LE" }
```

The simulator listens on `tcpBindHost:tcpPort`. A PLC on the network needs
`tcpBindHost` set to `0.0.0.0` (or the PC's address), and the PC's IP and
port configured in the PLC.

### 3.3 Machines and logistics devices: `device_kinds.py`

Every kind of device on the exchange is declared once in
`device_kinds.py`. The frames, the component lookup, the simulator
inspector's PLC I/O, the mock PLC and the generated ST STRUCTs all read from
there. The current kinds, in frame order:

| Kind (layout list) | PLC STRUCTs | Commands (PLC -> simulator) | Feedback (simulator -> PLC) |
|---|---|---|---|
| Pop-up stopper (`stoppers`) | `Stopper_Command`, `Stopper_Feedback` | `release` BOOL | `isUp` BOOL, `occupied` BOOL |
| 6-axis robot, FANUC UOP style (`robots`) | `Robot_Command`, `Robot_Feedback` | `start`, `enable`, `hold`, `faultReset`, `gripperClose` BOOL; `program`, `speedOverride` BYTE; `targetX`, `targetY`, `targetZ`, `targetYaw` REAL | `busy`, `holding`, `partAtPick`, `placeReady`, `ready`, `atHome`, `inPosition`, `fault` BOOL; `cycles` UINT; `faultCode` BYTE; `j1`..`j6`, `tcpX`, `tcpY`, `tcpZ` REAL |
| Elevator (`elevators`) | `Elevator_Command`, `Elevator_Feedback` | `targetLevel` BYTE | `moving`, `atLevel`, `occupied` BOOL; `level` BYTE |
| Barcode / RFID / DWS scanner (`scanners`) | `Scanner_Feedback` | - | `noRead`, `present` BOOL; `code` UDINT; `destination` BYTE; `readCount` UINT; `weight`, `measuredLength`, `measuredWidth`, `measuredHeight` REAL |
| Pusher / wheel diverter (`diverters`) | `Diverter_Command`, `Diverter_Feedback` | `divert` BOOL; `commandSpeed` REAL (m/s), `commandAngle` REAL (deg added to `divertAngle`, 0 = as designed) | `active`, `present` BOOL; `presentCode` UDINT; `diverted` UINT |
| Turntable (`turntables`) | `Turntable_Command`, `Turntable_Feedback` | `targetIndex` BYTE | `turning`, `inPosition`, `occupied` BOOL; `index` BYTE |
| 90-degree transfer (`transfers`) | `Transfer_Command`, `Transfer_Feedback` | `transfer` BOOL | `present`, `centered`, `up` BOOL; `transferred` UINT |
| Accumulation conveyor (`accumulators`) | `Accumulator_Command`, `Accumulator_Feedback` | `release` BOOL | `full` BOOL; `zones` UINT (bit i = zone i occupied) |
| Guard door (`doors`) | `GuardDoor_Command`, `GuardDoor_Feedback` | `lock` BOOL | `closed`, `locked` BOOL |
| Safety relay (`safety`) | `SafetyRelay_Command`, `SafetyRelay_Feedback` | `reset` BOOL (rising edge) | `ok`, `tripped` BOOL |
| Chute: a sink with a capacity (`sinks`) | `Chute_Command`, `Chute_Feedback` | `empty` BOOL (rising edge) | `full` BOOL; `count` UINT |

Counters such as `readCount`, `diverted`, `transferred`, `cycles` and
`count` wrap at 65535. React to a change, not to a value.

**Command source.** Each device that takes commands has a `control`
setting: `plc` (the default: the PLC's frames set the commands) or `manual`
(the operator sets them in the simulator window's inspector, and the PLC's
frames are ignored for it). The frame layout is the same either way.

### 3.4 Adding a kind of device

The steps are also at the top of `device_kinds.py`:

1. **The model:** add it to `config_parser.py`, with a list for it in
   `SimulatorConfig` and a `control: ControlMode = "plc"` field if it takes
   commands.
2. **The registration:** append one `register(DeviceKind(...))` to
   `device_kinds.py`, with its list name, model, STRUCT name, commands and
   feedback. Append it at the end: the order is the frames' order, so
   existing frames keep their bytes.
3. **The simulation:** what it does (`machines.py` / `logistics.py`): read
   its command fields, write its feedback fields.
4. **The mock PLC's logic:** a `@device_program("<list>")` function in
   `plc_logic.py` (section 4.4).
5. **Its look:** the designer item and catalogue entry, and the 2D / 3D
   drawing.
6. **The PLC side:** regenerate it (`python st_export.py ...`). The new
   kind gets its `<Name>_Command` / `<Name>_Feedback` STRUCTs, and the
   frame STRUCTs a member per device, without changes to the generator.

---

## 4. The mock PLC (`plc_mock.py`, Python)

### 4.1 How it extracts and composes

The mock PLC uses a copy of the layout (`SimulatorConfig`) as its process
image. Decoding a frame writes the received values into that copy's
components; encoding reads the values to send from it.

```python
from config_parser import load_config
from mqtt_data_exchange import io_topics
from mqtt_message_binding import ComponentMessage, custom_component_mapping

config = load_config("logistics_full.json")          # the process image
pub, sub = io_topics(config)
input_msg = ComponentMessage(custom_component_mapping(config, pub["input"]["custom_ids"], direction="out"),
                             endianness="<")
output_msg = ComponentMessage(custom_component_mapping(config, sub["output"]["custom_ids"], direction="in"),
                              endianness="<")
print(input_msg.payload_size, output_msg.payload_size)   # 165 247
```

- **Extract:** `input_msg.deserialize(payload, config)` writes every input
  field into the components:
  - `config.diverters[0].presentCode`
  - `config.robots[0].ready`
  - `config.sensors[...].sensor_states` (a bool or bit-string; for a
    banner, beam i is character i)
- **Compose:** set the command fields on the components, then
  `payload = output_msg.serialize(config)`:
  - `belt.speed = 1.0`
  - `diverter.enable = True`
  - `stopper.release = True`
  - `led.isOn = True`
- **By id:** `mqtt_message_binding.find_component_by_id("div_1", config)`
  returns that component.
- **Reading a sensor:** `plc_logic.occupied(sensor)` gives "something in
  front of it" (it handles light-on / NC sensors).
  `plc_logic.part_at_pick(config, robot)` reads a robot's pick sensor.

### 4.2 A minimal PLC client

`plc_mock.py` adds the PLC program, metrics, the layout check and a
monitoring bridge. This is the core of it, which you can use as a starting
point for your own tests:

```python
import threading, time
from config_parser import load_config
from mqtt_data_exchange import io_topics
from mqtt_message_binding import ComponentMessage, custom_component_mapping, find_component_by_id
from tcp_framed_transport import FrameType, TcpFramedClient

config = load_config("logistics_full.json")
pub, sub = io_topics(config)
rx = ComponentMessage(custom_component_mapping(config, pub["input"]["custom_ids"], direction="out"), "<")
tx = ComponentMessage(custom_component_mapping(config, sub["output"]["custom_ids"], direction="in"), "<")
lock = threading.Lock()

def on_frame(frame):                                   # the TCP thread
    if frame.frame_type == FrameType.IO_MAPPING and len(frame.payload) == rx.payload_size:
        with lock:
            rx.deserialize(frame.payload, config)      # extract

link = TcpFramedClient("127.0.0.1", 5001, on_frame, length_endianness="LE")
link.start()
infeed, div_1 = find_component_by_id("infeed", config), find_component_by_id("div_1", config)
while True:                                            # the scan
    with lock:
        infeed.speed = 1.0                             # compose
        div_1.commandSpeed, div_1.commandAngle = div_1.speed, 0.0   # 0: its divertAngle, as designed
        div_1.enable = div_1.presentCode != 0          # divert everything, for the demo
        payload = tx.serialize(config)
    link.send(payload, frame_type=FrameType.IO_MAPPING, node_id=0)
    time.sleep(0.05)
```

### 4.3 The mock PLC's program

`plc_mock.py` runs `plc_logic.make_controller(config)`: `DeliveryController`
for one line, or `LineController` for a layout with several lines. Each
scan it does three things:

1. **Buttons:** reads them, START / STOP / RESET by label.
2. **Belts:** runs them at their layout speed while the line runs; a sensor
   blocked too long is a jam (a robot's pick sensor is exempt).
3. **Devices:** calls `MachineControl.step`, which runs the device program
   of each `plc`-controlled device in registry order (scanners before
   diverters: the reads feed the tracking).

### 4.4 Adding or changing a device's PLC logic

Each kind has one function in `plc_logic.py`:

```python
@device_program("diverters")
def _diverter(control, d, scan: Scan) -> None:
    d.commandSpeed, d.commandAngle = d.speed, d.divertAngle   # commandAngle: deg to the travel
    d.commandSpeed, d.commandAngle = d.speed, 0.0     # commandAngle: a correction to divertAngle
    fire = scan.running and d.presentCode != 0 and control.track.get(d.presentCode) == d.destination
    d.enable = fire
```

- **Its inputs:** `scan` gives `running`, `dt` (s since the last scan),
  `speeds`, `events` (log lines) and `reset` (Start / Reset pressed this
  scan).
- **Memory between scans:** `control.state(device)` is a dict kept per
  device.
- **Other shared state:** `control.track` (code -> destination, from the
  scanners), `control.config`, `control.belts`.

### 4.5 Running and checking

```bash
python main.py logistics_full.json                    # the simulator
python plc_mock.py logistics_full.json --autostart    # the PLC, Start pressed at once
python tools/verify_exchange.py logistics_full.json --seconds 60
```

`plc_mock.py` prints its input and output frames and
`[PLC] I/O layout check: OK`, then `[METRICS] rx ... tx ... malformed 0`.

`verify_exchange.py` runs both in their own processes and checks:

- the handshake;
- frames both ways with no errors;
- that the plant follows: belts run, chutes fill, scanners read, doors
  lock, the safety relay stays OK.

---

## 5. A real PLC: Siemens SIMATIC AX (ST)

### 5.1 Generate the types and serializers

```bash
python st_export.py logistics_full.json --out <AX>/src/generated --tests-out <AX>/test/generated --contract <AX>/contract/io_contract.json
python st_export.py logistics_full.json --out <AX>/src/generated --tests-out <AX>/test/generated --check   # CI: exit 1 if stale
```

| File | What it gives the PLC program |
|---|---|
| `SimIoTypes.st` | `NAMESPACE Simulator.Io`: a STRUCT per device kind and direction (`Diverter_Command`, `Diverter_Feedback`, `BeltDrive_Command`, ...); `Sim_Input` / `Sim_Output` with one member per component, named by its id |
| `SimIoSerde.st` | `DeserializeFromBytes(startOffset, source, value)` / `SerializeToBytes(startOffset, value, dest)` overloads for each frame STRUCT, written from the Python packer's byte and bit offsets |
| `SimIoContract.st` | `CLASS SimIoContract`: `FRAME_MAGIC`, `HEADER_SIZE`, `FRAME_TYPE_*`, `SIM_INPUT_SIZE` / `_FINGERPRINT`, `SIM_OUTPUT_SIZE` / `_FINGERPRINT`, `CONTRACT_FINGERPRINT` |
| `SimIoVectorsTest.st` | AxUnit tests: sample values serialize to exactly the Python bytes, and back |
| `io_contract.json` | the same contract as data (each field's byte, bit, type, member) |

The generated frame STRUCTs for `logistics_full.json` (excerpt):

```
Diverter_Command : STRUCT
    enable : BOOL;
    commandSpeed : REAL;
    commandAngle : REAL;
END_STRUCT;
Sim_Input : STRUCT                       // 165 bytes
    start : BOOL;                        // buttons, sensors: BOOL
    s_pick : BOOL;
    st_feed : Stopper_Feedback;          // devices: their kind's STRUCT
    robot_1 : Robot_Feedback;
    dws_1 : Scanner_Feedback;
    div_1 : Diverter_Feedback;
    chute_1 : Chute_Feedback;
    banner_1 : ARRAY[0..15] OF BOOL;     // laser banner: one BOOL per beam
    clock : LINT;                        // simulator time, ns
END_STRUCT;
Sim_Output : STRUCT                      // 247 bytes
    lamp_run : BOOL;
    st_feed : Stopper_Command;
    robot_1 : Robot_Command;
    div_1 : Diverter_Command;
    infeed : BeltDrive_Command;          // belts: speed, directionAngle
    tt_1 : Turntable_Command;
END_STRUCT;
```

Struct member order does not matter for the bytes: the serializers write
each field at its offset.

### 5.2 Receive and extract

The AX project's `TcpClient`, `TCP_StreamParser` and `MagicFrameCodec`
(`src/communication/`) already do the TCP connection and the framing. With
the generated code, the receive side of a cyclic program becomes:

```
USING Simulator.Io;
USING Ax.Communication;

    VAR
        simIn  : Sim_Input;
        simOut : Sim_Output;
        contract : SimIoContract;
        inputValid : BOOL;
    END_VAR

    tcpClient.Run();
    tcp_praser.Execute(tcpClient.ReceiveBuffer, tcpClient.ReceiveLength);

    // Only the I/O frame of the right size goes into Sim_Input: the simulator
    // also sends LAYOUT_CHECK replies.
    IF tcp_praser.PayloadReady
       AND magicCodec.ReceivedFrameType = contract.FRAME_TYPE_IO_MAPPING
       AND TO_UDINT(tcp_praser.ExtractedLength) = contract.SIM_INPUT_SIZE THEN
        DeserializeFromBytes(startOffset := UDINT#0, source := tcp_praser.ExtractedPayload, value := simIn);
        inputValid := TRUE;
    END_IF;

    // Extract: fields by component id
    //   simIn.s_pick                      the robot's pick sensor
    //   simIn.div_1.presentCode           the barcode in a diverter's zone
    //   simIn.robot_1.ready               the robot can take a start
    //   simIn.banner_1[3]                 beam 3 of a laser banner
    //   simIn.clock                       simulator time, ns (LINT)
```

### 5.3 Compose and send

```
    // Compose: the commands, by component id
    simOut.lamp_run := running;
    simOut.infeed.speed := REAL#1.0;                 // belt speed, m/s (negative: reverse)
    simOut.infeed.directionAngle := REAL#0.0;        // the belt's flow angle - keep the layout's value
    simOut.st_feed.release := releaseFeed;
    simOut.robot_1.enable := TRUE;
    simOut.robot_1.program := BYTE#1;                // 1 pick and place
    simOut.robot_1.start := simIn.s_pick AND simIn.robot_1.ready AND simIn.robot_1.placeReady;
    simOut.div_1.enable := fireDiverter1;
    simOut.div_1.commandSpeed := REAL#1.2;           // m/s
    simOut.div_1.commandAngle := REAL#0.0;           // deg added to its divertAngle: 0 = as designed

    outLen := SerializeToBytes(startOffset := UDINT#0, value := simOut, dest := outBuffer);
    magicCodec.SendFrameType := contract.FRAME_TYPE_IO_MAPPING;
    magicCodec.Pack(payload := outBuffer, payloadLength := outLen);
    tcpClient.QueueSend(dataLength := magicCodec.PackedLength, data := magicCodec.PackedBuffer);
```

Send the output frame every scan, even when nothing changed: it is the PLC's
current command image.

**Buffer sizes.** Every buffer must hold the largest frame plus 14 bytes of
framing. For `logistics_full.json` that is 165 + 14 bytes in and
247 + 14 bytes out. In the AX project these are 1024 bytes:

- `MagicFrameCodec.MAX_PACKED_LENGTH`;
- `TcpClient.MAX_SEND_LENGTH` and `MAX_RECEIVE_LENGTH`.

Size `outBuffer` to at least `SIM_OUTPUT_SIZE` too. A frame that does not fit
is dropped (`DroppedPackCount` rises) and the simulator sees nothing.

### 5.4 Layout check

Once per connection, send the fingerprints the generator wrote:

```
    IF tcpClient.IsConnected AND NOT layoutChecked THEN
        // Serialize takes variables: copy the constants first.
        inFingerprint := contract.SIM_INPUT_FINGERPRINT;   inSize := TO_UINT(contract.SIM_INPUT_SIZE);
        outFingerprint := contract.SIM_OUTPUT_FINGERPRINT; outSize := TO_UINT(contract.SIM_OUTPUT_SIZE);
        checkLen := Serialize(UDINT#0,  inFingerprint,  checkBuffer, Endianness#Little);
        checkLen := Serialize(UDINT#4,  inSize,         checkBuffer, Endianness#Little);
        checkLen := Serialize(UDINT#6,  outFingerprint, checkBuffer, Endianness#Little);
        checkLen := Serialize(UDINT#10, outSize,        checkBuffer, Endianness#Little);
        checkBuffer[12] := BYTE#0;                                  // request
        magicCodec.SendFrameType := contract.FRAME_TYPE_LAYOUT_CHECK;
        magicCodec.Pack(payload := checkBuffer, payloadLength := UDINT#13);
        tcpClient.QueueSend(dataLength := magicCodec.PackedLength, data := magicCodec.PackedBuffer);
        layoutChecked := TRUE;
    ELSIF NOT tcpClient.IsConnected THEN
        layoutChecked := FALSE;
    END_IF;

    // The answer: a LAYOUT_CHECK frame, byte 12 = 1 match / 2 mismatch
    IF tcp_praser.PayloadReady AND magicCodec.ReceivedFrameType = contract.FRAME_TYPE_LAYOUT_CHECK THEN
        layoutOk := tcp_praser.ExtractedPayload[12] = BYTE#1;
    END_IF;
```

Here `checkBuffer` is `ARRAY[0..12] OF BYTE`, `inFingerprint` /
`outFingerprint` are DWORD, `inSize` / `outSize` UINT, and `checkLen` UDINT.
A PLC that skips this still works.
The check only turns "the frames do not match" into a clear message on both
sides instead of silently wrong values.

### 5.5 One function block per kind of device

Because every device of a kind has the same STRUCTs, PLC logic can be
written once per kind and called per device:

```
CLASS DiverterControl
    VAR PUBLIC
        Destination : BYTE;
    END_VAR
    METHOD PUBLIC Run
        VAR_INPUT
            fb : Diverter_Feedback;
            destinationOfCode : BYTE;      // from your tracking: what the code in its zone is for
            running : BOOL;
        END_VAR
        VAR_IN_OUT
            cmd : Diverter_Command;
        END_VAR
        cmd.commandSpeed := REAL#1.2;
        cmd.commandAngle := REAL#0.0;          // a correction to the layout's divertAngle
        cmd.enable := running AND fb.present AND fb.presentCode <> UDINT#0
                      AND destinationOfCode = Destination;
    END_METHOD
END_CLASS

    // the cyclic program: one instance per diverter
    div1.Run(fb := simIn.div_1, destinationOfCode := Track(simIn.div_1.presentCode), running := running,
             cmd := simOut.div_1);
```

For the full logic of every kind (stopper hold times, robot start, elevator
and turntable cycles, transfers at corners, chute emptying, door lock,
safety reset), the reference is `plc_logic.py`'s device programs
(section 4.4). They are small, and port to ST one to one.

### 5.6 After the layout changes

1. Save the layout in the designer and use the same file for the simulator.
2. Regenerate: `python st_export.py <layout> --out ...`. `--check` in CI
   catches a stale PLC side.
3. Check the buffer sizes (section 5.3) against the new
   `SIM_INPUT_SIZE` / `SIM_OUTPUT_SIZE`.
4. Rebuild and run the vector tests: `apax build`, `apax test`.
5. Connect, then confirm `I/O layout check ... OK` in the simulator's log.

---

## 6. Troubleshooting

| Symptom | Cause, fix |
|---|---|
| Simulator log: `Ignoring ... frames: expected N bytes, got M` | The PLC's output frame is a different size: the two sides use different layouts. Regenerate the PLC side from the simulator's layout. |
| `I/O layout MISMATCH` | As above. The message names which frame differs and both sizes. |
| Nothing moves, frames flow | The PLC sends zero speeds / no commands, or the device is on `control: manual` (the simulator ignores the PLC for it). Check the inspector's PLC I/O in the simulator window. |
| Nothing arrives at the simulator, AX side | The output frame is larger than `MAX_PACKED_LENGTH` / `MAX_SEND_LENGTH` (`DroppedPackCount` rises). Raise them (section 5.3). |
| Values look shifted or random | The byte order or the frame type was not checked. Use `LE` on both sides, deserialize only `FRAME_TYPE_IO_MAPPING` frames of `SIM_INPUT_SIZE` bytes. |
| A counter "jumps back" | UINT counters wrap at 65535. Compare with the last value seen, not with a threshold. |
| A diverter fires but nothing turns | `commandSpeed` 0: it holds the parcel. Send the speed and the angle (deg to the travel) with `enable`; wheels also stop when `enable` is off. |
| `Port 5001 is in use` | Another simulator is running. Close it, or use another port on both sides. |
