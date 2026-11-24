# Simulator ↔ SIMATIC AX exchange: evaluation and workflow

For how to define the exchange, and how to extract and compose the data in
`plc_mock.py` and in AX ST, see [plc-data-exchange-guide.md](plc-data-exchange-guide.md).

This note covers how the I/O exchange between the simulator (Python) and the
PLC program (SIMATIC AX, ST) is defined today, what goes wrong, what was
changed on the Python side, and the recommended workflow for keeping the two
in sync. The AX project (`ax-dev/ax_dev`) was only read, not changed. The PLC
changes it needs are listed at the end.

## 1. How a definition flows today

Today there are two sources of truth, joined by hand-copying.

**Python side:** the layout's components decide the wire layout:
- `io_topics()` picks the components for each channel.
- `paddng_component()` / `MACHINE_FIELDS` in `mqtt_message_binding.py` give
  each component's bits and registers, laid out bits first, padded, then the
  rest, with no alignment.
- `FrameType` in `tcp_framed_transport.py` holds the frame types.
- The metric layouts are dataclasses in `mqtt_data_exchange.py`.

**AX side:**
- The STRUCTs in `src/project/site_1/types.st` were typed by hand to match
  `io_layout.py --st`.
- `tools/generate_st_serde.py` parses those STRUCTs and writes
  `src/types-generated.st`.
- The frame-type constants (`magic_frame_codec.st`) and the framing code
  (`MagicFrameCodec`, `TCP_StreamParser`) are hand ports of the Python code.

**The only cross-check** is `test_io_packing.py`. It asserts hard-coded
sizes, but never reads the AX files.

**Where the two sides can drift:**

| Topic | Python | AX | Risk |
|---|---|---|---|
| Byte order | `mqttSettings.endianness`, applied to the payload and the length field | `Serialize`/`Deserialize` default (little-endian); `MagicFrameCodec.Pack` little-endian | Setting `"BE"` breaks the PLC silently |
| BOOL arrays (laser banners) | Bits in the wide section, each banner padded to whole bytes | The generator moves every BOOL, array elements included, into the leading bit block | An `ARRAY OF BOOL` banner on the PLC is misplaced |
| Metric layouts | Packed in declaration order | Reordered bits first by the generator | Any metric with a bit after a value mismatches |
| Frame types | `FrameType` enum, appended to by a script (it already holds `metri_1 = 4`) | Two constants, copied by hand | New frame types never reach the PLC |
| Fingerprint | `layout_fingerprint()`, `LAYOUT_CHECK` | Unknown to the PLC, which never sends a check | A changed layout is only noticed through a wrong-size counter |
| Versioning | none | none | A header change silently broke the PLC once (project notes) |

## 2. The existing generator (`tools/generate_st_serde.py`)

**What it does.** It takes ST STRUCT declarations and writes overloaded
`DeserializeFromBytes` / `SerializeToBytes` per STRUCT: BOOLs via
`GetBit`/`SetBit` first, then the other fields through `Serialize` /
`Deserialize` in declaration order. For today's I/O frames the result
matches the Python packer: 18 / 122 / 26 bytes.

**Problems:**
- **Fields dropped silently.** Any declaration line its regex doesn't match
  is skipped, which shifts every later offset. That includes `a, b : BOOL;`,
  `STRING[n]`, arrays with constant bounds, multi-line declarations and
  nested STRUCTs.
- **Byte order is hard-wired** to the library default; there is no option.
- **No `NAMESPACE`**, and its source line references point at the wrong
  lines.
- **No staleness check.** It is run by hand and isn't an apax script, so
  `apax build` can't notice a stale `types-generated.st`.
- **The direction is inverted.** It derives the wire layout from
  hand-written ST that was itself copied from Python. The rule "bits first"
  is implemented twice, in two repositories, and the edge cases above
  differ between them.

**Verdict:** keep it for the PLC's own internal STRUCTs, but stop using it
for the exchange frames. If you keep it, make it fail on unparsed lines and
add `--endianness`, `--namespace` and `--check`.

## 3. Recommended workflow: Python as the single source

One generator writes the PLC side of the exchange: `ax_export.py`, from
the simulator's own definition (`st_export.contract`). Three ways in, one
code path, so nothing is defined twice:

- **Designer:** File > Export to SIMATIC AX (`Ctrl+Shift+E`). It asks for
  the settings the first time and saves them with the layout (`axExport`):
  the `.st` file (relative to the layout), namespace, STRUCT prefix,
  frames, tags, bit arrays, max frame and an optional AxUnit test file.
- **Command line:** `st_export.py layout.json` writes as saved; options
  override, `--save` keeps them, `--check` exits 1 when stale.
- **AX project:** `tools/generate_st_serde.py --sim-config layout.json`
  passes its options to the simulator's `st_export.py` (it has no copy of
  the code; its struct mode for hand-written STRUCTs stays).

```bash
python st_export.py layout.json -o <AX>/src/sim_io.st --namespace Plant.Io --frames Sim_Input,Sim_Output --save
python st_export.py layout.json            # as saved
python st_export.py layout.json --check    # exit 1 if stale
```

The one `.st` file holds:

| Part | Contents |
|---|---|
| Device STRUCTs | A STRUCT per kind of device and direction, its PLC interface (`Diverter_Command`: `divert`, `commandSpeed`, `commandAngle`; `Diverter_Feedback`, `BeltDrive_Command`, ...), one per `device_kinds.py` entry, so a new kind gets its STRUCTs without touching the generator |
| Frame STRUCTs | `Sim_Input`, `Sim_Output`, the `Metric_Layout_*`s (or the chosen frames; `--prefix Station_1` names them `Station_1_*`), with a member per component named after it (an ST identifier, never a keyword, unique ignoring case): a device or belt of its kind's STRUCT, a sensor / button / lamp a BOOL, a laser banner an `ARRAY OF BYTE` (beam i = bit i MOD 8 of byte i DIV 8, any number of beams) |
| Serializers | `DeserializeFromBytes` / `SerializeToBytes` per frame, from the contract's byte and bit offsets (not re-derived from member order), in the layout's byte order; each first checks that the buffer holds the frame from `startOffset` |
| PLC tags | `Sim_Tags` with a variable per tag (the designer's PLC Tags), `ReadTags` / `WriteTags` |
| `SimIoContract` | Constants: every `FrameType`, each frame's size and layout fingerprint (the CRC that `LAYOUT_CHECK` uses), the contract fingerprint |
| Test file (optional) | An AxUnit test per frame: sample values must serialize to exactly the simulator packer's bytes, and back |

A frame bigger than the AX `TcpClient`'s buffer carries (a 4096-byte
payload; ax_dev's `docs/frame-size.md`) is refused with the reason. The file carries the contract fingerprint in its
header and a do-not-edit note.

In the AX project's `apax.yml` (proposed; not applied), with
`SIM_DIR: "../../conveyor-flow-simulator/python_sim"` and `LAYOUT` set under
`variables:`, and the export settings saved in the layout:

```yaml
  gen_io:
    - python $SIM_DIR/st_export.py $SIM_DIR/$LAYOUT
  check_io:
    - python $SIM_DIR/st_export.py $SIM_DIR/$LAYOUT --check
```

Then run `apax check_io && apax build && apax test` before every load.
On the Python side, `test_st_export.py` checks that the packer reproduces
`ComponentMessage`'s bytes for live values of every frame, across several
layouts and both byte orders. A CI job can also run `--check` against the
AX checkout, so either repository fails when the two drift.

Migration steps:
1. Run `apax gen_io`.
2. In `tcp_client.st`, use the generated `Sim_Input` / `Sim_Output` (or
   `--prefix Station_1`: `Station_1_Input` / `Station_1_Output`) and their
   overloads instead of the hand-written STRUCTs. The field names are the
   component ids, as today; a laser banner is an `ARRAY OF BYTE` (a
   32-beam one: `ARRAY[0..3] OF BYTE`, the same bytes as the hand-written
   DWORD; `bitArrays`: `ARRAY[0..31] OF BOOL`).
3. Replace the hand-copied frame-type constants with
   `SimIoContract.FRAME_TYPE_*`.
4. Delete the I/O STRUCTs from `types.st` and their part of
   `types-generated.st`.

> The generated ST hasn't been compiled yet. `apax build` stopped with an
> expired SIMATIC AX license token (valid until 30 Aug 2026). Run
> `apax login` / renew the license, then `apax build` and `apax test` with
> the generated files to confirm them on the real compiler.

**Checking the interface by hand.** The designer's **PLC I/O** sections
list each component's signals, named and typed as in the generated STRUCTs
(`io_signals.py` reads the same mapping). The simulator window's inspector
shows the same signals with their live values. While you bring up one PLC
function block, set the other components' **Command source** to `manual`
there: the simulator then ignores their part of the PLC frames, and you set
their commands in the inspector. For example, run a belt, fire a pusher,
steer a wheel diverter, or force a sensor, and watch the PLC's reaction in
the feedback rows.

Note: diverters now take `commandSpeed` (m/s) and `commandAngle` (deg)
REALs after `divert`: the speed to divert at, and a correction added to the
layout's `divertAngle` (0 = as designed; `plc_mock.py` sends the layout's
`speed` and 0). Layouts with diverters get a longer input frame, so
regenerate the AX side (`apax gen_io`). A robot's pick trigger is an IR
sensor at its pick point (`pickSensorId`), an ordinary sensor in the input
frame: the PLC starts the robot when it is blocked.

## 4. TCP exchange: what changed in Python (compatible with the current PLC)

- **Replies go to the asking client only.** `TcpFramedServer.send_to(conn)`
  and `ParsedFrame.conn` were added. The `LAYOUT_CHECK` reply used to be
  broadcast. The PLC decodes every frame it receives as `Station_1_Input`
  without checking the frame type, so another client's layout check
  overwrote its inputs and set its clock from garbage. The same fix was made
  in `plc_foxglove_bridge.py`.
- **Parser:**
  - zero-length frames are valid (they used to wipe the buffer);
  - a bad length skips one byte and resynchronizes instead of flushing good
    frames behind it;
  - `bytearray.find` replaces per-byte loops;
  - reads are 4 KB;
  - `SkippedBytes` counts resyncs.
- **Sockets:** `TCP_NODELAY` (no Nagle delay on 10 ms frames) and
  `SO_KEEPALIVE` on both ends.
- **Supervision:**
  - `tcpRxTimeoutMs` closes a client that has gone silent;
  - `client_info()` lists every client's node id, receive age and counters;
  - a startup warning appears when a frame exceeds the PLC's 4096-byte
    payload limit;
  - `tcpMaxPayload` drops (and counts) frames over a limit.
- **Output ownership:** `tcpOutputOwner` accepts outputs from one node id
  only, and `tcpRequireLayoutCheck` only from a connection whose layout
  check matched. Both are opt-in, so today's behaviour is unchanged.
- **Length byte order:** `tcpLengthEndianness` can now differ from the
  payload's. The default is unchanged.

## 5. What the PLC side needs (not done: the AX project was not modified)

1. In `CommunicationProgram` (`tcp_client.st`), deserialize only when
   `ReceivedFrameType = FRAME_TYPE_IO_MAPPING` and
   `ExtractedLength = SIM_INPUT_SIZE`.
2. `TcpClient.Run` should clear `IsConnected` and reconnect on receive/send
   errors or an rx timeout. Today the PLC never reconnects after the
   simulator restarts.
3. Process every frame per scan, not only the last one.
4. Range-check the length as a DINT before `TO_INT` in `TryExtract`.
5. Send `LAYOUT_CHECK` on connect using `SimIoContract` fingerprints, and
   refuse to drive outputs on a mismatch.
6. Raise the 256-byte buffers. Done: the AX library takes 4096-byte
   payloads now (frames of 4110; ax_dev's `docs/frame-size.md`).

Next, versioned (both sides): a protocol v2 on a new magic, carrying a
version byte, flags, a sequence number, the layout fingerprint and a CRC-32.
Add a HELLO frame exchanging every fingerprint. The PLC side would be a
`MagicFrameCodecV2 IMPLEMENTS IFrameCodec`; the client and parser stay as
they are.
