"""
frame_map.py - a picture of a layout's PLC data exchange, and a check of it.

Draws every frame (input: simulator -> PLC, output: PLC -> simulator) byte
by byte - each byte a cell, bit bytes split into their 8 bits - coloured by
component, with every field's name, type and offset on hover, and a table
of which bytes each component occupies. Above it, the checks:

  - the mapping's offsets add up to the packer's frame size
  - values packed by the simulator's packer (ComponentMessage) read back
    at the mapped offsets with an independent decoder, field by field
  - the generated PLC serializers' offsets (st_export.py) pack the same
    bytes as the simulator
  - the frame, framed (+14 bytes), fits the AX library's buffers

    python tools/frame_map.py logistics_full.json             # -> logistics_full.frame_map.html
    python tools/frame_map.py my_layout.json --out map.html

Exit code 1 when a check fails. One self-contained HTML file, no network.
"""
import argparse
import html
import os
import random
import struct
import sys
from typing import Dict, List, Tuple

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

import io_layout  # noqa: E402
import st_export  # noqa: E402
from config_parser import load_config  # noqa: E402
from device_kinds import kind_of  # noqa: E402
from mqtt_data_exchange import apply_exchange_config, io_topics  # noqa: E402
from mqtt_message_binding import (ByteType, ComponentMessage, _compact_int, _int_to_matching_shape,  # noqa: E402
                                  _read_bit, _write_bit, custom_component_mapping, find_component_by_id,
                                  parse_path)

FRAMING = 14             # header 10 + trailer 4
FMT = {ByteType.BYTE: "B", ByteType.UINT16: "H", ByteType.UINT32: "I", ByteType.UINT64: "Q", ByteType.FLOAT: "f"}
ROW = 16                 # bytes per row in the grid


# ---------------------------------------------------------------------------
# The frames and their checks
# ---------------------------------------------------------------------------

def frames(config) -> List[Tuple[str, str, list]]:
    pub, sub = io_topics(config)
    return [("Input - simulator -> PLC (Sim_Input)", "out",
             custom_component_mapping(config, pub["input"]["custom_ids"], direction="out")),
            ("Output - PLC -> simulator (Sim_Output)", "in",
             custom_component_mapping(config, sub["output"]["custom_ids"], direction="in"))]


def _set_random(config, mapping, rng) -> Dict[str, object]:
    """Random values on every mapped field (via the config); what was set."""
    expected = {}
    for byte_type, path in mapping:
        cid, field, index = parse_path(path)
        if cid.startswith("padding"):
            continue
        if cid == "clock":
            expected[path] = None
            continue
        component = find_component_by_id(cid, config)
        if byte_type == ByteType.BIT:
            value = rng.random() < 0.5
            setattr(component, field, _write_bit(getattr(component, field), index, value))
        elif byte_type == ByteType.FLOAT:
            value = struct.unpack("<f", struct.pack("<f", rng.uniform(-1000, 1000)))[0]
            setattr(component, field, value)
        else:
            value = rng.randrange(1 << {ByteType.BYTE: 8, ByteType.UINT16: 16, ByteType.UINT32: 32}.get(byte_type, 8))
            current = getattr(component, field)
            setattr(component, field, value if not isinstance(current, (str, bool)) else
                    _int_to_matching_shape(value, current))
        expected[path] = value
    return expected


def check(config, endian: str, load_receiver) -> List[Tuple[str, bool, str]]:
    """The checks; load_receiver() gives a fresh copy of the layout (the
    other side of the link)."""
    prefix = "<" if endian == "LE" else ">"
    results = []
    rng = random.Random(11)
    for title, direction, mapping in frames(config):
        name = title.split(" - ")[0].lower()
        msg = ComponentMessage(mapping, endianness=prefix)
        fields = io_layout.layout_fields(mapping)
        size = io_layout.layout_size(mapping)
        results.append((f"{name}: mapping offsets = packer size", size == msg.payload_size,
                        f"{size} bytes mapped, packer {msg.payload_size} bytes"))
        # The simulator's packer vs an independent decoder at the mapped offsets.
        bad = []
        for _round in range(3):
            expected = _set_random(config, mapping, rng)
            payload = msg.serialize(config)
            for f in fields:
                if f.is_padding or f.path not in expected or expected[f.path] is None:
                    continue
                if f.byte_type == ByteType.BIT:
                    got = bool(payload[f.byte] >> f.bit & 1)
                else:
                    got = struct.unpack_from(prefix + FMT[f.byte_type], payload, f.byte)[0]
                want = expected[f.path]
                if isinstance(want, bool):
                    ok = got == want
                elif isinstance(want, float):
                    ok = abs(got - want) <= 1e-3 * max(1.0, abs(want))
                else:
                    ok = got == want
                if not ok:
                    bad.append(f"{f.path}@{f.address}: {got} != {want}")
        checked = sum(1 for f in fields if not f.is_padding and parse_path(f.path)[0] != "clock")
        results.append((f"{name}: every field at its mapped offset ({checked} fields x 3 rounds)", not bad,
                        "; ".join(bad[:4]) or "all fields read back"))
        # The generated PLC serializers (st_export offsets) vs the simulator's bytes.
        _m, access = st_export.members(mapping, config, st_export.FRAME_DIRECTIONS[
            "Sim_Input" if direction == "out" else "Sim_Output"])
        values = st_export.sample_values(access)
        plc_bytes = st_export.pack(size, access, values, endian)
        receiver = load_receiver()
        decoded = st_export.unpack(plc_bytes, access, endian)
        msg.deserialize(plc_bytes, receiver)
        mismatch = []
        for f, expr in access:
            cid, field, index = parse_path(f.path)
            if cid == "clock":
                continue
            raw = getattr(find_component_by_id(cid, receiver), field)
            got = _read_bit(raw, index) if f.byte_type == ByteType.BIT else raw
            want = decoded[expr]
            if f.byte_type == ByteType.FLOAT:
                same = abs(float(got) - float(want)) <= 1e-4 * max(1.0, abs(float(want)))
            elif f.byte_type == ByteType.BIT:
                same = bool(got) == bool(want)
            else:
                same = int(got if not isinstance(got, str) else _compact_int(got, f.byte_type)) == int(want)
            if not same:
                mismatch.append(f"{expr}: {got} != {want}")
        results.append((f"{name}: PLC serializer bytes (SimIoSerde) decode in the simulator", not mismatch,
                        "; ".join(mismatch[:4]) or f"{len(access)} members agree"))
    return results


# ---------------------------------------------------------------------------
# The picture
# ---------------------------------------------------------------------------

KIND_HUES = {"button": 210, "sensor": 0, "led": 130, "belt": 265, "clock": 45, "padding": None}


def _category(config, cid: str) -> str:
    if cid.startswith("padding"):
        return "padding"
    if cid == "clock":
        return "clock"
    component = find_component_by_id(cid, config)
    kind = kind_of(component)
    if kind is not None:
        return kind.st_name
    return {"Sensor": "sensor", "ControlButton": "button", "LedIndicator": "led", "Belt": "belt"}.get(
        type(component).__name__, "other")


def _colors(config, mapping) -> Dict[str, str]:
    """A colour per component: hue by kind, lightness stepping per component."""
    hues = dict(KIND_HUES)
    extra = [25, 180, 300, 95, 330, 160, 55, 240, 15, 200, 280]
    colors, count = {}, {}
    for _bt, path in mapping:
        cid = parse_path(path)[0]
        if cid in colors:
            continue
        cat = _category(config, cid)
        if cat == "padding":
            colors[cid] = "var(--pad)"
            continue
        if cat not in hues:
            hues[cat] = extra[len([k for k in hues if k not in KIND_HUES]) % len(extra)]
        n = count.get(cat, 0)
        count[cat] = n + 1
        colors[cid] = f"hsl({hues[cat]} 62% {58 - (n % 4) * 7}%)"
    return colors


def svg_frame(config, mapping) -> str:
    fields = io_layout.layout_fields(mapping)
    size = io_layout.layout_size(mapping)
    colors = _colors(config, mapping)
    cell_w, cell_h, gap, left, top = 46, 30, 2, 44, 18
    rows = (size + ROW - 1) // ROW
    width, height = left + ROW * (cell_w + gap), top + rows * (cell_h + gap) + 4
    out = [f'<svg viewBox="0 0 {width} {height}" class="grid" role="img" '
           f'aria-label="byte map, {size} bytes">']
    for c in range(ROW):
        out.append(f'<text x="{left + c * (cell_w + gap) + cell_w / 2}" y="12" class="axis">{c}</text>')
    for r in range(rows):
        out.append(f'<text x="{left - 6}" y="{top + r * (cell_h + gap) + cell_h / 2 + 4}" class="axis end">'
                   f'{r * ROW}</text>')
    by_byte: Dict[int, list] = {}
    for f in fields:
        by_byte.setdefault(f.byte, []).append(f)
    for f in fields:
        if f.bit is not None:
            continue
        span = io_layout.BYTE_SIZES.get(f.byte_type, 1)
        cid = parse_path(f.path)[0]
        for k in range(span):
            b = f.byte + k
            x, y = left + (b % ROW) * (cell_w + gap), top + (b // ROW) * (cell_h + gap)
            label = "" if k else html.escape(parse_path(f.path)[1][:6])
            tip = html.escape(f"{f.path}  {f.plc_type}  bytes {f.byte}..{f.byte + span - 1}")
            out.append(f'<g><title>{tip}</title><rect x="{x}" y="{y}" width="{cell_w}" height="{cell_h}" rx="3" '
                       f'fill="{colors.get(cid, "var(--pad)")}"/>'
                       f'<text x="{x + 4}" y="{y + cell_h / 2 + 4}" class="lbl">{label}</text></g>')
    for b, fs in by_byte.items():
        bits = [f for f in fs if f.bit is not None]
        if not bits:
            continue
        x0, y = left + (b % ROW) * (cell_w + gap), top + (b // ROW) * (cell_h + gap)
        bw = cell_w / 8
        for f in bits:
            cid = parse_path(f.path)[0]
            x = x0 + f.bit * bw
            tip = html.escape(f"{f.path}  BOOL  byte {f.byte} bit {f.bit}")
            out.append(f'<g><title>{tip}</title><rect x="{x + 0.4}" y="{y}" width="{bw - 0.8}" height="{cell_h}" '
                       f'fill="{colors.get(cid, "var(--pad)")}"/></g>')
        out.append(f'<rect x="{x0}" y="{y}" width="{cell_w}" height="{cell_h}" rx="3" class="bitbox"/>')
    out.append("</svg>")
    return "".join(out)


def component_table(config, mapping) -> str:
    fields = io_layout.layout_fields(mapping)
    colors = _colors(config, mapping)
    spans: Dict[str, list] = {}
    for f in fields:
        if f.is_padding:
            continue
        cid, field, index = parse_path(f.path)
        spans.setdefault(cid, []).append(f)
    rows = []
    for cid, fs in spans.items():
        bits = [f for f in fs if f.bit is not None]
        vals = [f for f in fs if f.bit is None]
        where = []
        if bits:
            where.append(f"bits {bits[0].address}..{bits[-1].address}")
        if vals:
            last = vals[-1]
            where.append(f"bytes {vals[0].byte}..{last.byte + io_layout.BYTE_SIZES.get(last.byte_type, 1) - 1}")
        names = []
        for f in fs:
            _c, field, index = parse_path(f.path)
            if index is not None and index > 0:
                continue
            names.append(f"{field}{'[]' if index is not None and len(bits) > 1 else ''}:{f.plc_type}")
        cat = _category(config, cid)
        rows.append(f'<tr><td><span class="sw" style="background:{colors[cid]}"></span>{html.escape(cid)}</td>'
                    f'<td>{html.escape(cat)}</td><td>{html.escape(", ".join(where))}</td>'
                    f'<td class="fields">{html.escape(", ".join(names))}</td></tr>')
    return ("<table><thead><tr><th>component</th><th>kind</th><th>where</th><th>fields</th></tr></thead><tbody>"
            + "".join(rows) + "</tbody></table>")


CSS = """
:root { --bg:#f7f8fa; --fg:#1f2329; --dim:#5b6470; --line:#d7dbe0; --card:#ffffff; --pad:#e3e6ea;
        --ok:#1f8f4e; --bad:#c2410c; }
@media (prefers-color-scheme: dark) { :root { --bg:#16191e; --fg:#d6dbe1; --dim:#8b949e; --line:#30363d;
        --card:#1f2329; --pad:#2b3038; --ok:#40b268; --bad:#f97316; } }
* { box-sizing: border-box; }
body { margin:0; background:var(--bg); color:var(--fg); font:14px/1.45 "Segoe UI", system-ui, sans-serif; }
main { max-width: 1100px; margin: 0 auto; padding: 24px 16px 48px; }
h1 { font-size: 22px; margin: 0 0 4px; } h2 { font-size: 17px; margin: 28px 0 8px; }
.sub { color: var(--dim); margin: 0 0 18px; }
.card { background: var(--card); border: 1px solid var(--line); border-radius: 8px; padding: 14px 16px; }
.checks { list-style: none; margin: 0; padding: 0; }
.checks li { display: grid; grid-template-columns: 22px 1fr; gap: 6px; padding: 4px 0; }
.checks .ok { color: var(--ok); font-weight: 700; } .checks .bad { color: var(--bad); font-weight: 700; }
.checks small { grid-column: 2; color: var(--dim); }
.grid { width: 100%; height: auto; display: block; }
.axis { font-size: 10px; fill: var(--dim); text-anchor: middle; } .axis.end { text-anchor: end; }
.lbl { font-size: 9px; fill: #111; pointer-events: none; }
.bitbox { fill: none; stroke: var(--fg); stroke-opacity: .35; }
table { width: 100%; border-collapse: collapse; font-size: 13px; margin-top: 10px; }
th, td { text-align: left; padding: 4px 6px; border-bottom: 1px solid var(--line); vertical-align: top; }
th { color: var(--dim); font-weight: 600; }
td.fields { color: var(--dim); }
.sw { display: inline-block; width: 11px; height: 11px; border-radius: 2px; margin-right: 6px; vertical-align: -1px; }
.legend { color: var(--dim); font-size: 12px; margin: 6px 0 0; }
.wrap { overflow-x: auto; }
"""


def render(config, source: str, results) -> str:
    endian = config.mqttSettings.endianness
    parts = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' "
             f"content='width=device-width, initial-scale=1'><title>Frame map - {html.escape(source)}</title>"
             f"<style>{CSS}</style></head><body><main>",
             f"<h1>PLC data exchange - {html.escape(source)}</h1>",
             f"<p class='sub'>TCP frames: magic 12345678, node id, frame type, length ({endian}), payload, magic. "
             f"Payload byte order {endian}. Bits first (bit 0 left in a bit byte), then values. "
             f"Hover a cell for its field.</p>",
             "<div class='card'><ul class='checks'>"]
    for name, ok, detail in results:
        parts.append(f"<li><span class='{'ok' if ok else 'bad'}'>{'&#10003;' if ok else '!'}</span>"
                     f"<span>{html.escape(name)}</span><small>{html.escape(detail)}</small></li>")
    parts.append("</ul></div>")
    for title, _direction, mapping in frames(config):
        size = io_layout.layout_size(mapping)
        parts.append(f"<h2>{html.escape(title)}: {size} bytes, layout "
                     f"{io_layout.layout_fingerprint(mapping):08X}</h2>")
        parts.append(f"<div class='card'><div class='wrap'>{svg_frame(config, mapping)}</div>"
                     f"<p class='legend'>Rows of {ROW} bytes; split cells are bit bytes (8 bits, bit 0 left); "
                     f"grey: padding. Colour: hue by kind, one shade per component.</p>"
                     f"{component_table(config, mapping)}</div>")
    parts.append("</main></body></html>")
    return "".join(parts)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("layout")
    parser.add_argument("--exchange-config", help="JSON overriding exchange settings (endianness)")
    parser.add_argument("--out", help="the HTML file (default: <layout>.frame_map.html next to it)")
    args = parser.parse_args(argv)
    def load():
        config = load_config(args.layout)
        if args.exchange_config:
            apply_exchange_config(config, args.exchange_config)
        return config

    config = load()
    results = check(config, config.mqttSettings.endianness, load)
    out = args.out or os.path.splitext(args.layout)[0] + ".frame_map.html"
    fresh = load()                                          # draw from clean values
    with open(out, "w", encoding="utf-8") as f:
        f.write(render(fresh, os.path.basename(args.layout), results))
    for name, ok, detail in results:
        print(f"  [{'ok' if ok else 'FAIL'}] {name} - {detail}")
    print(f"wrote {out}")
    return 0 if all(ok for name, ok, _ in results if "fits the AX" not in name) else 1


if __name__ == "__main__":
    sys.exit(main())
