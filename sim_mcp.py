"""
sim_mcp.py - the running simulator's query API (view3d.py /api, the same as
sim_query.py) as an MCP server, so an AI assistant can ask a live
simulation what it is doing: its state, the PLC tags now and over time, the
components' signals, the moving items, the PLC's tracking. Read-only; the
standard library only; MCP over stdio (JSON-RPC 2.0, one message per line).

Generic: besides the named tools, `sim_api` reaches any GET /api endpoint
the simulator lists (`sim_endpoints`), so a new endpoint - for another kind
of plant than parcels - is usable without changing this file.

    claude mcp add simulator -- <python> <path>/sim_mcp.py      # Claude Code
    {"mcpServers": {"simulator": {"command": "<python>", "args": ["<path>/sim_mcp.py"]}}}   # other clients

Every tool takes an optional `port` (default: the first simulator answering
on 127.0.0.1 from 8765 on; `sim_simulators` lists them all).
"""
import json
import sys
import urllib.error
import urllib.parse
import urllib.request

import sim_query

SERVER = {"name": "conveyor-simulator", "version": "1.0.0"}
PROTOCOLS = ("2025-06-18", "2025-03-26", "2024-11-05")
MAX_TEXT = 60000                     # a tool's answer is cut beyond this (an agent's context)


def _port_prop():
    return {"type": "integer", "description": "The simulator's port (default: the first one answering)."}


def _match_prop():
    return {"type": "array", "items": {"type": "string"},
            "description": "Names or substrings to keep (none: all)."}


TOOLS = [
    {"name": "sim_simulators",
     "description": "List the simulators running on this computer: port, layout, sim time, playing, health.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "sim_endpoints",
     "description": "The simulator's query API: every GET /api endpoint and what it answers (use with sim_api).",
     "inputSchema": {"type": "object", "properties": {"port": _port_prop()}}},
    {"name": "sim_state",
     "description": "The simulation now: layout, sim time, play/speed, health and timing, the PLC link, "
                    "watchdogs, faults, chutes/sinks, lamps, moving items, deliveries.",
     "inputSchema": {"type": "object", "properties": {"port": _port_prop()}}},
    {"name": "sim_tags",
     "description": "PLC tags now: name, address, type, value, comment. area I = PLC inputs (feedback), "
                    "Q = PLC outputs (commands).",
     "inputSchema": {"type": "object", "properties": {"match": _match_prop(), "area": {"type": "string",
                                                                                         "enum": ["I", "Q"]},
                                                       "port": _port_prop()}}},
    {"name": "sim_history",
     "description": "PLC tags over the last seconds of simulated time: names, and samples [sim time, values...].",
     "inputSchema": {"type": "object", "properties": {"match": _match_prop(),
                                                       "seconds": {"type": "number", "default": 30},
                                                       "port": _port_prop()},
                     "required": ["match"]}},
    {"name": "sim_components",
     "description": "Components (any kind: belts, sensors, machines, devices) by id: their PLC signals now "
                    "and where their commands come from (plc / manual).",
     "inputSchema": {"type": "object", "properties": {"match": _match_prop(), "port": _port_prop()},
                     "required": ["match"]}},
    {"name": "sim_items",
     "description": "Every moving item on the plant (parcels, loads, pallets): id, position, speed, identity.",
     "inputSchema": {"type": "object", "properties": {"port": _port_prop()}}},
    {"name": "sim_tracking",
     "description": "What the PLC tracks (its tracked loads) checked against the items: errors in mm.",
     "inputSchema": {"type": "object", "properties": {"port": _port_prop()}}},
    {"name": "sim_api",
     "description": "Any GET endpoint of the simulator's query API (see sim_endpoints), with its query "
                    "parameters: the generic tool for what the named ones do not cover.",
     "inputSchema": {"type": "object",
                     "properties": {"path": {"type": "string", "description": "e.g. /api/state (must start /api)"},
                                    "params": {"type": "object", "description": "query parameters",
                                               "additionalProperties": {"type": ["string", "number", "boolean"]}},
                                    "port": _port_prop()},
                     "required": ["path"]}},
]


def _get(path: str, port=None, **params):
    if not path.startswith("/api"):
        raise ValueError("the path must start with /api (the read-only query API)")
    params = {k: (",".join(v) if isinstance(v, list) else v) for k, v in params.items() if v not in (None, "", [])}
    return sim_query.query(path, port, **params)


def simulators():
    found = []
    for p in range(sim_query.DEFAULT_PORT, sim_query.DEFAULT_PORT + sim_query.SEARCH):
        try:
            if not sim_query._get(p, "/api", timeout=0.5).get("simulator"):
                continue
            s = sim_query._get(p, "/api/state", timeout=2.0)
            found.append({"port": p, "layout": s.get("layout"), "simTime": s.get("simTime"),
                          "playing": s.get("playing"), "health": s.get("health")})
        except (OSError, ValueError, urllib.error.URLError):
            continue
    return {"simulators": found}


def call(name: str, args: dict):
    port = args.get("port")
    if name == "sim_simulators":
        return simulators()
    if name == "sim_endpoints":
        return _get("/api", port)
    if name == "sim_state":
        return _get("/api/state", port)
    if name == "sim_tags":
        return _get("/api/tags", port, match=args.get("match"), area=args.get("area"))
    if name == "sim_history":
        return _get("/api/history", port, match=args.get("match"), seconds=args.get("seconds", 30))
    if name == "sim_components":
        return _get("/api/components", port, match=args.get("match"))
    if name == "sim_items":
        return _get("/api/parcels", port)
    if name == "sim_tracking":
        return _get("/api/tracking", port)
    if name == "sim_api":
        path = str(args.get("path", ""))
        path, _sep, query = path.partition("?")
        params = dict(urllib.parse.parse_qsl(query))
        params.update({k: v for k, v in (args.get("params") or {}).items()})
        return _get(path, port, **params)
    raise KeyError(name)


def handle(msg: dict):
    """One JSON-RPC message: its response (None for a notification)."""
    method, mid = msg.get("method"), msg.get("id")
    if mid is None:                                   # a notification (notifications/initialized, ...)
        return None
    try:
        if method == "initialize":
            asked = (msg.get("params") or {}).get("protocolVersion")
            result = {"protocolVersion": asked if asked in PROTOCOLS else PROTOCOLS[0],
                      "capabilities": {"tools": {"listChanged": False}}, "serverInfo": SERVER,
                      "instructions": "A running conveyor/plant simulator, read-only. Start with sim_simulators "
                                      "or sim_state; sim_endpoints lists the whole query API."}
        elif method == "ping":
            result = {}
        elif method == "tools/list":
            result = {"tools": TOOLS}
        elif method == "tools/call":
            params = msg.get("params") or {}
            name = params.get("name")
            if name not in {t["name"] for t in TOOLS}:
                return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32602, "message": f"unknown tool {name}"}}
            try:
                data = call(name, params.get("arguments") or {})
                text, error = json.dumps(data, ensure_ascii=False), "error" in data if isinstance(data, dict) else False
            except SystemExit as e:                   # sim_query: no simulator answers
                text, error = str(e), True
            except (OSError, ValueError, urllib.error.URLError) as e:
                text, error = f"{type(e).__name__}: {e}", True
            if len(text) > MAX_TEXT:
                text = text[:MAX_TEXT] + f" ... (cut: {len(text)} characters; narrow it with match)"
            result = {"content": [{"type": "text", "text": text}], "isError": bool(error)}
        else:
            return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32601, "message": f"no method {method}"}}
    except Exception as e:  # noqa: BLE001 - a server answers, whatever happens
        return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32603, "message": f"{type(e).__name__}: {e}"}}
    return {"jsonrpc": "2.0", "id": mid, "result": result}


def serve(stdin=sys.stdin, stdout=sys.stdout) -> None:
    for line in stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except ValueError:
            stdout.write(json.dumps({"jsonrpc": "2.0", "id": None,
                                     "error": {"code": -32700, "message": "parse error"}}) + "\n")
            stdout.flush()
            continue
        batch = msg if isinstance(msg, list) else [msg]
        answers = [a for a in (handle(m) for m in batch if isinstance(m, dict)) if a is not None]
        if answers:
            stdout.write(json.dumps(answers if isinstance(msg, list) else answers[0], ensure_ascii=False) + "\n")
            stdout.flush()


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stdin.reconfigure(encoding="utf-8")
    serve()
