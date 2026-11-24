"""
test_sim_mcp.py - the simulator's query API as an MCP server (sim_mcp.py):
the MCP handshake and tool list over stdio, each tool on the API it wraps,
the generic sim_api, errors as tool errors (no simulator, a path outside
/api), and a real round trip against a 3D view server's /api.
"""
import io
import json
import unittest
from unittest import mock

import sim_mcp


def run(*messages):
    out = io.StringIO()
    sim_mcp.serve(io.StringIO("\n".join(json.dumps(m) for m in messages) + "\n"), out)
    return [json.loads(line) for line in out.getvalue().splitlines()]


def call(name, **arguments):
    answer = run({"jsonrpc": "2.0", "id": 9, "method": "tools/call", "params": {"name": name,
                                                                              "arguments": arguments}})[0]
    result = answer["result"]
    return result["isError"], result["content"][0]["text"]


class ProtocolTest(unittest.TestCase):
    def test_handshake_and_tools(self):
        init, tools = run({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                           "params": {"protocolVersion": "2025-03-26", "capabilities": {}}},
                          {"jsonrpc": "2.0", "method": "notifications/initialized"},   # no answer
                          {"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
        self.assertEqual(init["result"]["protocolVersion"], "2025-03-26")
        self.assertIn("tools", init["result"]["capabilities"])
        names = [t["name"] for t in tools["result"]["tools"]]
        self.assertEqual(names, ["sim_simulators", "sim_endpoints", "sim_state", "sim_tags", "sim_history",
                                 "sim_components", "sim_items", "sim_tracking", "sim_api"])
        self.assertTrue(all(t["inputSchema"]["type"] == "object" for t in tools["result"]["tools"]))

    def test_unknown_method_and_tool(self):
        a, b = run({"jsonrpc": "2.0", "id": 1, "method": "resources/list"},
                   {"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "nope"}})
        self.assertEqual(a["error"]["code"], -32601)
        self.assertEqual(b["error"]["code"], -32602)

    def test_tools_call_the_api(self):
        with mock.patch("sim_query.query", return_value={"ok": 1}) as q:
            self.assertEqual(call("sim_tags", match=["chute", "belt"], area="Q"), (False, '{"ok": 1}'))
            q.assert_called_with("/api/tags", None, match="chute,belt", area="Q")
            call("sim_history", match=["x"], seconds=5, port=8770)
            q.assert_called_with("/api/history", 8770, match="x", seconds=5)
            call("sim_items")
            q.assert_called_with("/api/parcels", None)
            call("sim_api", path="/api/future?kind=robot", params={"n": 3})     # generic: any endpoint
            q.assert_called_with("/api/future", None, kind="robot", n=3)

    def test_errors_are_tool_errors(self):
        error, text = call("sim_api", path="/button")
        self.assertTrue(error)
        self.assertIn("/api", text)
        with mock.patch("sim_query.query", side_effect=SystemExit("no simulator answers")):
            self.assertEqual(call("sim_state"), (True, "no simulator answers"))
        with mock.patch("sim_query.query", return_value={"error": "no state yet"}):
            self.assertTrue(call("sim_state")[0])

    def test_a_long_answer_is_cut(self):
        with mock.patch("sim_query.query", return_value={"x": "a" * (sim_mcp.MAX_TEXT + 100)}):
            error, text = call("sim_state")
        self.assertFalse(error)
        self.assertIn("narrow it with match", text)


class LiveTest(unittest.TestCase):
    def test_against_a_3d_view_servers_api(self):
        import view3d
        server = view3d.View3DServer(port=0)
        server.start()
        self.addCleanup(server.stop)
        port = int(server.url.rstrip("/").rsplit(":", 1)[1])
        error, text = call("sim_endpoints", port=port)
        self.assertFalse(error)
        self.assertIn("/api/state", json.loads(text)["endpoints"])


if __name__ == "__main__":
    unittest.main()
