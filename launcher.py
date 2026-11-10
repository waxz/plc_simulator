"""
launcher.py - one start window for the whole tool set: pick a layout, then
open it in the designer (layout_editor.py) or run it in the simulator
(main.py) - optionally with the mock PLC (plc_mock.py) and the Foxglove
bridge (plc_foxglove_bridge.py) - and see / stop what is running.

    python launcher.py

Each program runs as its own process with this Python; the launcher keeps
the recent layouts in the shared settings file (hmi_style.load_settings).
"""
import os
import subprocess
import sys
from dataclasses import dataclass, field
from typing import List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
RECENT_KEY = "recentLayouts"
MAX_RECENT = 10
BRIDGE_ADDRESS = "127.0.0.1:5002"
TCP_EXCHANGE = os.path.join(HERE, "tcp_exchange_config.json")


@dataclass
class SimulationOptions:
    layout: str
    exchange_config: Optional[str] = None   # None: the layout's own exchange settings
    headless: bool = False
    view3d: bool = False
    mock_plc: bool = False
    bridge: bool = False


@dataclass
class Launch:
    name: str                    # shown in the running list
    args: List[str]
    env: dict = field(default_factory=dict)


def designer_launch(layout: Optional[str]) -> Launch:
    args = [sys.executable, os.path.join(HERE, "layout_editor.py")] + ([os.path.abspath(layout)] if layout else [])
    return Launch(f"Designer: {os.path.basename(layout) if layout else 'new layout'}", args)


def simulation_launches(options: SimulationOptions) -> List[Launch]:
    """What 'Run Simulator' starts, in order: the bridge first (it listens),
    then the simulator, then the mock PLC (it connects to both)."""
    layout = os.path.abspath(options.layout)
    name = os.path.basename(layout)
    exchange = ["--exchange-config", os.path.abspath(options.exchange_config)] if options.exchange_config else []
    launches = []
    if options.bridge:
        launches.append(Launch(f"Foxglove bridge: {name}",
                               [sys.executable, os.path.join(HERE, "plc_foxglove_bridge.py"), layout, *exchange,
                                "--listen", BRIDGE_ADDRESS]))
    sim = [sys.executable, os.path.join(HERE, "main.py"), layout, *exchange]
    if options.headless:
        sim.append("--headless")
    if options.view3d:
        sim += ["--view3d", "open"]
    launches.append(Launch(f"Simulator: {name}", sim,
                           {"SDL_VIDEODRIVER": "dummy"} if options.headless else {}))
    if options.mock_plc:
        mock = [sys.executable, os.path.join(HERE, "plc_mock.py"), layout, *exchange, "--autostart"]
        if options.bridge:
            mock += ["--bridge", BRIDGE_ADDRESS]
        launches.append(Launch(f"Mock PLC: {name}", mock))
    return launches


def recent_layouts() -> List[str]:
    import hmi_style
    return [p for p in hmi_style.load_settings().get(RECENT_KEY, []) if os.path.exists(p)]


def remember_layout(path: str) -> None:
    import hmi_style
    path = os.path.abspath(path)
    recent = [p for p in hmi_style.load_settings().get(RECENT_KEY, []) if os.path.abspath(p) != path]
    hmi_style.save_settings(**{RECENT_KEY: [path] + recent[:MAX_RECENT - 1]})


def start(launch: Launch) -> subprocess.Popen:
    return subprocess.Popen(launch.args, cwd=HERE, env={**os.environ, **launch.env})


def _window():
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QLabel,
                                   QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QPushButton,
                                   QVBoxLayout, QWidget)

    import hmi_style as hmi

    class LauncherWindow(QMainWindow):
        """Pick a layout; Design or Simulate it; watch and stop what runs."""

        def __init__(self, starter=start):
            super().__init__()
            self.starter = starter
            self.running: List[tuple] = []          # (Launch, Popen, list item)
            self.setWindowTitle("Conveyor Simulator")
            central = QWidget()
            layout = QVBoxLayout(central)
            layout.setSpacing(10)
            title = QLabel("<b style='font-size:15pt'>Conveyor Simulator</b><br>"
                           "<span>Design a layout, then run it against a PLC.</span>")
            layout.addWidget(title)

            box = QGroupBox("Layout")
            row = QHBoxLayout(box)
            self.layout_combo = QComboBox()
            self.layout_combo.setEditable(True)
            self.layout_combo.setMinimumWidth(420)
            self.layout_combo.addItems(recent_layouts())
            row.addWidget(self.layout_combo, 1)
            browse = QPushButton("Browse...")
            browse.clicked.connect(self._browse)
            row.addWidget(browse)
            layout.addWidget(box)

            design = QGroupBox("Design")
            row = QHBoxLayout(design)
            self.design_btn = QPushButton("Open in Designer")
            self.design_btn.clicked.connect(self.open_designer)
            row.addWidget(self.design_btn)
            self.new_btn = QPushButton("New Layout")
            self.new_btn.clicked.connect(lambda: self._run([designer_launch(None)]))
            row.addWidget(self.new_btn)
            row.addStretch(1)
            layout.addWidget(design)

            simulate = QGroupBox("Simulate")
            column = QVBoxLayout(simulate)
            row = QHBoxLayout()
            row.addWidget(QLabel("PLC exchange"))
            self.exchange_combo = QComboBox()
            self.exchange_combo.addItem("From the layout", None)
            if os.path.exists(TCP_EXCHANGE):
                self.exchange_combo.addItem("TCP, port 5001 (tcp_exchange_config.json)", TCP_EXCHANGE)
            self.exchange_combo.addItem("Other file...", "browse")
            self.exchange_combo.activated.connect(self._exchange_chosen)
            row.addWidget(self.exchange_combo, 1)
            column.addLayout(row)
            row = QHBoxLayout()
            self.headless = QCheckBox("Headless (no window)")
            self.view3d = QCheckBox("3D view")
            self.mock_plc = QCheckBox("Mock PLC (runs the line)")
            self.bridge = QCheckBox(f"Foxglove bridge ({BRIDGE_ADDRESS})")
            for box_ in (self.headless, self.view3d, self.mock_plc, self.bridge):
                row.addWidget(box_)
            row.addStretch(1)
            column.addLayout(row)
            self.run_btn = QPushButton("Run Simulator")
            self.run_btn.setObjectName("runSimButton")
            self.run_btn.clicked.connect(self.run_simulation)
            column.addWidget(self.run_btn)
            layout.addWidget(simulate)

            running = QGroupBox("Running")
            column = QVBoxLayout(running)
            self.running_list = QListWidget()
            column.addWidget(self.running_list)
            row = QHBoxLayout()
            self.stop_btn = QPushButton("Stop")
            self.stop_btn.clicked.connect(self.stop_selected)
            row.addWidget(self.stop_btn)
            self.stop_all_btn = QPushButton("Stop All")
            self.stop_all_btn.clicked.connect(self.stop_all)
            row.addWidget(self.stop_all_btn)
            row.addStretch(1)
            column.addLayout(row)
            layout.addWidget(running, 1)
            self.setCentralWidget(central)
            self.resize(760, 560)
            self._poll = QTimer(self)
            self._poll.timeout.connect(self._refresh_running)
            self._poll.start(1000)

        # -- the layout ------------------------------------------------------

        def layout_path(self) -> Optional[str]:
            path = self.layout_combo.currentText().strip()
            if not path or not os.path.exists(path):
                QMessageBox.warning(self, "Layout", "Choose a layout file first (Browse...).")
                return None
            return path

        def _browse(self):
            path, _ = QFileDialog.getOpenFileName(self, "Open Layout", HERE, "Layouts (*.json)")
            if path:
                self.layout_combo.insertItem(0, path)
                self.layout_combo.setCurrentIndex(0)

        def _exchange_chosen(self, index: int):
            if self.exchange_combo.itemData(index) != "browse":
                return
            path, _ = QFileDialog.getOpenFileName(self, "Exchange Settings", HERE, "JSON (*.json)")
            if path:
                self.exchange_combo.insertItem(index, os.path.basename(path), path)
                self.exchange_combo.setCurrentIndex(index)
            else:
                self.exchange_combo.setCurrentIndex(0)

        # -- starting ---------------------------------------------------------

        def options(self, layout: str) -> SimulationOptions:
            return SimulationOptions(layout, exchange_config=self.exchange_combo.currentData(),
                                     headless=self.headless.isChecked(), view3d=self.view3d.isChecked(),
                                     mock_plc=self.mock_plc.isChecked(), bridge=self.bridge.isChecked())

        def open_designer(self):
            path = self.layout_path()
            if path:
                remember_layout(path)
                self._run([designer_launch(path)])

        def run_simulation(self):
            path = self.layout_path()
            if path:
                remember_layout(path)
                self._run(simulation_launches(self.options(path)))

        def _run(self, launches: List[Launch]):
            for launch in launches:
                try:
                    process = self.starter(launch)
                except OSError as exc:
                    QMessageBox.warning(self, "Start", f"Could not start {launch.name}:\n{exc}")
                    return
                item = QListWidgetItem(f"{launch.name}  (running)")
                self.running_list.addItem(item)
                self.running.append((launch, process, item))

        # -- running -----------------------------------------------------------

        def _refresh_running(self):
            for launch, process, item in self.running:
                code = process.poll()
                if code is not None:
                    item.setText(f"{launch.name}  (ended, exit code {code})")

        def stop_selected(self):
            for launch, process, item in self.running:
                if item.isSelected() and process.poll() is None:
                    process.terminate()
            self._refresh_running()

        def stop_all(self):
            for _launch, process, _item in self.running:
                if process.poll() is None:
                    process.terminate()
            self._refresh_running()

        def closeEvent(self, event):
            alive = [p for _l, p, _i in self.running if p.poll() is None]
            if alive and QMessageBox.question(
                    self, "Quit", f"Stop the {len(alive)} running program(s) too?") == QMessageBox.Yes:
                self.stop_all()
            super().closeEvent(event)

    hmi.apply_saved_theme()
    return LauncherWindow


def main() -> int:
    from PySide6.QtWidgets import QApplication
    import hmi_style as hmi
    app = QApplication(sys.argv)
    window_cls = _window()
    window = window_cls()
    window.setStyleSheet(hmi.qt_stylesheet())        # the designer's theme
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
