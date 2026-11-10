"""
io_table.py - the designer's I/O table: every PLC signal of the layout in
one place, as a tag table in a PLC engineering tool or a virtual
commissioning tool's I/O list.

The rows come from the exchange contract (st_export.contract), the same
data the generated ST and both sides of the exchange use, so the table
always shows what is really on the wire: per signal the component, the
signal, the direction seen from the PLC, the PLC type, the frame, the byte
and bit, and the ST member it becomes. A run of indexed bits (a laser
banner's beams) is one row.
"""
import csv
import re
from typing import Callable, List, Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QFileDialog, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QPushButton,
                               QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

import control_panel
import st_export

COLUMNS = ("Component", "Signal", "Direction", "PLC type", "Frame", "Byte", "Bit", "ST member")
# The frames the PLC exchanges I/O in, and the direction seen from the PLC.
FRAMES = {"Sim_Input": "PLC input", "Sim_Output": "PLC output", "Belt_Feedback": "PLC input"}
INDEX = re.compile(r"\[(\d+)\]$")
PLC_TYPES = {"bit": "BOOL", "byte": "BYTE", "uint16": "UINT", "uint32": "UDINT", "uint64": "ULINT", "float": "REAL"}


def rows_of(config) -> tuple:
    """(rows, summary) for a layout: rows of strings in COLUMNS order, and a
    one-line summary of the frames (sizes, fingerprints)."""
    control_panel.refresh(config)
    contract = st_export.contract(config)
    rows: List[list] = []
    parts = []
    for frame in contract["frames"]:
        direction = FRAMES.get(frame["name"])
        if direction is None:
            continue                                    # metric layouts: not I/O
        parts.append(f"{frame['name']} {frame['size']} B ({frame['fingerprint']:08X})")
        runs = {}
        for field in frame["fields"]:
            component, _sep, signal = field["path"].partition("/")
            plc_type = "LINT" if component == "clock" else PLC_TYPES.get(field["type"], field["type"])
            member = field["member"]
            signal_base = INDEX.sub("", signal)
            # An array in the PLC (its ST member is indexed, e.g. a laser
            # banner's beams): one row for the whole run of bits. A single
            # bit (a button's sensor_states[0]) is a plain BOOL.
            indexed = INDEX.search(member) if field["type"] == "bit" else None
            if indexed:
                index = int(indexed.group(1))
                key = (component, signal_base)
                run = runs.get(key)
                if run is not None:                     # one more bit of the same array
                    run["last"] = index
                    run["end"] = (field["byte"], field["bit"])
                    continue
                run = runs[key] = {"first": index, "last": index,
                                   "start": (field["byte"], field["bit"]), "end": (field["byte"], field["bit"])}
                row = [component, signal_base, direction, "BOOL", frame["name"], "", "",
                       f"{frame['name']}.{INDEX.sub('', member)}"]
                run["row"] = row
                rows.append(row)
                continue
            if field["type"] == "bit":
                signal = signal_base
            rows.append([component, signal, direction, plc_type, frame["name"], str(field["byte"]),
                         "" if field["bit"] is None else str(field["bit"]), f"{frame['name']}.{member}"])
        for run in runs.values():                       # fill in the arrays' extent
            row = run["row"]
            count = run["last"] - run["first"] + 1
            row[1] = f"{row[1]}[{run['first']}..{run['last']}]"
            row[3] = f"ARRAY[0..{count - 1}] OF BOOL"
            (b0, i0), (b1, i1) = run["start"], run["end"]
            row[5] = str(b0) if b0 == b1 else f"{b0}..{b1}"
            row[6] = f"{i0}..{i1}" if b0 == b1 else ""
    return rows, "  ·  ".join(parts)


class IoTablePanel(QWidget):
    """The I/O table dock: filter, sortable columns, CSV export; a
    double-click selects the component on the canvas."""

    def __init__(self, get_config: Callable, on_select: Optional[Callable[[str], None]] = None):
        super().__init__()
        self.get_config = get_config
        self.on_select = on_select
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        top = QHBoxLayout()
        self.filter = QLineEdit()
        self.filter.setPlaceholderText("Filter by component, signal, type, frame...")
        self.filter.textChanged.connect(self._apply_filter)
        top.addWidget(self.filter, 1)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh)
        top.addWidget(self.refresh_btn)
        self.export_btn = QPushButton("Export CSV...")
        self.export_btn.clicked.connect(self._export)
        top.addWidget(self.export_btn)
        layout.addLayout(top)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSortingEnabled(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.cellDoubleClicked.connect(self._double_clicked)
        layout.addWidget(self.table, 1)
        self.rows: List[list] = []

    def refresh(self):
        config = self.get_config()
        if config is None:
            return
        try:
            self.rows, summary = rows_of(config)
        except Exception as exc:                        # an unfinished layout: say so, keep the old rows
            self.summary.setText(f"Cannot build the I/O list: {exc}")
            return
        inputs = sum(1 for r in self.rows if r[2] == "PLC input")
        self.summary.setText(f"{len(self.rows)} signals ({inputs} inputs, {len(self.rows) - inputs} outputs)  "
                             f"·  {summary}")
        self.table.setSortingEnabled(False)
        self.table.setRowCount(len(self.rows))
        for r, row in enumerate(self.rows):
            for c, text in enumerate(row):
                cell = QTableWidgetItem(text)
                if c in (5, 6) and text.isdigit():
                    cell.setData(Qt.DisplayRole, int(text))  # numeric sort
                self.table.setItem(r, c, cell)
        self.table.setSortingEnabled(True)
        self._apply_filter(self.filter.text())

    def _apply_filter(self, text: str):
        needle = text.strip().lower()
        for r in range(self.table.rowCount()):
            cells = (self.table.item(r, c) for c in range(self.table.columnCount()))
            visible = not needle or any(cell is not None and needle in cell.text().lower() for cell in cells)
            self.table.setRowHidden(r, not visible)

    def visible_rows(self) -> List[list]:
        return [[self.table.item(r, c).text() for c in range(self.table.columnCount())]
                for r in range(self.table.rowCount()) if not self.table.isRowHidden(r)]

    def _double_clicked(self, row: int, _column: int):
        cell = self.table.item(row, 0)
        if cell is not None and self.on_select is not None:
            self.on_select(cell.text())

    def export_csv(self, path: str) -> None:
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(COLUMNS)
            writer.writerows(self.visible_rows())

    def _export(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export I/O Table", "io_table.csv", "CSV (*.csv)")
        if path:
            self.export_csv(path)
