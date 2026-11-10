"""
io_table.py - the designer's PLC tag table: every PLC signal of the layout
as a tag, laid out like the PLC tags editor of TIA Portal - tag tables in a
tree on the left ("Show all tags", "Default tag table", your own), and per
tag its Name, Tag table, Data type, Address and Comment.

The rows come from the exchange contract (st_export.contract), the same
data the generated ST and both sides of the exchange use, so the table
always shows what is really on the wire. The address is the signal's place
in its frame in TIA notation: the frames the PLC reads are inputs (%I, one
after the other from byte 0), the frame it writes is outputs (%Q) - %I0.3,
%IB4, %QW6, %QD8. A run of indexed bits (a laser banner's beams) is one
Array of Bool tag.

Name, tag table and comment are yours: edited in the table, kept in the
layout (SimulatorConfig.plcTags, by signal - "<component>/<signal>"), undoable.
Unedited tags have a generated name (the component id, plus the signal when
the component has more than one) in the "Default tag table".

Export: CSV of the rows shown, or an .xlsx laid out as TIA Portal's PLC tag
export ("PLC Tags" sheet: Name, Path, Data Type, Logical Address, Comment,
Hmi Visible / Accessible / Writeable) for Import in a PLC tag table. TIA
PLC tags are elementary, so an array becomes one Bool tag per bit
(<name>_<i>); a 64-bit signal (the clock) is left out.
"""
import csv
import re
import zipfile
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple
from xml.sax.saxutils import escape

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QIcon, QPainter, QPixmap, QUndoCommand
from PySide6.QtWidgets import (QAbstractItemView, QFileDialog, QHBoxLayout, QHeaderView, QInputDialog, QLabel,
                               QLineEdit, QMenu, QPushButton, QSplitter, QTableWidget, QTableWidgetItem,
                               QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

import control_panel
import st_export
from config_parser import PlcTag

COLUMNS = ("Name", "Tag table", "Data type", "Address", "Comment", "Component", "Signal", "ST member")
NAME, TABLE, DATA_TYPE, ADDRESS, COMMENT, COMPONENT, SIGNAL, MEMBER = range(len(COLUMNS))
EDITABLE = (NAME, TABLE, COMMENT)
DEFAULT_TABLE = "Default tag table"
ALL_TAGS = "Show all tags"
# The frames the PLC exchanges I/O in, and their area seen from the PLC.
FRAMES = {"Sim_Input": "I", "Sim_Output": "Q", "Belt_Feedback": "I"}
INDEX = re.compile(r"\[(\d+)\]$")
# Contract type -> (TIA data type, size in bytes, address width letter).
TYPES = {"bit": ("Bool", 0, ""), "byte": ("Byte", 1, "B"), "uint16": ("UInt", 2, "W"), "uint32": ("UDInt", 4, "D"),
         "uint64": ("ULInt", 8, ""), "float": ("Real", 4, "D")}
MAX_NAME = 128


@dataclass
class Tag:
    key: str                # "<component>/<signal>": what an edit is kept by
    component: str
    signal: str
    area: str               # "I" (the PLC reads it) or "Q" (the PLC writes it)
    data_type: str          # TIA: Bool, Byte, UInt, UDInt, ULInt / LInt, Real, Array[0..n] of Bool
    byte: int               # in the PLC's area (frames of one area one after the other)
    bit: Optional[int]
    count: int = 1          # bits of an array
    member: str = ""        # the ST member, Sim_Input.<...>
    frame: str = ""
    name: str = ""
    table: str = DEFAULT_TABLE
    comment: str = ""

    @property
    def is_input(self) -> bool:
        return self.area == "I"

    @property
    def size(self) -> int:
        for tia, size, _w in TYPES.values():
            if tia == self.data_type:
                return size
        return 8 if self.data_type == "LInt" else 0

    @property
    def address(self) -> str:
        a = self.area
        if self.data_type == "Bool":
            return f"%{a}{self.byte}.{self.bit}"
        if self.count > 1:                          # an array of bits
            last = self.byte * 8 + self.bit + self.count - 1
            return f"%{a}{self.byte}.{self.bit}..%{a}{last // 8}.{last % 8}"
        width = {1: "B", 2: "W", 4: "D"}.get(self.size)
        if width is None:                           # 64 bits: no I/O address width in TIA
            return f"%{a}B{self.byte}..%{a}B{self.byte + self.size - 1}"
        return f"%{a}{width}{self.byte}"

    def cells(self) -> List[str]:
        return [self.name, self.table, self.data_type, self.address, self.comment, self.component, self.signal,
                self.member]


def _labels(config) -> Dict[str, str]:
    labels = {}
    for attr in ("belts", "sensors", "sources", "sinks", "buttons", "leds", "pallets", "stoppers", "robots",
                 "elevators", "scanners", "diverters", "turntables", "accumulators", "transfers", "doors",
                 "safety", "controlPanels"):
        for model in getattr(config, attr, []):
            label = getattr(model, "label", "")
            if label:
                labels.setdefault(model.id, str(label))
    return labels


def tags_of(config) -> Tuple[List[Tag], str]:
    """(tags, summary) for a layout: every PLC signal as a tag, with your
    names / tables / comments (config.plcTags) or the generated ones, and a
    one-line summary of the frames (sizes, fingerprints)."""
    control_panel.refresh(config)
    contract = st_export.contract(config)
    tags: List[Tag] = []
    parts = []
    base = {"I": 0, "Q": 0}
    for frame in contract["frames"]:
        area = FRAMES.get(frame["name"])
        if area is None:
            continue                                    # metric layouts: not I/O
        offset = base[area]
        base[area] += frame["size"]
        parts.append(f"{frame['name']} {frame['size']} B ({frame['fingerprint']:08X}) at %{area}{offset}")
        arrays: Dict[tuple, Tag] = {}
        for field in frame["fields"]:
            component, _sep, signal = field["path"].partition("/")
            tia, _size, _w = TYPES.get(field["type"], (field["type"], 0, ""))
            if component == "clock":
                tia = "LInt"
            member = field["member"]
            signal_base = INDEX.sub("", signal)
            byte, bit = offset + field["byte"], field["bit"]
            # An array in the PLC (its ST member is indexed, e.g. a laser
            # banner's beams): one tag for the whole run of bits. A single
            # bit (a button's sensor_states[0]) is a plain Bool.
            indexed = INDEX.search(member) if field["type"] == "bit" else None
            if indexed:
                key = (component, signal_base)
                tag = arrays.get(key)
                if tag is not None:
                    tag.count += 1
                    continue
                tag = arrays[key] = Tag(f"{component}/{signal_base}", component, signal_base, area, "Bool", byte,
                                        bit, member=f"{frame['name']}.{INDEX.sub('', member)}", frame=frame["name"])
                tags.append(tag)
                continue
            if field["type"] == "bit":
                signal = signal_base
            tags.append(Tag(f"{component}/{signal}", component, signal, area, tia, byte, bit,
                            member=f"{frame['name']}.{member}", frame=frame["name"]))
        for tag in arrays.values():
            tag.data_type = f"Array[0..{tag.count - 1}] of Bool"
            tag.signal = f"{tag.signal}[0..{tag.count - 1}]"
    _name(tags, config)
    return tags, "  ·  ".join(parts)


def _name(tags: List[Tag], config) -> None:
    """Your names / tables / comments, else the generated ones (unique)."""
    per_component: Dict[str, int] = {}
    for tag in tags:
        per_component[tag.component] = per_component.get(tag.component, 0) + 1
    labels = _labels(config)
    edits = {t.key: t for t in getattr(config, "plcTags", [])}
    taken = {e.name for e in edits.values() if e.name}
    for tag in tags:
        edit = edits.get(tag.key)
        tag.table = (edit.table if edit is not None and edit.table else DEFAULT_TABLE)
        default = "Simulated time (ns)" if tag.component == "clock" else labels.get(tag.component, "")
        tag.comment = edit.comment if edit is not None and edit.comment else default
        if edit is not None and edit.name:
            tag.name = edit.name
            continue
        signal = INDEX.sub("", tag.signal.split("[")[0])
        name = tag.component if per_component[tag.component] == 1 else f"{tag.component}_{signal}"
        unique, n = name, 2
        while unique in taken:
            unique, n = f"{name}_{n}", n + 1
        tag.name = unique
        taken.add(unique)


def rows_of(config) -> Tuple[List[list], str]:
    """(rows, summary): the tags as rows of strings in COLUMNS order."""
    tags, summary = tags_of(config)
    return [t.cells() for t in tags], summary


def check_name(name: str, tags: List[Tag], key: str) -> str:
    """"" when `name` can be the tag's name (TIA's rules: not empty, no
    quote, at most 128 characters, unique), else why not."""
    if not name.strip():
        return "A tag needs a name."
    if '"' in name:
        return "A tag name can't contain a double quote."
    if len(name) > MAX_NAME:
        return f"A tag name has at most {MAX_NAME} characters."
    if any(t.name == name and t.key != key for t in tags):
        return f"There is already a tag named '{name}'."
    return ""


def set_tag(config, key: str, edit: Optional[PlcTag]) -> None:
    """Your entry for one tag in the layout: `edit`, or none (generated)."""
    rest = [t for t in config.plcTags if t.key != key]
    config.plcTags = rest + ([edit] if edit is not None and (edit.name or edit.table or edit.comment) else [])


class TagEditCommand(QUndoCommand):
    """One tag's name / table / comment changed: undoable like any edit."""

    def __init__(self, config, key: str, before: Optional[PlcTag], after: Optional[PlcTag], text: str,
                 on_change: Callable[[], None]):
        super().__init__(text)
        self.config, self.key, self.on_change = config, key, on_change
        self.before = before.model_copy() if before is not None else None
        self.after = after.model_copy() if after is not None else None

    def redo(self):
        set_tag(self.config, self.key, self.after)
        self.on_change()

    def undo(self):
        set_tag(self.config, self.key, self.before)
        self.on_change()


def tia_rows(tags: List[Tag]) -> Tuple[List[list], int]:
    """(rows, skipped): the "PLC Tags" sheet of TIA Portal's tag export,
    header first - arrays as one Bool per bit, 64-bit signals skipped."""
    rows = [["Name", "Path", "Data Type", "Logical Address", "Comment", "Hmi Visible", "Hmi Accessible",
             "Hmi Writeable", "Typeobject ID", "Version ID"]]
    skipped = 0
    for tag in tags:
        writeable = "False" if tag.is_input else "True"
        if tag.count > 1:
            for i in range(tag.count):
                bit = tag.byte * 8 + tag.bit + i
                rows.append([f"{tag.name}_{i}", tag.table, "Bool", f"%{tag.area}{bit // 8}.{bit % 8}",
                             f"{tag.comment} [{i}]".strip(), "True", "True", writeable, "", ""])
        elif tag.size == 8:
            skipped += 1
        else:
            rows.append([tag.name, tag.table, tag.data_type, tag.address, tag.comment, "True", "True", writeable,
                         "", ""])
    return rows, skipped


def _column(n: int) -> str:
    letters = ""
    n += 1
    while n:
        n, r = divmod(n - 1, 26)
        letters = chr(65 + r) + letters
    return letters


def write_xlsx(path: str, sheet: str, rows: List[list]) -> None:
    """A one-sheet .xlsx of text cells - no spreadsheet library needed."""
    def row_xml(r, cells):
        return f'<row r="{r}">' + "".join(
            f'<c r="{_column(c)}{r}" t="inlineStr"><is><t xml:space="preserve">{escape(str(v))}</t></is></c>'
            for c, v in enumerate(cells)) + "</row>"
    sheet_xml = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                 '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetData>'
                 + "".join(row_xml(r, cells) for r, cells in enumerate(rows, 1)) + "</sheetData></worksheet>")
    files = {
        "[Content_Types].xml":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
            '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
            '<Default Extension="xml" ContentType="application/xml"/>'
            '<Override PartName="/xl/workbook.xml" '
            'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
            '<Override PartName="/xl/worksheets/sheet1.xml" '
            'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/></Types>',
        "_rels/.rels":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/'
            'officeDocument" Target="xl/workbook.xml"/></Relationships>',
        "xl/workbook.xml":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
            'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
            f'<sheets><sheet name="{escape(sheet)}" sheetId="1" r:id="rId1"/></sheets></workbook>',
        "xl/_rels/workbook.xml.rels":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/'
            'worksheet" Target="worksheets/sheet1.xml"/></Relationships>',
        "xl/worksheets/sheet1.xml": sheet_xml,
    }
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for name, text in files.items():
            z.writestr(name, text)


class _Cell(QTableWidgetItem):
    """Sorts by its sort key (an address by area, byte, bit) when it has one."""

    def __lt__(self, other):
        mine, theirs = self.data(Qt.UserRole + 1), other.data(Qt.UserRole + 1)
        if mine is not None and theirs is not None:
            return tuple(mine) < tuple(theirs)
        return self.text().lower() < other.text().lower()     # (super().__lt__ crashes PySide while sorting)


_ICONS: Dict[str, QIcon] = {}


def _tag_icon(area: str) -> QIcon:
    """A small tag symbol, green for an input, orange for an output."""
    icon = _ICONS.get(area)
    if icon is None:
        pixmap = QPixmap(14, 14)
        pixmap.fill(Qt.transparent)
        p = QPainter(pixmap)
        p.setRenderHint(QPainter.Antialiasing)
        colour = QColor(64, 178, 104) if area == "I" else QColor(232, 140, 40)
        p.setBrush(colour)
        p.setPen(colour.darker(140))
        p.drawRoundedRect(1, 3, 12, 8, 2, 2)
        p.setPen(QColor(255, 255, 255))
        p.drawLine(4, 7, 10, 7)
        p.end()
        icon = _ICONS[area] = QIcon(pixmap)
    return icon


class IoTablePanel(QWidget):
    """The PLC tags dock: tag tables on the left, the tags on the right -
    clicking a tag selects, zooms to and flashes its component on the
    canvas; Name, Tag table and Comment edited in place (double-click /
    F2); a right-click moves tags to another table; filter, sortable
    columns, CSV and TIA Portal (.xlsx) export."""

    def __init__(self, get_config: Callable, on_select: Optional[Callable[[str], None]] = None,
                 undo_stack=None, status: Optional[Callable[[str], None]] = None):
        super().__init__()
        self.get_config = get_config
        self.on_select = on_select
        self.undo_stack = undo_stack
        self.status = status or (lambda _m: None)
        self.tags: List[Tag] = []
        self.rows: List[list] = []
        self._filling = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        top = QHBoxLayout()
        self.filter = QLineEdit()
        self.filter.setPlaceholderText("Filter by name, address, type, comment, component...")
        self.filter.setClearButtonEnabled(True)
        self.filter.textChanged.connect(self._apply_filter)
        top.addWidget(self.filter, 1)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh)
        top.addWidget(self.refresh_btn)
        self.export_btn = QPushButton("Export CSV...")
        self.export_btn.clicked.connect(self._export)
        top.addWidget(self.export_btn)
        self.tia_btn = QPushButton("Export to TIA Portal...")
        self.tia_btn.setToolTip("An .xlsx in TIA Portal's PLC tag format: PLC tags > tag table > Import")
        self.tia_btn.clicked.connect(self._export_tia)
        top.addWidget(self.tia_btn)
        layout.addLayout(top)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)

        split = QSplitter(Qt.Horizontal)
        self.tables = QTreeWidget()
        self.tables.setHeaderLabel("PLC tags")
        self.tables.setRootIsDecorated(False)
        self.tables.currentItemChanged.connect(lambda *_a: self._apply_filter(self.filter.text()))
        split.addWidget(self.tables)
        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.verticalHeader().setVisible(True)        # TIA numbers its rows
        self.table.verticalHeader().setDefaultSectionSize(22)
        self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed
                                   | QAbstractItemView.AnyKeyPressed)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.horizontalHeader().setSortIndicator(ADDRESS, Qt.AscendingOrder)    # address order at first
        self.table.setSortingEnabled(True)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Interactive)
        header.setStretchLastSection(True)
        self.table.itemChanged.connect(self._item_changed)
        self.table.cellDoubleClicked.connect(self._double_clicked)
        # Clicking a tag (or moving to it with the arrow keys) shows its
        # component on the canvas: selected, zoomed to, flashed.
        self.table.currentCellChanged.connect(self._current_changed)
        self.table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self._context_menu)
        split.addWidget(self.table)
        split.setStretchFactor(1, 1)
        split.setSizes([150, 800])
        layout.addWidget(split, 1)

    # -- filling --------------------------------------------------------------

    def refresh(self):
        config = self.get_config()
        if config is None:
            return
        try:
            self.tags, summary = tags_of(config)
        except Exception as exc:                        # an unfinished layout: say so, keep the old rows
            self.summary.setText(f"Cannot build the tag list: {exc}")
            return
        self.rows = [t.cells() for t in self.tags]
        inputs = sum(1 for t in self.tags if t.is_input)
        self.summary.setText(f"{len(self.tags)} tags ({inputs} inputs %I, {len(self.tags) - inputs} outputs %Q)  "
                             f"·  {summary}")
        self._fill_tables()
        self._filling = True
        self.table.setSortingEnabled(False)
        self.table.setRowCount(len(self.tags))
        for r, tag in enumerate(self.tags):
            for c, text in enumerate(tag.cells()):
                cell = _Cell(text)
                cell.setData(Qt.UserRole, tag.key)
                if c == NAME:
                    cell.setIcon(_tag_icon(tag.area))
                    cell.setToolTip(f"{'Input' if tag.is_input else 'Output'} {tag.address} - {tag.member}")
                if c == ADDRESS:
                    cell.setData(Qt.UserRole + 1, [0 if tag.is_input else 1, tag.byte, tag.bit or 0])
                if c not in EDITABLE:
                    cell.setFlags(cell.flags() & ~Qt.ItemIsEditable)
                    if c in (COMPONENT, SIGNAL, MEMBER):
                        cell.setForeground(self.palette().placeholderText())
                self.table.setItem(r, c, cell)
        self.table.setSortingEnabled(True)
        self._filling = False
        if not getattr(self, "_sized", False) and self.tags:
            self.table.resizeColumnsToContents()
            self._sized = True
        self._apply_filter(self.filter.text())

    def _fill_tables(self):
        current = self.current_table()
        counts: Dict[str, int] = {}
        for tag in self.tags:
            counts[tag.table] = counts.get(tag.table, 0) + 1
        self.tables.blockSignals(True)
        self.tables.clear()
        names = [ALL_TAGS, DEFAULT_TABLE] + sorted(n for n in counts if n != DEFAULT_TABLE)
        for name in names:
            n = len(self.tags) if name == ALL_TAGS else counts.get(name, 0)
            item = QTreeWidgetItem([f"{name} [{n}]"])
            item.setData(0, Qt.UserRole, name)
            if name == ALL_TAGS:
                font = item.font(0)
                font.setBold(True)
                item.setFont(0, font)
            self.tables.addTopLevelItem(item)
            if name == (current or ALL_TAGS):
                self.tables.setCurrentItem(item)
        self.tables.blockSignals(False)

    def current_table(self) -> Optional[str]:
        item = self.tables.currentItem()
        return item.data(0, Qt.UserRole) if item is not None else None

    def table_names(self) -> List[str]:
        return [self.tables.topLevelItem(i).data(0, Qt.UserRole) for i in range(self.tables.topLevelItemCount())
                if self.tables.topLevelItem(i).data(0, Qt.UserRole) != ALL_TAGS]

    def show_table(self, name: str):
        for i in range(self.tables.topLevelItemCount()):
            item = self.tables.topLevelItem(i)
            if item.data(0, Qt.UserRole) == name:
                self.tables.setCurrentItem(item)

    def _apply_filter(self, text: str = ""):
        needle = (text if isinstance(text, str) else self.filter.text()).strip().lower()
        table = self.current_table()
        for r in range(self.table.rowCount()):
            cells = [self.table.item(r, c) for c in range(self.table.columnCount())]
            visible = not needle or any(cell is not None and needle in cell.text().lower() for cell in cells)
            if table not in (None, ALL_TAGS) and cells[TABLE] is not None:
                visible = visible and cells[TABLE].text() == table
            self.table.setRowHidden(r, not visible)

    def visible_rows(self) -> List[list]:
        return [[self.table.item(r, c).text() for c in range(self.table.columnCount())]
                for r in range(self.table.rowCount()) if not self.table.isRowHidden(r)]

    # -- editing --------------------------------------------------------------

    def tag(self, key: str) -> Optional[Tag]:
        return next((t for t in self.tags if t.key == key), None)

    def edit_tag(self, key: str, name: Optional[str] = None, table: Optional[str] = None,
                 comment: Optional[str] = None) -> bool:
        """Change one tag (None keeps a value); False with the reason in the
        status when the name can't be used."""
        config, tag = self.get_config(), self.tag(key)
        if config is None or tag is None:
            return False
        name = tag.name if name is None else name.strip()
        if name != tag.name:
            problem = check_name(name, self.tags, key)
            if problem:
                self.status(problem)
                self._refresh_soon()                    # the old name back in the cell
                return False
        before = next((t for t in config.plcTags if t.key == key), None)
        # Only what you changed is kept: an untouched name stays generated
        # (it follows the layout), an untouched comment the component's label.
        table = (tag.table if table is None else table.strip()) or DEFAULT_TABLE
        comment = tag.comment if comment is None else comment
        after = PlcTag(key=key,
                       name=name if name != tag.name or (before is not None and before.name) else "",
                       table="" if table == DEFAULT_TABLE else table,
                       comment=comment if comment != tag.comment or (before is not None and before.comment) else "")
        return self._push(key, before, after, f"{'Rename' if name != tag.name else 'Edit'} tag {tag.name}")

    def reset_name(self, key: str) -> bool:
        """Back to the generated name."""
        config = self.get_config()
        before = next((t for t in config.plcTags if t.key == key), None) if config is not None else None
        if before is None or not before.name:
            return False
        return self._push(key, before, before.model_copy(update={"name": ""}), f"Reset tag {before.name}")

    def _push(self, key: str, before: Optional[PlcTag], after: PlcTag, text: str) -> bool:
        if (before or PlcTag(key=key)) == after:
            return True
        command = TagEditCommand(self.get_config(), key, before, after, text, self._refresh_soon)
        if self.undo_stack is not None:
            self.undo_stack.push(command)
        else:
            command.redo()
        return True

    def _refresh_soon(self):
        """Rebuilt after the edit's signal has returned (not inside it)."""
        QTimer.singleShot(0, self.refresh)

    def _item_changed(self, item: QTableWidgetItem):
        if self._filling or item.column() not in EDITABLE:
            return
        key = item.data(Qt.UserRole)
        field = {NAME: "name", TABLE: "table", COMMENT: "comment"}[item.column()]
        self.edit_tag(key, **{field: item.text()})

    def selected_keys(self) -> List[str]:
        rows = sorted({i.row() for i in self.table.selectedIndexes()})
        return [self.table.item(r, NAME).data(Qt.UserRole) for r in rows if self.table.item(r, NAME) is not None]

    def move_to_table(self, keys: List[str], table: str):
        """Put tags in a tag table (a new name makes a new table), one undo step."""
        if self.undo_stack is not None and len(keys) > 1:
            self.undo_stack.beginMacro(f"Move {len(keys)} tags to {table}")
        for key in keys:
            self.edit_tag(key, table=table)
        if self.undo_stack is not None and len(keys) > 1:
            self.undo_stack.endMacro()

    def _context_menu(self, pos):
        keys = self.selected_keys()
        if not keys:
            return
        menu = QMenu(self)
        move = QMenu("Move to Tag Table", menu)
        for name in self.table_names():
            move.addAction(name, lambda n=name: self.move_to_table(keys, n))
        move.addSeparator()
        move.addAction("New Tag Table...", lambda: self._new_table(keys))
        menu.addMenu(move)
        menu.addAction("Reset to Generated Name", lambda: [self.reset_name(k) for k in keys])
        tag = self.tag(keys[0])
        if tag is not None and self.on_select is not None:
            menu.addSeparator()
            menu.addAction(f"Go to {tag.component}", lambda: self.on_select(tag.component))
        menu.exec(self.table.viewport().mapToGlobal(pos))

    def _new_table(self, keys: List[str]):
        name, ok = QInputDialog.getText(self, "New Tag Table", "Tag table name:", text="Tag table_1")
        if ok and name.strip():
            self.move_to_table(keys, name.strip())
            self.show_table(name.strip())

    def _current_changed(self, row: int, _column: int, previous_row: int, _previous_column: int):
        if self._filling or row < 0 or row == previous_row:
            return
        cell = self.table.item(row, COMPONENT)
        if cell is not None and self.on_select is not None:
            self.on_select(cell.text())

    def _double_clicked(self, row: int, column: int):
        if column in EDITABLE:
            return                                      # editing the cell
        cell = self.table.item(row, COMPONENT)
        if cell is not None and self.on_select is not None:
            self.on_select(cell.text())

    # -- export ---------------------------------------------------------------

    def export_csv(self, path: str) -> None:
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(COLUMNS)
            writer.writerows(self.visible_rows())

    def export_tia(self, path: str) -> int:
        """The tags of the current tag table (all of them on Show all tags)
        as TIA Portal's PLC tag .xlsx; returns how many rows were written."""
        table = self.current_table()
        tags = [t for t in self.tags if table in (None, ALL_TAGS) or t.table == table]
        rows, skipped = tia_rows(tags)
        write_xlsx(path, "PLC Tags", rows)
        note = f"; {skipped} 64-bit tag(s) left out" if skipped else ""
        self.status(f"Exported {len(rows) - 1} PLC tags to {path}{note}.")
        return len(rows) - 1

    def _export(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export PLC Tags", "plc_tags.csv", "CSV (*.csv)")
        if path:
            self.export_csv(path)

    def _export_tia(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export PLC Tags to TIA Portal", "PLCTags.xlsx",
                                              "Excel (*.xlsx)")
        if path:
            self.export_tia(path)
