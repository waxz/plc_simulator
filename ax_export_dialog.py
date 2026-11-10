"""
ax_export_dialog.py - the designer's SIMATIC AX export settings (File >
Export to SIMATIC AX, and the PLC Tags dock's button): where the .st file
goes, its NAMESPACE and STRUCT prefix, which frames, the PLC tags, the bit
arrays, the payload limit and an optional AxUnit golden-vector test. The
settings are the layout's (SimulatorConfig.axExport), so the next export -
from here, st_export.py or the AX project's generate_st_serde.py - writes
the same file the same way. ax_export.py does the writing.
"""
import os
from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QDialog, QDialogButtonBox, QFileDialog, QFormLayout, QHBoxLayout, QLabel,
                               QLineEdit, QListWidget, QListWidgetItem, QPushButton, QSpinBox, QVBoxLayout, QWidget)

from config_parser import AxExport


def default_settings(contract: dict) -> AxExport:
    """A first export: the I/O frames (not the metric layouts), the tags."""
    return AxExport(frames=[f["name"] for f in contract["frames"] if f["name"].startswith(("Sim_", "Belt_"))])


class AxExportDialog(QDialog):
    def __init__(self, parent, settings: AxExport, contract: dict, layout_path: Optional[str]):
        super().__init__(parent)
        self.setWindowTitle("Export to SIMATIC AX")
        self.layout_path = layout_path
        self.contract = contract
        form = QFormLayout()
        self.output = QLineEdit(settings.output)
        self.output.setPlaceholderText("sim_io.st - relative to the layout file")
        form.addRow("ST file", self._with_browse(self.output, "Choose the .st file"))
        self.namespace = QLineEdit(settings.namespace)
        self.namespace.setPlaceholderText("none")
        form.addRow("Namespace", self.namespace)
        self.prefix = QLineEdit(settings.prefix)
        self.prefix.setPlaceholderText("Sim (e.g. Station_1: Station_1_Input)")
        form.addRow("STRUCT prefix", self.prefix)
        self.frames = QListWidget()
        chosen = set(settings.frames) or {f["name"] for f in contract["frames"]}
        for frame in contract["frames"]:
            item = QListWidgetItem(f"{frame['name']}  ({frame['size']} bytes)")
            item.setData(Qt.UserRole, frame["name"])
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if frame["name"] in chosen else Qt.Unchecked)
            self.frames.addItem(item)
        self.frames.setMaximumHeight(110)
        form.addRow("Frames", self.frames)
        self.tags = QCheckBox("PLC tags: Sim_Tags with ReadTags / WriteTags")
        self.tags.setChecked(settings.tags)
        form.addRow("", self.tags)
        self.bit_arrays = QCheckBox("Laser banner beams as ARRAY OF BOOL (else one word)")
        self.bit_arrays.setChecked(settings.bitArrays)
        form.addRow("", self.bit_arrays)
        self.max_payload = QSpinBox(minimum=0, maximum=65535, value=settings.maxPayload)
        self.max_payload.setSpecialValueText("no limit")
        self.max_payload.setSuffix(" bytes")
        self.max_payload.setToolTip("Refuse a frame bigger than the PLC's TCP buffer carries "
                                    "(242: the AX TcpClient's 256-byte buffer)")
        form.addRow("Max frame", self.max_payload)
        self.vectors = QLineEdit(settings.vectors)
        self.vectors.setPlaceholderText("none - an AxUnit golden-vector test file")
        form.addRow("Test file", self._with_browse(self.vectors, "Choose the AxUnit test file"))
        note = QLabel("Saved with the layout: the next export - here, st_export.py or the AX project's "
                      "generate_st_serde.py - writes the same file.")
        note.setWordWrap(True)
        buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        self.export_button = buttons.addButton("Export", QDialogButtonBox.AcceptRole)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(note)
        layout.addWidget(buttons)
        self.resize(560, self.sizeHint().height())

    def _with_browse(self, edit: QLineEdit, title: str) -> QWidget:
        row = QWidget()
        box = QHBoxLayout(row)
        box.setContentsMargins(0, 0, 0, 0)
        box.addWidget(edit, 1)
        button = QPushButton("Browse...")
        button.clicked.connect(lambda: self._browse(edit, title))
        box.addWidget(button)
        return row

    def _browse(self, edit: QLineEdit, title: str):
        start = self._absolute(edit.text()) or (os.path.dirname(self.layout_path) if self.layout_path else "")
        path, _ = QFileDialog.getSaveFileName(self, title, start, "Structured Text (*.st)")
        if path:
            edit.setText(self._relative(path))

    def _absolute(self, path: str) -> str:
        if not path or os.path.isabs(path) or not self.layout_path:
            return path
        return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(self.layout_path)), path))

    def _relative(self, path: str) -> str:
        """Relative to the layout file, so the layout and the AX project can move together."""
        if not self.layout_path:
            return path
        try:
            return os.path.relpath(path, os.path.dirname(os.path.abspath(self.layout_path))).replace("\\", "/")
        except ValueError:                       # another drive
            return path

    def settings(self) -> AxExport:
        checked = [self.frames.item(i).data(Qt.UserRole) for i in range(self.frames.count())
                   if self.frames.item(i).checkState() == Qt.Checked]
        every = len(checked) == self.frames.count()
        return AxExport(output=self.output.text().strip(), namespace=self.namespace.text().strip(),
                        prefix=self.prefix.text().strip(), frames=[] if every else checked,
                        tags=self.tags.isChecked(), bitArrays=self.bit_arrays.isChecked(),
                        maxPayload=self.max_payload.value(), vectors=self.vectors.text().strip())
