"""
data_visualization_panel.py - Qt dock panel for the data-visualization
feature: browse client-reported metrics (performance counters, cycle
times, function elapsed times, function results, vector values) grouped
in a tree, and view them either as a CSV-style table or a time-varying
line plot.

Reads from a shared MetricsStore (metrics_store.py), fed by
MQTTSyncEngine's metrics channel (see METRICS_MAPPING in
mqtt_data_exchange.py) on a background network thread - this module only
ever touches the store from the Qt/main thread, via a QTimer-driven
refresh, so no locking of its own is needed here (the store itself is
thread-safe).

No external charting dependency: TimeSeriesPlotWidget is a hand-rolled
QPainter line plot, matching this project's existing style of hand-rolled
QGraphicsItem/QPainter rendering (cad_editor_prototype.py, visualizer.py)
rather than pulling in pyqtgraph/matplotlib for one widget.
"""
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from PySide6.QtCore import QAbstractTableModel, QDateTime, QModelIndex, QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDateTimeEdit,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QSplitter,
    QTableView,
    QTabWidget,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from metrics_store import MetricsStore

# How often the panel re-reads the store - a monitoring panel doesn't need
# to track every single sample the instant it arrives; 2 Hz is smooth
# enough to watch and cheap enough to always run in the background.
REFRESH_INTERVAL_MS = 500
# Row cap for the table view - MetricsStore itself already bounds each
# series (max_samples_per_series), but with many distinct metrics the
# flattened table_rows() output could still be large; cap what actually
# gets pushed into the Qt model/view.
TABLE_ROW_LIMIT = 500

# Plot tab's From/To timestamp selector: default window width (seconds)
# used the first time data appears, and whenever "Follow latest" carries
# the window forward - how much history behind the newest visible sample
# is drawn. Filters each checked series' points before they reach
# TimeSeriesPlotWidget, rather than changing what MetricsStore retains
# (the store's own bound is about memory; this is purely a view-level
# "show me this time range" filter).
PLOT_WINDOW_DEFAULT_S = 60
# Qt's date/time widgets work in QDateTime, not raw epoch floats.
PLOT_TIMESTAMP_DISPLAY_FORMAT = "yyyy-MM-dd HH:mm:ss"

# Table view modes: the full scrolling history (MetricsStore.table_rows),
# or a one-row-per-metric snapshot of just the most recent sample
# (MetricsStore.latest_rows).
TABLE_MODE_ALL_RECORDS = "All records"
TABLE_MODE_LATEST_VALUE = "Latest value"


def _format_value(value: Any) -> str:
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, float):
        return f"{value:.4f}"
    if isinstance(value, list):
        return "[" + ", ".join(f"{v:.3f}" if isinstance(v, float) else str(v) for v in value) + "]"
    return str(value)


def _ts_to_qdatetime(ts: float) -> QDateTime:
    return QDateTime.fromMSecsSinceEpoch(int(ts * 1000))


def _qdatetime_to_ts(value: QDateTime) -> float:
    return value.toMSecsSinceEpoch() / 1000.0


class MetricsTableModel(QAbstractTableModel):
    """Read-only table model over a snapshot of MetricsStore.table_rows()
    - (ts, group, name, kind, value) tuples, newest first."""

    HEADERS = ["Time", "Group", "Name", "Kind", "Value"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rows: List[Tuple[float, str, str, str, Any]] = []

    def set_rows(self, rows: List[Tuple[float, str, str, str, Any]]) -> None:
        self.beginResetModel()
        self._rows = rows
        self.endResetModel()

    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return 0 if parent.isValid() else len(self._rows)

    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return 0 if parent.isValid() else len(self.HEADERS)

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role != Qt.DisplayRole or orientation != Qt.Horizontal:
            return None
        return self.HEADERS[section]

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole):
        if not index.isValid() or role != Qt.DisplayRole:
            return None
        ts, group, name, kind, value = self._rows[index.row()]
        column = index.column()
        if column == 0:
            return datetime.fromtimestamp(ts).strftime("%H:%M:%S.%f")[:-3]
        if column == 1:
            return group
        if column == 2:
            return name
        if column == 3:
            return kind
        if column == 4:
            return _format_value(value)
        return None


class TimeSeriesPlotWidget(QWidget):
    """Multi-series time-series line plot. `set_series()` takes a dict of
    label -> [(timestamp, numeric_value), ...] (oldest first) and is the
    entire public surface - all axis scaling/legend/gridline layout is
    recomputed from that on every paint, so there's no separate "which
    points are currently visible" state to keep in sync by hand."""

    _PALETTE = [
        QColor("#3b82f6"), QColor("#22c55e"), QColor("#ef4444"),
        QColor("#eab308"), QColor("#a855f7"), QColor("#06b6d4"),
        QColor("#f97316"), QColor("#ec4899"),
    ]
    _BACKGROUND = QColor("#1e1e1e")
    _GRID_COLOR = QColor("#333333")
    _AXIS_LABEL_COLOR = QColor("#aaaaaa")
    _BORDER_COLOR = QColor("#555555")
    _EMPTY_TEXT_COLOR = QColor("#888888")
    _LEGEND_TEXT_COLOR = QColor("#dddddd")

    def __init__(self, parent=None):
        super().__init__(parent)
        self._series: Dict[str, List[Tuple[float, float]]] = {}
        self.setMinimumHeight(200)

    def set_series(self, series: Dict[str, List[Tuple[float, float]]]) -> None:
        self._series = series
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect()
        painter.fillRect(rect, self._BACKGROUND)

        non_empty = {label: pts for label, pts in self._series.items() if pts}
        if not non_empty:
            painter.setPen(self._EMPTY_TEXT_COLOR)
            painter.drawText(rect, Qt.AlignCenter, "No data selected")
            painter.end()
            return

        margin_left, margin_right, margin_top, margin_bottom = 55, 10, 10, 10
        plot_rect = rect.adjusted(margin_left, margin_top, -margin_right, -margin_bottom)

        all_points = [pt for pts in non_empty.values() for pt in pts]
        xs = [p[0] for p in all_points]
        ys = [p[1] for p in all_points]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        if x_max - x_min < 1e-9:
            x_max = x_min + 1.0
        if y_max - y_min < 1e-9:
            y_min -= 1.0
            y_max += 1.0
        y_pad = (y_max - y_min) * 0.1
        y_min -= y_pad
        y_max += y_pad

        def to_px(ts: float, value: float) -> QPointF:
            x = plot_rect.left() + (ts - x_min) / (x_max - x_min) * plot_rect.width()
            y = plot_rect.bottom() - (value - y_min) / (y_max - y_min) * plot_rect.height()
            return QPointF(x, y)

        grid_lines = 4
        for i in range(grid_lines + 1):
            y = plot_rect.top() + i * plot_rect.height() / grid_lines
            painter.setPen(self._GRID_COLOR)
            painter.drawLine(QPointF(plot_rect.left(), y), QPointF(plot_rect.right(), y))
            value = y_max - i * (y_max - y_min) / grid_lines
            painter.setPen(self._AXIS_LABEL_COLOR)
            painter.drawText(
                QRectF(0, y - 8, margin_left - 6, 16), Qt.AlignRight | Qt.AlignVCenter, f"{value:.2f}"
            )

        painter.setPen(self._BORDER_COLOR)
        painter.drawRect(plot_rect)

        for idx, (label, points) in enumerate(non_empty.items()):
            if len(points) < 2:
                continue
            color = self._PALETTE[idx % len(self._PALETTE)]
            painter.setPen(QPen(color, 2))
            path_points = [to_px(ts, value) for ts, value in points]
            for i in range(len(path_points) - 1):
                painter.drawLine(path_points[i], path_points[i + 1])

        legend_x = plot_rect.left() + 6
        legend_y = plot_rect.top() + 6
        for idx, label in enumerate(non_empty.keys()):
            color = self._PALETTE[idx % len(self._PALETTE)]
            painter.fillRect(QRectF(legend_x, legend_y + idx * 16, 10, 10), color)
            painter.setPen(self._LEGEND_TEXT_COLOR)
            painter.drawText(
                QRectF(legend_x + 14, legend_y + idx * 16 - 3, 260, 16),
                Qt.AlignLeft | Qt.AlignVCenter,
                label,
            )

        painter.end()


class DataVisualizationPanel(QWidget):
    """Top-level panel: a group/metric tree on the left (checkboxes select
    what's plotted; clicking a group filters the table to it), and a
    Table/Plot tab pair on the right, both fed from the same MetricsStore
    on a timer."""

    def __init__(self, store: MetricsStore, parent=None):
        super().__init__(parent)
        self.store = store
        # Which (group, name) leaves are checked "show in plot".
        self._checked_metrics: set = set()
        # Which group the table is currently filtered to - None = all.
        self._selected_group: Optional[str] = None
        # Tracks which tree items already exist so refresh() only adds
        # NEW groups/metrics instead of rebuilding (and losing check-state
        # and expansion/scroll position) every tick.
        self._group_items: Dict[str, QTreeWidgetItem] = {}
        self._metric_items: Dict[Tuple[str, str], QTreeWidgetItem] = {}

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        splitter = QSplitter(Qt.Horizontal)
        outer.addWidget(splitter)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Metrics"])
        self.tree.itemChanged.connect(self._on_item_changed)
        self.tree.itemSelectionChanged.connect(self._on_selection_changed)
        splitter.addWidget(self.tree)

        self.tabs = QTabWidget()

        table_container = QWidget()
        table_layout = QVBoxLayout(table_container)
        table_layout.setContentsMargins(0, 0, 0, 0)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Show:"))
        self.table_mode_combo = QComboBox()
        self.table_mode_combo.addItems([TABLE_MODE_ALL_RECORDS, TABLE_MODE_LATEST_VALUE])
        self.table_mode_combo.currentTextChanged.connect(lambda _: self._refresh_table())
        mode_row.addWidget(self.table_mode_combo)
        mode_row.addStretch(1)
        table_layout.addLayout(mode_row)

        self.table_model = MetricsTableModel(self)
        self.table_view = QTableView()
        self.table_view.setModel(self.table_model)
        self.table_view.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table_view.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table_view.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        table_layout.addWidget(self.table_view)

        self.tabs.addTab(table_container, "Table")

        plot_container = QWidget()
        plot_layout = QVBoxLayout(plot_container)
        plot_layout.setContentsMargins(0, 0, 0, 0)

        # From/To timestamp selectors, not a slider - a slider gives coarse,
        # fiddly control over an arbitrary time range; typing/spinning an
        # exact timestamp (or picking one from the calendar popup) is both
        # more precise and easier to operate. "Follow latest" (checked by
        # default) keeps the window's WIDTH fixed and its edges carried
        # forward to track new data automatically; editing either field
        # manually - like scrubbing a video timeline - switches to a frozen
        # view of exactly the range requested, until re-checked.
        now = datetime.now().timestamp()
        self._plot_from_ts = now - PLOT_WINDOW_DEFAULT_S
        self._plot_to_ts = now

        window_row = QHBoxLayout()
        window_row.addWidget(QLabel("From:"))
        self.plot_from_edit = QDateTimeEdit()
        self.plot_from_edit.setDisplayFormat(PLOT_TIMESTAMP_DISPLAY_FORMAT)
        self.plot_from_edit.setCalendarPopup(True)
        self.plot_from_edit.dateTimeChanged.connect(self._on_plot_range_edited)
        window_row.addWidget(self.plot_from_edit)

        window_row.addWidget(QLabel("To:"))
        self.plot_to_edit = QDateTimeEdit()
        self.plot_to_edit.setDisplayFormat(PLOT_TIMESTAMP_DISPLAY_FORMAT)
        self.plot_to_edit.setCalendarPopup(True)
        self.plot_to_edit.dateTimeChanged.connect(self._on_plot_range_edited)
        window_row.addWidget(self.plot_to_edit)

        self.plot_follow_checkbox = QCheckBox("Follow latest")
        self.plot_follow_checkbox.setChecked(True)
        window_row.addWidget(self.plot_follow_checkbox)
        window_row.addStretch(1)
        plot_layout.addLayout(window_row)

        self._set_plot_range_edits(self._plot_from_ts, self._plot_to_ts)

        self.plot_widget = TimeSeriesPlotWidget()
        plot_layout.addWidget(self.plot_widget)

        self.tabs.addTab(plot_container, "Plot")
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(1, 1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(REFRESH_INTERVAL_MS)

        self.refresh()

    def refresh(self) -> None:
        self._sync_tree()
        self._refresh_table()
        self._refresh_plot()

    def _sync_tree(self) -> None:
        for group in self.store.groups():
            group_item = self._group_items.get(group)
            if group_item is None:
                group_item = QTreeWidgetItem(self.tree, [group])
                group_item.setFlags(group_item.flags() & ~Qt.ItemIsUserCheckable)
                font = group_item.font(0)
                font.setBold(True)
                group_item.setFont(0, font)
                self._group_items[group] = group_item
                self.tree.expandItem(group_item)

            for name in self.store.names(group):
                key = (group, name)
                if key in self._metric_items:
                    continue
                kind = self.store.kind_of(group, name) or "scalar"
                leaf = QTreeWidgetItem(group_item, [f"{name}  ({kind})"])
                leaf.setFlags(leaf.flags() | Qt.ItemIsUserCheckable)
                leaf.setCheckState(0, Qt.Checked if key in self._checked_metrics else Qt.Unchecked)
                leaf.setData(0, Qt.UserRole, key)
                self._metric_items[key] = leaf

    def _on_item_changed(self, item: QTreeWidgetItem, column: int) -> None:
        key = self._metric_key_for_item(item)
        if key is None:
            return  # a group item - not checkable, nothing to do
        if item.checkState(0) == Qt.Checked:
            self._checked_metrics.add(key)
        else:
            self._checked_metrics.discard(key)
        self._refresh_plot()

    def _on_selection_changed(self) -> None:
        items = self.tree.selectedItems()
        if not items:
            self._selected_group = None
        else:
            item = items[0]
            key = self._metric_key_for_item(item)
            # A metric leaf's group is its parent's text; a group item's
            # own text IS the group - either way, filter the table to
            # that group (metric-level filtering isn't needed yet: the
            # table already sorts newest-first, so the selected metric's
            # own rows are right at the top within its group).
            self._selected_group = key[0] if key is not None else item.text(0)
        self._refresh_table()

    @staticmethod
    def _metric_key_for_item(item: QTreeWidgetItem) -> Optional[Tuple[str, str]]:
        return item.data(0, Qt.UserRole)

    def _refresh_table(self) -> None:
        if self.table_mode_combo.currentText() == TABLE_MODE_LATEST_VALUE:
            rows = self.store.latest_rows(group=self._selected_group)
        else:
            rows = self.store.table_rows(group=self._selected_group, limit=TABLE_ROW_LIMIT)
        self.table_model.set_rows(rows)

    def _set_plot_range_edits(self, from_ts: float, to_ts: float) -> None:
        """Push from_ts/to_ts into the QDateTimeEdit widgets without
        re-triggering _on_plot_range_edited - used when "Follow latest"
        is advancing the window programmatically, not from a user edit."""
        self.plot_from_edit.blockSignals(True)
        self.plot_to_edit.blockSignals(True)
        self.plot_from_edit.setDateTime(_ts_to_qdatetime(from_ts))
        self.plot_to_edit.setDateTime(_ts_to_qdatetime(to_ts))
        self.plot_from_edit.blockSignals(False)
        self.plot_to_edit.blockSignals(False)

    def _on_plot_range_edited(self, _qdatetime: QDateTime) -> None:
        # A manual edit to either field takes over from auto-following -
        # like scrubbing a video timeline stops "live" playback.
        self.plot_follow_checkbox.setChecked(False)
        self._plot_from_ts = _qdatetime_to_ts(self.plot_from_edit.dateTime())
        self._plot_to_ts = _qdatetime_to_ts(self.plot_to_edit.dateTime())
        self._refresh_plot()

    def _refresh_plot(self) -> None:
        series: Dict[str, List[Tuple[float, float]]] = {}
        for group, name in sorted(self._checked_metrics):
            samples = self.store.series(group, name)
            if not samples:
                continue
            kind = self.store.kind_of(group, name)
            if kind == "vector":
                latest_value = samples[-1].value
                component_count = len(latest_value) if isinstance(latest_value, list) else 0
                for component_idx in range(component_count):
                    points = [
                        (sample.ts, sample.value[component_idx])
                        for sample in samples
                        if isinstance(sample.value, list) and component_idx < len(sample.value)
                    ]
                    if points:
                        series[f"{group}/{name}[{component_idx}]"] = points
            else:
                points = [
                    (sample.ts, sample.value)
                    for sample in samples
                    if isinstance(sample.value, (int, float)) and not isinstance(sample.value, bool)
                ]
                if points:
                    series[f"{group}/{name}"] = points

        if series:
            if self.plot_follow_checkbox.isChecked():
                # Carry the window forward, preserving whatever width is
                # currently selected, relative to the newest sample
                # actually present - not wall-clock now() - so this stays
                # meaningful for synthetic/replayed timestamps too.
                width = self._plot_to_ts - self._plot_from_ts
                if width <= 0:
                    width = PLOT_WINDOW_DEFAULT_S
                latest_ts = max(pt[0] for pts in series.values() for pt in pts)
                self._plot_to_ts = latest_ts
                self._plot_from_ts = latest_ts - width
                self._set_plot_range_edits(self._plot_from_ts, self._plot_to_ts)

            from_ts, to_ts = self._plot_from_ts, self._plot_to_ts
            series = {
                label: filtered
                for label, pts in series.items()
                if (filtered := [pt for pt in pts if from_ts <= pt[0] <= to_ts])
            }

        self.plot_widget.set_series(series)
