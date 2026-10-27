"""
test_data_visualization_panel.py - unit tests for DataVisualizationPanel's
data-binding logic: tree population from a MetricsStore, group-selection
table filtering, and checkbox-driven plot series (including vector
flattening into per-component series and text metrics being excluded from
the plot).

Uses real Qt API calls (setCheckState, setCurrentItem) rather than
synthetic QTest mouse events - this session hit repeated QTest-simulation
artifacts unrelated to actual bugs, so state changes are driven directly
through the same Qt calls a click ultimately triggers.
"""
import sys
import unittest

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

app = QApplication.instance() or QApplication(sys.argv)

from data_visualization_panel import (
    PLOT_WINDOW_DEFAULT_S,
    TABLE_MODE_ALL_RECORDS,
    TABLE_MODE_LATEST_VALUE,
    DataVisualizationPanel,
    MetricsTableModel,
    _format_value,
    _qdatetime_to_ts,
    _ts_to_qdatetime,
)
from metrics_store import MetricsStore


def make_panel() -> DataVisualizationPanel:
    store = MetricsStore()
    store.add_sample("scan_time_ms", 5.0, group="perf", ts=1.0)
    store.add_sample("scan_time_ms", 6.0, group="perf", ts=2.0)
    store.add_sample("belt_velocity", [1.0, 0.5], group="physics", kind="vector", ts=1.0)
    store.add_sample("belt_velocity", [1.2, 0.6], group="physics", kind="vector", ts=2.0)
    store.add_sample("status", "ok", group="plc", kind="text", ts=1.0)
    return DataVisualizationPanel(store)


class FormatValueTest(unittest.TestCase):
    def test_float_is_fixed_precision(self):
        self.assertEqual(_format_value(3.14159), "3.1416")

    def test_bool_is_not_treated_as_a_float(self):
        # bool is an int subclass in Python - must not fall through the
        # float-formatting branch.
        self.assertEqual(_format_value(True), "True")

    def test_vector_is_bracketed_and_comma_joined(self):
        self.assertEqual(_format_value([1.0, 2.5]), "[1.000, 2.500]")

    def test_plain_string_passes_through(self):
        self.assertEqual(_format_value("ok"), "ok")


class TreePopulationTest(unittest.TestCase):
    def test_tree_reflects_every_group_and_metric_in_the_store(self):
        panel = make_panel()
        self.assertEqual(set(panel._group_items.keys()), {"perf", "physics", "plc"})
        self.assertEqual(
            set(panel._metric_items.keys()),
            {("perf", "scan_time_ms"), ("physics", "belt_velocity"), ("plc", "status")},
        )

    def test_refresh_is_idempotent_no_duplicate_items(self):
        panel = make_panel()
        panel.refresh()
        panel.refresh()
        self.assertEqual(panel.tree.topLevelItemCount(), 3)

    def test_a_new_metric_added_later_is_picked_up_on_next_refresh(self):
        panel = make_panel()
        panel.store.add_sample("cpu_load", 42.0, group="perf", ts=3.0)
        panel.refresh()
        self.assertIn(("perf", "cpu_load"), panel._metric_items)

    def test_check_state_survives_a_refresh_that_adds_an_unrelated_metric(self):
        panel = make_panel()
        leaf = panel._metric_items[("physics", "belt_velocity")]
        leaf.setCheckState(0, Qt.Checked)
        panel.store.add_sample("cpu_load", 1.0, group="perf", ts=3.0)
        panel.refresh()
        self.assertEqual(leaf.checkState(0), Qt.Checked)
        self.assertIn(("physics", "belt_velocity"), panel._checked_metrics)


class TableFilterTest(unittest.TestCase):
    def test_unfiltered_table_shows_every_group(self):
        panel = make_panel()
        self.assertEqual(panel.table_model.rowCount(), 5)

    def test_selecting_a_group_item_filters_the_table_to_it(self):
        panel = make_panel()
        panel.tree.setCurrentItem(panel._group_items["perf"])
        self.assertEqual(panel._selected_group, "perf")
        self.assertEqual(panel.table_model.rowCount(), 2)

    def test_selecting_a_metric_leaf_filters_by_its_parent_group(self):
        panel = make_panel()
        panel.tree.setCurrentItem(panel._metric_items[("physics", "belt_velocity")])
        self.assertEqual(panel._selected_group, "physics")
        self.assertEqual(panel.table_model.rowCount(), 2)


class PlotSelectionTest(unittest.TestCase):
    def test_checking_a_scalar_metric_adds_one_series_with_its_raw_points(self):
        panel = make_panel()
        panel._metric_items[("perf", "scan_time_ms")].setCheckState(0, Qt.Checked)
        self.assertEqual(list(panel.plot_widget._series.keys()), ["perf/scan_time_ms"])
        self.assertEqual(panel.plot_widget._series["perf/scan_time_ms"], [(1.0, 5.0), (2.0, 6.0)])

    def test_checking_a_vector_metric_expands_into_one_series_per_component(self):
        panel = make_panel()
        panel._metric_items[("physics", "belt_velocity")].setCheckState(0, Qt.Checked)
        self.assertEqual(
            set(panel.plot_widget._series.keys()),
            {"physics/belt_velocity[0]", "physics/belt_velocity[1]"},
        )
        self.assertEqual(panel.plot_widget._series["physics/belt_velocity[0]"], [(1.0, 1.0), (2.0, 1.2)])
        self.assertEqual(panel.plot_widget._series["physics/belt_velocity[1]"], [(1.0, 0.5), (2.0, 0.6)])

    def test_checking_a_text_metric_contributes_no_plot_series(self):
        panel = make_panel()
        panel._metric_items[("plc", "status")].setCheckState(0, Qt.Checked)
        self.assertIn(("plc", "status"), panel._checked_metrics)
        self.assertEqual(panel.plot_widget._series, {})

    def test_unchecking_removes_its_series(self):
        panel = make_panel()
        leaf = panel._metric_items[("perf", "scan_time_ms")]
        leaf.setCheckState(0, Qt.Checked)
        self.assertIn("perf/scan_time_ms", panel.plot_widget._series)
        leaf.setCheckState(0, Qt.Unchecked)
        self.assertNotIn(("perf", "scan_time_ms"), panel._checked_metrics)
        self.assertNotIn("perf/scan_time_ms", panel.plot_widget._series)

    def test_multiple_checked_metrics_coexist_in_the_plot(self):
        panel = make_panel()
        panel._metric_items[("perf", "scan_time_ms")].setCheckState(0, Qt.Checked)
        panel._metric_items[("physics", "belt_velocity")].setCheckState(0, Qt.Checked)
        self.assertEqual(
            set(panel.plot_widget._series.keys()),
            {"perf/scan_time_ms", "physics/belt_velocity[0]", "physics/belt_velocity[1]"},
        )


def make_panel_with_wide_time_range() -> DataVisualizationPanel:
    store = MetricsStore()
    # 11 samples spread across 100 seconds (one every 10s) - wide enough
    # that a slider window smaller than the full range actually excludes
    # some of them, unlike make_panel()'s samples which all sit within 1s
    # of each other.
    for i in range(11):
        store.add_sample("scan_time_ms", float(i), group="perf", ts=float(i * 10))
    return DataVisualizationPanel(store)


class TimestampConversionTest(unittest.TestCase):
    def test_round_trips_a_whole_second_timestamp(self):
        self.assertEqual(_qdatetime_to_ts(_ts_to_qdatetime(1000.0)), 1000.0)

    def test_round_trips_a_millisecond_precise_timestamp(self):
        self.assertAlmostEqual(_qdatetime_to_ts(_ts_to_qdatetime(1000.123)), 1000.123, places=3)


class PlotTimeRangeSelectorTest(unittest.TestCase):
    def test_default_range_spans_the_default_window_and_follow_is_checked(self):
        panel = make_panel()
        self.assertTrue(panel.plot_follow_checkbox.isChecked())
        self.assertAlmostEqual(panel._plot_to_ts - panel._plot_from_ts, PLOT_WINDOW_DEFAULT_S, delta=1.0)

    def test_narrow_frozen_range_excludes_points_outside_it(self):
        panel = make_panel_with_wide_time_range()
        panel._metric_items[("perf", "scan_time_ms")].setCheckState(0, Qt.Checked)
        panel.plot_from_edit.setDateTime(_ts_to_qdatetime(70.0))
        panel.plot_to_edit.setDateTime(_ts_to_qdatetime(100.0))
        points = panel.plot_widget._series["perf/scan_time_ms"]
        self.assertEqual([ts for ts, _ in points], [70.0, 80.0, 90.0, 100.0])

    def test_wide_frozen_range_includes_every_sample(self):
        panel = make_panel_with_wide_time_range()
        panel._metric_items[("perf", "scan_time_ms")].setCheckState(0, Qt.Checked)
        panel.plot_from_edit.setDateTime(_ts_to_qdatetime(0.0))
        panel.plot_to_edit.setDateTime(_ts_to_qdatetime(100.0))
        self.assertEqual(len(panel.plot_widget._series["perf/scan_time_ms"]), 11)

    def test_editing_either_field_manually_unchecks_follow_latest(self):
        panel = make_panel_with_wide_time_range()
        self.assertTrue(panel.plot_follow_checkbox.isChecked())
        panel.plot_from_edit.setDateTime(_ts_to_qdatetime(70.0))
        self.assertFalse(panel.plot_follow_checkbox.isChecked())

    def test_a_frozen_range_does_not_move_when_new_data_arrives(self):
        panel = make_panel_with_wide_time_range()
        panel._metric_items[("perf", "scan_time_ms")].setCheckState(0, Qt.Checked)
        panel.plot_from_edit.setDateTime(_ts_to_qdatetime(0.0))
        panel.plot_to_edit.setDateTime(_ts_to_qdatetime(50.0))
        self.assertFalse(panel.plot_follow_checkbox.isChecked())

        panel.store.add_sample("scan_time_ms", 99.0, group="perf", ts=500.0)
        panel.refresh()

        # Still frozen to [0, 50] - the new far-future sample must not
        # appear, and the selector fields must not have moved.
        points = panel.plot_widget._series.get("perf/scan_time_ms", [])
        self.assertTrue(all(ts <= 50.0 for ts, _ in points))
        self.assertEqual(_qdatetime_to_ts(panel.plot_to_edit.dateTime()), 50.0)

    def test_follow_mode_carries_the_window_forward_preserving_its_width(self):
        panel = make_panel_with_wide_time_range()
        panel._metric_items[("perf", "scan_time_ms")].setCheckState(0, Qt.Checked)
        panel.refresh()
        width_before = panel._plot_to_ts - panel._plot_from_ts

        panel.store.add_sample("scan_time_ms", 42.0, group="perf", ts=500.0)
        panel.refresh()

        self.assertTrue(panel.plot_follow_checkbox.isChecked())
        self.assertEqual(panel._plot_to_ts, 500.0)
        self.assertAlmostEqual(panel._plot_to_ts - panel._plot_from_ts, width_before, places=6)
        # The edit widgets must reflect the carried-forward range too.
        self.assertEqual(_qdatetime_to_ts(panel.plot_to_edit.dateTime()), 500.0)

    def test_a_series_entirely_outside_the_selected_range_is_dropped_not_shown_empty(self):
        panel = make_panel_with_wide_time_range()
        panel._metric_items[("perf", "scan_time_ms")].setCheckState(0, Qt.Checked)
        # A second metric with only very old samples, older than the
        # explicit range selected below.
        panel.store.add_sample("stale", 1.0, group="perf", ts=-1000.0)
        panel.refresh()
        panel._metric_items[("perf", "stale")].setCheckState(0, Qt.Checked)
        panel.plot_from_edit.setDateTime(_ts_to_qdatetime(90.0))
        panel.plot_to_edit.setDateTime(_ts_to_qdatetime(100.0))
        self.assertNotIn("perf/stale", panel.plot_widget._series)


class TableModeToggleTest(unittest.TestCase):
    def _names_shown(self, panel):
        model = panel.table_model
        return [model.data(model.index(i, 2)) for i in range(model.rowCount())]

    def test_defaults_to_all_records(self):
        panel = make_panel()
        self.assertEqual(panel.table_mode_combo.currentText(), TABLE_MODE_ALL_RECORDS)

    def test_all_records_mode_shows_full_history(self):
        panel = make_panel()  # scan_time_ms has 2 samples in the fixture
        panel.table_mode_combo.setCurrentText(TABLE_MODE_ALL_RECORDS)
        self.assertEqual(self._names_shown(panel).count("scan_time_ms"), 2)
        self.assertEqual(panel.table_model.rowCount(), 5)

    def test_latest_value_mode_shows_one_row_per_metric(self):
        panel = make_panel()
        panel.table_mode_combo.setCurrentText(TABLE_MODE_LATEST_VALUE)
        self.assertEqual(self._names_shown(panel).count("scan_time_ms"), 1)
        self.assertEqual(panel.table_model.rowCount(), 3)

    def test_latest_value_mode_reports_the_most_recent_sample(self):
        panel = make_panel()
        panel.table_mode_combo.setCurrentText(TABLE_MODE_LATEST_VALUE)
        model = panel.table_model
        row = next(
            i for i in range(model.rowCount())
            if model.data(model.index(i, 2)) == "scan_time_ms"
        )
        self.assertEqual(model.data(model.index(row, 4)), "6.0000")  # ts=2.0 sample, not ts=1.0

    def test_switching_modes_respects_the_current_group_filter(self):
        panel = make_panel()
        panel.tree.setCurrentItem(panel._group_items["perf"])
        panel.table_mode_combo.setCurrentText(TABLE_MODE_LATEST_VALUE)
        self.assertEqual(panel.table_model.rowCount(), 1)
        self.assertEqual(self._names_shown(panel), ["scan_time_ms"])


class MetricsTableModelTest(unittest.TestCase):
    def test_set_rows_updates_row_and_column_count(self):
        model = MetricsTableModel()
        model.set_rows([(1.0, "g", "n", "scalar", 5.0)])
        self.assertEqual(model.rowCount(), 1)
        self.assertEqual(model.columnCount(), 5)

    def test_header_labels(self):
        model = MetricsTableModel()
        self.assertEqual(
            [model.headerData(i, Qt.Horizontal) for i in range(5)],
            ["Time", "Group", "Name", "Kind", "Value"],
        )


if __name__ == "__main__":
    unittest.main()
