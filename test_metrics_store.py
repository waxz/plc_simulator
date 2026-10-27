"""
test_metrics_store.py - unit tests for metrics_store.py's MetricsStore:
bounded ring-buffer storage, grouping, kind inference, batch ingestion,
and thread-safety under concurrent writers.
"""
import threading
import unittest

from metrics_store import DEFAULT_GROUP, MetricsStore, infer_kind


class InferKindTest(unittest.TestCase):
    def test_list_is_vector(self):
        self.assertEqual(infer_kind([1.0, 2.0, 3.0]), "vector")

    def test_bool_is_text(self):
        # Checked before int, since bool is an int subclass in Python -
        # a metric reporting True/False shouldn't be miscategorized as a
        # numeric scalar.
        self.assertEqual(infer_kind(True), "text")

    def test_str_is_text(self):
        self.assertEqual(infer_kind("ok"), "text")

    def test_float_is_scalar(self):
        self.assertEqual(infer_kind(12.5), "scalar")

    def test_int_is_scalar(self):
        self.assertEqual(infer_kind(3), "scalar")


class AddSampleTest(unittest.TestCase):
    def test_infers_kind_when_not_given(self):
        store = MetricsStore()
        store.add_sample("scan_time_ms", 12.3)
        self.assertEqual(store.kind_of(DEFAULT_GROUP, "scan_time_ms"), "scalar")

    def test_explicit_kind_overrides_inference(self):
        store = MetricsStore()
        store.add_sample("flag", 1, kind="text")
        self.assertEqual(store.kind_of(DEFAULT_GROUP, "flag"), "text")

    def test_defaults_to_default_group(self):
        store = MetricsStore()
        store.add_sample("x", 1.0)
        self.assertEqual(store.groups(), [DEFAULT_GROUP])

    def test_custom_group(self):
        store = MetricsStore()
        store.add_sample("scan_time_ms", 1.0, group="physics")
        self.assertEqual(store.groups(), ["physics"])
        self.assertEqual(store.names("physics"), ["scan_time_ms"])

    def test_empty_name_raises(self):
        store = MetricsStore()
        with self.assertRaises(ValueError):
            store.add_sample("", 1.0)

    def test_unknown_kind_raises(self):
        store = MetricsStore()
        with self.assertRaises(ValueError):
            store.add_sample("x", 1.0, kind="bogus")

    def test_explicit_ts_is_preserved(self):
        store = MetricsStore()
        store.add_sample("x", 1.0, ts=1000.0)
        sample = store.latest(DEFAULT_GROUP, "x")
        self.assertEqual(sample.ts, 1000.0)

    def test_omitted_ts_gets_a_real_timestamp(self):
        import time

        before = time.time()
        store = MetricsStore()
        store.add_sample("x", 1.0)
        after = time.time()
        sample = store.latest(DEFAULT_GROUP, "x")
        self.assertGreaterEqual(sample.ts, before)
        self.assertLessEqual(sample.ts, after)

    def test_re_registering_with_a_different_kind_adopts_the_new_kind(self):
        store = MetricsStore()
        store.add_sample("x", 1.0)  # scalar
        self.assertEqual(store.kind_of(DEFAULT_GROUP, "x"), "scalar")
        store.add_sample("x", [1.0, 2.0])  # now a vector
        self.assertEqual(store.kind_of(DEFAULT_GROUP, "x"), "vector")
        # The old scalar sample is still in the series - kind describes
        # "how to interpret the series going forward", not a retroactive
        # rewrite of history.
        self.assertEqual(len(store.series(DEFAULT_GROUP, "x")), 2)


class RingBufferBoundTest(unittest.TestCase):
    def test_series_is_bounded_to_max_samples_per_series(self):
        store = MetricsStore(max_samples_per_series=5)
        for i in range(20):
            store.add_sample("x", float(i))
        samples = store.series(DEFAULT_GROUP, "x")
        self.assertEqual(len(samples), 5)
        # Oldest evicted first - only the most recent 5 values survive.
        self.assertEqual([s.value for s in samples], [15.0, 16.0, 17.0, 18.0, 19.0])

    def test_different_series_have_independent_bounds(self):
        store = MetricsStore(max_samples_per_series=3)
        for i in range(10):
            store.add_sample("a", i)
            store.add_sample("b", i * 2)
        self.assertEqual(len(store.series(DEFAULT_GROUP, "a")), 3)
        self.assertEqual(len(store.series(DEFAULT_GROUP, "b")), 3)


class LatestAndSeriesTest(unittest.TestCase):
    def test_latest_returns_none_for_unknown_metric(self):
        store = MetricsStore()
        self.assertIsNone(store.latest(DEFAULT_GROUP, "nope"))

    def test_latest_returns_most_recently_added_sample(self):
        store = MetricsStore()
        store.add_sample("x", 1.0)
        store.add_sample("x", 2.0)
        store.add_sample("x", 3.0)
        self.assertEqual(store.latest(DEFAULT_GROUP, "x").value, 3.0)

    def test_series_snapshot_is_independent_of_further_writes(self):
        store = MetricsStore()
        store.add_sample("x", 1.0)
        snapshot = store.series(DEFAULT_GROUP, "x")
        store.add_sample("x", 2.0)
        self.assertEqual(len(snapshot), 1)
        self.assertEqual(len(store.series(DEFAULT_GROUP, "x")), 2)

    def test_series_for_unknown_metric_is_empty_list(self):
        store = MetricsStore()
        self.assertEqual(store.series(DEFAULT_GROUP, "nope"), [])


class AddSamplesBatchTest(unittest.TestCase):
    def test_accepts_a_well_formed_batch(self):
        store = MetricsStore()
        count = store.add_samples(
            [
                {"group": "physics", "name": "scan_time_ms", "value": 5.0},
                {"group": "physics", "name": "belt_1_velocity", "value": [1.0, 0.5], "kind": "vector"},
                {"name": "result", "value": "ok"},
            ]
        )
        self.assertEqual(count, 3)
        self.assertEqual(sorted(store.groups()), [DEFAULT_GROUP, "physics"])
        self.assertEqual(sorted(store.names("physics")), ["belt_1_velocity", "scan_time_ms"])

    def test_a_malformed_entry_is_skipped_without_discarding_the_rest(self):
        store = MetricsStore()
        count = store.add_samples(
            [
                {"name": "good_1", "value": 1.0},
                {"value": 2.0},  # missing "name"
                "not even a dict",
                {"name": "", "value": 3.0},  # invalid name -> ValueError inside add_sample
                {"name": "good_2", "value": 4.0},
            ]
        )
        self.assertEqual(count, 2)
        self.assertEqual(sorted(store.names(DEFAULT_GROUP)), ["good_1", "good_2"])

    def test_empty_batch_returns_zero(self):
        store = MetricsStore()
        self.assertEqual(store.add_samples([]), 0)


class TableRowsTest(unittest.TestCase):
    def test_flattens_all_groups_and_metrics(self):
        store = MetricsStore()
        store.add_sample("a", 1.0, group="g1", ts=1.0)
        store.add_sample("b", 2.0, group="g2", ts=2.0)
        rows = store.table_rows()
        self.assertEqual(len(rows), 2)
        names = {row[2] for row in rows}
        self.assertEqual(names, {"a", "b"})

    def test_filters_by_group(self):
        store = MetricsStore()
        store.add_sample("a", 1.0, group="g1", ts=1.0)
        store.add_sample("b", 2.0, group="g2", ts=2.0)
        rows = store.table_rows(group="g1")
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0][2], "a")

    def test_unknown_group_returns_no_rows(self):
        store = MetricsStore()
        store.add_sample("a", 1.0, group="g1")
        self.assertEqual(store.table_rows(group="nope"), [])

    def test_sorted_newest_first(self):
        store = MetricsStore()
        store.add_sample("a", 1.0, ts=1.0)
        store.add_sample("a", 2.0, ts=3.0)
        store.add_sample("a", 3.0, ts=2.0)
        rows = store.table_rows()
        self.assertEqual([row[0] for row in rows], [3.0, 2.0, 1.0])

    def test_limit_caps_row_count_after_sorting(self):
        store = MetricsStore()
        for i in range(10):
            store.add_sample("a", float(i), ts=float(i))
        rows = store.table_rows(limit=3)
        self.assertEqual(len(rows), 3)
        self.assertEqual([row[0] for row in rows], [9.0, 8.0, 7.0])

    def test_row_shape_is_ts_group_name_kind_value(self):
        store = MetricsStore()
        store.add_sample("scan_time_ms", 5.0, group="physics", ts=42.0)
        row = store.table_rows()[0]
        self.assertEqual(row, (42.0, "physics", "scan_time_ms", "scalar", 5.0))


class LatestRowsTest(unittest.TestCase):
    def test_one_row_per_metric_using_its_most_recent_sample(self):
        store = MetricsStore()
        store.add_sample("a", 1.0, ts=1.0)
        store.add_sample("a", 2.0, ts=2.0)
        store.add_sample("b", 5.0, ts=1.5)
        rows = store.latest_rows()
        self.assertEqual(len(rows), 2)
        by_name = {row[2]: row for row in rows}
        self.assertEqual(by_name["a"][4], 2.0)
        self.assertEqual(by_name["a"][0], 2.0)  # ts of the latest sample, not the first
        self.assertEqual(by_name["b"][4], 5.0)

    def test_filters_by_group(self):
        store = MetricsStore()
        store.add_sample("a", 1.0, group="g1")
        store.add_sample("b", 2.0, group="g2")
        rows = store.latest_rows(group="g1")
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0][2], "a")

    def test_unknown_group_returns_no_rows(self):
        store = MetricsStore()
        store.add_sample("a", 1.0, group="g1")
        self.assertEqual(store.latest_rows(group="nope"), [])

    def test_sorted_by_group_then_name_not_by_recency(self):
        store = MetricsStore()
        store.add_sample("z", 1.0, group="g2", ts=100.0)
        store.add_sample("a", 2.0, group="g1", ts=1.0)
        store.add_sample("m", 3.0, group="g1", ts=50.0)
        rows = store.latest_rows()
        self.assertEqual([(row[1], row[2]) for row in rows], [("g1", "a"), ("g1", "m"), ("g2", "z")])

    def test_empty_store_returns_no_rows(self):
        store = MetricsStore()
        self.assertEqual(store.latest_rows(), [])


class ClearTest(unittest.TestCase):
    def test_clear_empties_the_store(self):
        store = MetricsStore()
        store.add_sample("a", 1.0, group="g1")
        store.clear()
        self.assertEqual(store.groups(), [])
        self.assertEqual(store.table_rows(), [])


class ThreadSafetyTest(unittest.TestCase):
    def test_concurrent_writers_do_not_lose_or_corrupt_samples(self):
        store = MetricsStore(max_samples_per_series=100000)
        writer_count = 8
        samples_per_writer = 500

        def writer(writer_id: int):
            for i in range(samples_per_writer):
                store.add_sample(f"metric_{writer_id % 3}", float(i), group="stress")

        threads = [threading.Thread(target=writer, args=(i,)) for i in range(writer_count)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        total = sum(len(store.series("stress", f"metric_{i}")) for i in range(3))
        self.assertEqual(total, writer_count * samples_per_writer)


if __name__ == "__main__":
    unittest.main()
