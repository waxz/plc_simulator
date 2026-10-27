"""Shared helpers for generating component ids.

Used by both the "Add <component>" toolbar actions (visualizer.py) and the
Component Manager's duplicate/copy actions (editor_ui.py) so there is a
single, crash-proof strategy for picking the next id instead of several
copy-pasted `int(id.split("_")[1])` snippets that break on any id that
doesn't happen to match a simple `prefix_number` shape.
"""
import re
from typing import Iterable


def next_indexed_id(existing_ids: Iterable[str], prefix: str) -> str:
    """Returns `<prefix>_N`, where N is one more than the highest trailing
    number among ids that exactly match `<prefix>_<digits>`.

    Ids that don't match that shape (e.g. multi-segment ids like
    `button_1_1_1`) are ignored rather than raising, so unrelated ids never
    prevent generating a new one.
    """
    pattern = re.compile(rf'^{re.escape(prefix)}_(\d+)$')
    max_idx = 0
    for existing_id in existing_ids:
        m = pattern.match(existing_id)
        if m:
            max_idx = max(max_idx, int(m.group(1)))
    return f"{prefix}_{max_idx + 1}"


def unique_incremented_id(existing_ids: Iterable[str], base_id: str) -> str:
    """Returns a variant of `base_id` that isn't in `existing_ids`, by
    incrementing its trailing number (or appending `_2` if it has none),
    counting up until the result is unique. Used for duplicate/copy actions.
    """
    existing = set(existing_ids)
    m = re.search(r'(\d+)$', base_id)
    if m:
        num_str = m.group(1)
        prefix = base_id[:-len(num_str)]
        idx = int(num_str) + 1
    else:
        prefix = f"{base_id}_"
        idx = 2

    candidate = f"{prefix}{idx}"
    while candidate in existing:
        idx += 1
        candidate = f"{prefix}{idx}"
    return candidate
