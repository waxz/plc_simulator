"""
tag_monitor.py - the PLC tag monitor of the simulator window (F4).

Every PLC tag of the layout (plc_tags.tags_of: the names the AX generator
gives Plant_Tags' members, the TIA addresses of the frames) with its value
now, as the simulator has it: what it sends the PLC (inputs, %I) and what
the PLC last commanded (outputs, %Q). A value that changed in the last
second is highlighted.

F4 cycles: all tags -> inputs -> outputs -> off. The mouse wheel over the
panel scrolls it.

    monitor = TagMonitor(config)
    monitor.rows()          # [(tag, text, changed)] for the current view
    monitor.draw(renderer)  # onto the window, left of the canvas
"""
import time
from typing import Dict, List, Optional, Tuple

import pygame

import hmi_style as hmi
import io_signals
import plc_tags

VIEWS = ("all", "inputs", "outputs")
CHANGED_S = 1.0          # a value that changed this long ago is still highlighted
MAX_ROWS = 400


def format_value(tag: plc_tags.Tag, value) -> str:
    """A tag's value as a PLC monitor shows it."""
    if value is None:
        return "-"
    if tag.count > 1 or isinstance(value, str) and tag.data_type.startswith("Array"):
        bits = value if isinstance(value, str) else "".join("1" if b else "0" for b in value)
        return bits[:tag.count]                                     # beam 0 first
    if tag.data_type == "Bool":
        if isinstance(value, str):                                  # a sensor's "0" / "1"
            value = value[:1] == "1"
        elif isinstance(value, (list, tuple)):
            value = bool(value and value[0])
        return "TRUE" if value else "FALSE"
    if tag.data_type == "Real":
        return f"{float(value):.3f}"
    try:
        return str(int(value))
    except (TypeError, ValueError):
        return str(value)


class TagMonitor:
    def __init__(self, config):
        self.config = config
        self.view: Optional[str] = None          # None: hidden; else one of VIEWS
        self.scroll = 0
        self.rect: Optional[pygame.Rect] = None  # where it was drawn last (the wheel scrolls it there)
        self._tags: List[plc_tags.Tag] = []
        self._summary = ""
        self._last: Dict[str, str] = {}
        self._changed_at: Dict[str, float] = {}
        self.refresh()

    def refresh(self) -> None:
        """The tag list again (the layout changed)."""
        try:
            self._tags, self._summary = plc_tags.tags_of(self.config)
        except Exception as exc:  # noqa: BLE001 - a layout the exchange can't describe: show why
            self._tags, self._summary = [], f"no tags: {exc}"

    @property
    def visible(self) -> bool:
        return self.view is not None

    def cycle(self) -> None:
        """F4: all -> inputs -> outputs -> hidden."""
        if self.view is None:
            self.view = VIEWS[0]
        else:
            i = VIEWS.index(self.view) + 1
            self.view = VIEWS[i] if i < len(VIEWS) else None
        self.scroll = 0

    def scroll_by(self, rows: int) -> None:
        self.scroll = max(0, self.scroll + rows)

    def contains(self, pos) -> bool:
        return self.visible and self.rect is not None and self.rect.collidepoint(pos)

    def _shown(self) -> List[plc_tags.Tag]:
        if self.view == "inputs":
            return [t for t in self._tags if t.is_input]
        if self.view == "outputs":
            return [t for t in self._tags if not t.is_input]
        return list(self._tags)

    def rows(self, now: Optional[float] = None) -> List[Tuple[plc_tags.Tag, str, bool]]:
        """(tag, value text, changed lately) for the view, every tag's value read now."""
        now = time.monotonic() if now is None else now
        tags = self._shown()
        ids = sorted({t.component for t in tags if t.component != "clock"})
        values = io_signals.read(self.config, ids) if ids else {}
        out = []
        for tag in tags[:MAX_ROWS]:
            if tag.component == "clock":
                text = "(the simulator PC's clock, ns)"
            else:
                signal = plc_tags.INDEX.sub("", tag.signal.split("[")[0])
                value = values.get(tag.component, {}).get("values", {}).get(signal)
                text = format_value(tag, value)
            if self._last.get(tag.key) not in (None, text):
                self._changed_at[tag.key] = now
            self._last[tag.key] = text
            out.append((tag, text, now - self._changed_at.get(tag.key, -1e9) < CHANGED_S))
        return out

    # -- drawing ----------------------------------------------------------------------------

    def draw(self, r, top: int, bottom: int, left: int = 10) -> None:
        if not self.visible:
            return
        font, bold = r.hud_small, r.hud_bold
        line_h = font.get_height() + 2
        rows = self.rows()
        title = (f"PLC TAGS - {self.view} ({len(rows)})   F4: "
                 + {"all": "inputs", "inputs": "outputs", "outputs": "off"}[self.view] + "   wheel: scroll")
        name_w = max([font.size(t.name)[0] for t, _v, _c in rows] + [80])
        addr_w = max([font.size(t.address)[0] for t, _v, _c in rows] + [60])
        type_w = max([font.size(t.data_type)[0] for t, _v, _c in rows] + [40])
        value_w = max([font.size(v)[0] for _t, v, _c in rows] + [60])
        width = max(name_w + addr_w + type_w + value_w + 54, bold.size(title)[0] + 20)
        visible_rows = max(1, (bottom - top - 40) // line_h)
        self.scroll = min(self.scroll, max(0, len(rows) - visible_rows))
        shown = rows[self.scroll:self.scroll + visible_rows]
        height = 30 + line_h * max(1, len(shown)) + 8
        self.rect = pygame.Rect(left, top, width, height)
        surf = pygame.Surface(self.rect.size, pygame.SRCALPHA)
        pygame.draw.rect(surf, (*hmi.PANEL_DARK, 240), surf.get_rect(), border_radius=6)
        pygame.draw.rect(surf, (*hmi.PANEL_BORDER, 255), surf.get_rect(), 1, border_radius=6)
        r.screen.blit(surf, self.rect)
        r.screen.blit(bold.render(title, True, hmi.TEXT_DIM), (left + 10, top + 6))
        if self.scroll or self.scroll + visible_rows < len(rows):
            more = font.render(f"{self.scroll + 1}-{self.scroll + len(shown)} of {len(rows)}", True, hmi.TEXT_FAINT)
            r.screen.blit(more, (self.rect.right - more.get_width() - 10, top + 8))
        y = top + 28
        x_addr = left + 10 + name_w + 14
        x_type = x_addr + addr_w + 14
        x_value = x_type + type_w + 14
        for tag, value, changed in shown:
            if changed:                                      # a translucent amber bar, either theme
                bar = pygame.Surface((width - 8, line_h), pygame.SRCALPHA)
                bar.fill((*hmi.WARNING, 70))
                r.screen.blit(bar, (left + 4, y - 1))
            on = value == "TRUE" or (tag.count > 1 and "1" in value)
            colour = hmi.RUNNING if on else hmi.TEXT
            r.screen.blit(font.render(tag.name, True, hmi.TEXT if tag.is_input else hmi.TEXT_DIM), (left + 10, y))
            r.screen.blit(font.render(tag.address, True, hmi.TEXT_FAINT), (x_addr, y))
            r.screen.blit(font.render(tag.data_type, True, hmi.TEXT_FAINT), (x_type, y))
            r.screen.blit(font.render(value, True, colour), (x_value, y))
            y += line_h
        if not rows:
            r.screen.blit(font.render(self._summary or "no PLC tags", True, hmi.TEXT_FAINT), (left + 10, y))
