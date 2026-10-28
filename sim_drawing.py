"""
sim_drawing.py - the layout's DXF background (config.background, set up in
the layout editor) drawn behind the belts in the simulator window.

The drawing is read once with the layout's own window, filters, scale and
offset (cad/dxf_io.py, the same as the editor), then rendered onto a cached
surface a margin larger than the window: panning just moves that surface,
and the lines are only redrawn on zoom, resize, theme change or a pan
beyond the margin - a drawing with tens of thousands of lines stays smooth.
"""
import os
from typing import List, Optional, Tuple

import numpy as np
import pygame

import hmi_style as hmi


class DrawingLayer:
    def __init__(self):
        self.lines: List[np.ndarray] = []      # world metres, one (n, 2) array per polyline
        self.boxes = np.zeros((0, 4))          # per polyline: min_x, min_y, max_x, max_y
        self.loaded_from: Optional[str] = None
        self.error: Optional[str] = None
        self._surface = None
        self._key = None
        self._camera = (0.0, 0.0)
        self._margin = 0

    @property
    def has_drawing(self) -> bool:
        return bool(self.lines)

    @staticmethod
    def resolve(path: str, config_path: Optional[str]) -> str:
        if os.path.isabs(path) or not config_path:
            return path
        return os.path.join(os.path.dirname(os.path.abspath(config_path)), path)

    def load(self, background, config_path: Optional[str]) -> bool:
        """Read the background drawing; False (with `error` set) if it
        can't be. A layout without one, or with it hidden, has none."""
        self.lines, self.boxes, self.error, self._key = [], np.zeros((0, 4)), None, None
        if background is None or not background.visible:
            return False
        from cad import dxf_io  # ezdxf: only when a layout has a drawing
        path = self.resolve(background.path, config_path)
        try:
            geom = dxf_io.read_background(path, clip=background.clip, skip_types=background.skipTypes,
                                          hidden_layers=background.hiddenLayers)
        except Exception as exc:  # noqa: BLE001 - a missing / broken file must not stop the simulator
            self.error = f"Could not load the drawing {path}: {exc}"
            return False
        polylines = dxf_io.background_in_metres(geom, background.scale, background.offsetX, background.offsetY)
        self.lines = [np.asarray(pl, dtype=float) for pl in polylines if len(pl) >= 2]
        if self.lines:
            self.boxes = np.array([(a[:, 0].min(), a[:, 1].min(), a[:, 0].max(), a[:, 1].max()) for a in self.lines])
        self.loaded_from = path
        return True

    def bounds(self) -> Optional[Tuple[float, float, float, float]]:
        if not self.lines:
            return None
        return (self.boxes[:, 0].min(), self.boxes[:, 1].min(), self.boxes[:, 2].max(), self.boxes[:, 3].max())

    def draw(self, screen, camera_x: float, camera_y: float, pixels_per_metre: float):
        if not self.lines:
            return
        w, h = screen.get_size()
        key = (round(pixels_per_metre, 9), w, h, hmi.THEME)
        dx, dy = camera_x - self._camera[0], camera_y - self._camera[1]
        if key != self._key or abs(dx) > self._margin or abs(dy) > self._margin:
            self._render(w, h, camera_x, camera_y, pixels_per_metre)
            self._key = key
            dx = dy = 0
        screen.blit(self._surface, (dx - self._margin, dy - self._margin))

    def _render(self, w: int, h: int, camera_x: float, camera_y: float, k: float):
        m = max(w, h) // 2
        if self._surface is None or self._surface.get_size() != (w + 2 * m, h + 2 * m):
            self._surface = pygame.Surface((w + 2 * m, h + 2 * m), pygame.SRCALPHA)
        self._surface.fill((0, 0, 0, 0))
        self._margin, self._camera = m, (camera_x, camera_y)
        # Visible world window (with the margin), as in Renderer._screen_to_world.
        x0, x1 = (-m - camera_x) / k, (w + m - camera_x) / k
        y0, y1 = -(h + m - camera_y) / k, -(-m - camera_y) / k
        b = self.boxes
        visible = np.nonzero((b[:, 2] >= x0) & (b[:, 0] <= x1) & (b[:, 3] >= y0) & (b[:, 1] <= y1)
                             & (np.maximum(b[:, 2] - b[:, 0], b[:, 3] - b[:, 1]) * k >= 1.0))[0]
        colour = hmi.DRAWING
        ox, oy = camera_x + m, camera_y + m
        for i in visible:
            a = self.lines[i]
            pts = np.empty_like(a)
            pts[:, 0] = a[:, 0] * k + ox
            pts[:, 1] = -a[:, 1] * k + oy
            pygame.draw.lines(self._surface, colour, False, pts.tolist())
