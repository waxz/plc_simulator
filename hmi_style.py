"""
hmi_style.py - one look for both windows (the pygame simulator and the Qt
layout editor), in the spirit of an industrial operator panel (ISA-101
"high-performance HMI"): a calm grey canvas, equipment in muted greys,
colour only where it means something (running, blocked, alarm, selection),
one small sans-serif typeface, and labels only where they fit.

Colours are (r, g, b) tuples; qcolor()/hex_() convert for Qt.
"""
import os

# -- palette ------------------------------------------------------------------

BG = (31, 35, 41)            # canvas
GRID = (39, 44, 51)          # minor grid
GRID_MAJOR = (47, 53, 61)    # every 5th line
AXIS_X = (62, 92, 70)        # world X axis (y = 0)
AXIS_Y = (92, 62, 66)        # world Y axis (x = 0)

PANEL = (36, 40, 47)         # side panels, status bar
PANEL_DARK = (27, 30, 36)
PANEL_BORDER = (56, 62, 71)
TEXT = (214, 219, 225)
TEXT_DIM = (139, 148, 158)
TEXT_FAINT = (98, 106, 116)

ACCENT = (56, 152, 214)      # selection, focus
RUNNING = (64, 178, 104)     # motor running, sensor clear, OK
WARNING = (226, 168, 44)     # paused, lagging
ALARM = (222, 76, 64)        # fault, blocked beam, stop

BELT_EDGE = (20, 23, 28)
FLOW = (236, 239, 242)       # flow chevrons on a running belt
SOURCE = (70, 120, 200)
SINK = (200, 128, 60)
PARCEL_EDGE = (18, 20, 24)
TAG_BG = (22, 25, 30, 205)   # label backdrop (with alpha)

# -- type ---------------------------------------------------------------------

UI_FONT_FAMILY = "Segoe UI"  # Qt widgets (falls back to the platform sans)
LABEL_PX = 11                # canvas labels, screen pixels
HUD_PX = 13


def bundled_font_path(bold: bool = False) -> str:
    """Noto Sans shipped with pygame_gui (always there, any platform)."""
    import pygame_gui
    name = "NotoSans-Bold.ttf" if bold else "NotoSans-Regular.ttf"
    return os.path.join(os.path.dirname(pygame_gui.__file__), "data", name)


def hex_(rgb) -> str:
    return "#{:02x}{:02x}{:02x}".format(*rgb[:3])


def qcolor(rgb, alpha: int = 255):
    from PySide6.QtGui import QColor
    return QColor(rgb[0], rgb[1], rgb[2], rgb[3] if len(rgb) > 3 else alpha)


def shade(rgb, factor: float):
    """Darker (< 1) or lighter (> 1) version of a colour."""
    if factor < 1:
        return tuple(max(0, int(c * factor)) for c in rgb[:3])
    return tuple(min(255, int(c + (255 - c) * (factor - 1))) for c in rgb[:3])


# -- Qt widget style ----------------------------------------------------------

def qt_stylesheet() -> str:
    """Dark, compact style for the layout editor's widgets."""
    t, d, f = hex_(TEXT), hex_(TEXT_DIM), hex_(TEXT_FAINT)
    p, pd, b, a = hex_(PANEL), hex_(PANEL_DARK), hex_(PANEL_BORDER), hex_(ACCENT)
    return f"""
    QMainWindow, QDialog, QWidget {{ background: {p}; color: {t}; font-family: "{UI_FONT_FAMILY}"; font-size: 9pt; }}
    QMainWindow::separator {{ background: {pd}; width: 3px; height: 3px; }}
    QDockWidget {{ titlebar-close-icon: none; }}
    QDockWidget::title {{ background: {pd}; color: {d}; padding: 4px 8px; text-transform: uppercase;
                          font-size: 8pt; font-weight: 600; letter-spacing: 1px; border-bottom: 1px solid {b}; }}
    QMenuBar {{ background: {pd}; color: {t}; border-bottom: 1px solid {b}; }}
    QMenuBar::item {{ padding: 4px 10px; background: transparent; }}
    QMenuBar::item:selected, QMenu::item:selected {{ background: {a}; color: white; }}
    QMenu {{ background: {pd}; border: 1px solid {b}; padding: 4px 0; }}
    QMenu::item {{ padding: 4px 24px 4px 18px; }}
    QMenu::separator {{ height: 1px; background: {b}; margin: 4px 8px; }}
    QToolBar {{ background: {pd}; border: none; border-bottom: 1px solid {b}; spacing: 4px; padding: 2px 6px; }}
    QToolButton {{ color: {t}; background: transparent; border: 1px solid transparent; border-radius: 3px;
                   padding: 3px 8px; }}
    QToolButton:hover {{ border-color: {b}; background: {p}; }}
    QToolButton:checked {{ background: {a}; color: white; }}
    QToolButton:disabled, QLabel:disabled {{ color: {f}; }}
    QPushButton {{ background: {pd}; color: {t}; border: 1px solid {b}; border-radius: 3px; padding: 4px 10px; }}
    QPushButton:hover {{ border-color: {a}; }}
    QPushButton:pressed {{ background: {a}; color: white; }}
    QPushButton:disabled {{ color: {f}; border-color: {pd}; }}
    QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ background: {pd}; color: {t}; border: 1px solid {b};
                   border-radius: 2px; padding: 2px 4px; selection-background-color: {a}; }}
    QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color: {a}; }}
    QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {{ color: {f}; }}
    QComboBox QAbstractItemView {{ background: {pd}; border: 1px solid {b}; selection-background-color: {a}; }}
    QListWidget, QTreeWidget, QTableWidget {{ background: {pd}; border: 1px solid {b}; outline: none; }}
    QListWidget::item {{ padding: 3px 4px; }}
    QListWidget::item:selected {{ background: {a}; color: white; }}
    QListWidget::item:hover:!selected {{ background: {p}; }}
    QGroupBox {{ border: 1px solid {b}; border-radius: 3px; margin-top: 14px; padding-top: 6px; }}
    QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; color: {d}; font-weight: 600; }}
    QCheckBox {{ spacing: 6px; }}
    QStatusBar {{ background: {pd}; color: {d}; border-top: 1px solid {b}; }}
    QStatusBar QLabel {{ color: {d}; padding: 0 8px; }}
    QScrollBar:vertical, QScrollBar:horizontal {{ background: {pd}; border: none; width: 10px; height: 10px; }}
    QScrollBar::handle {{ background: {b}; border-radius: 4px; min-height: 24px; min-width: 24px; }}
    QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
    QToolTip {{ background: {pd}; color: {t}; border: 1px solid {b}; padding: 4px; }}
    QGraphicsView {{ border: none; background: {hex_(BG)}; }}
    QSplitter::handle {{ background: {pd}; }}
    """
