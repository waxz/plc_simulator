"""
component_library.py - reusable component templates, stored as data.

A library entry is a named snapshot of one component (any kind: a belt of
any type/shape, a sensor, source, sink, button or LED) without its id and
position. Placing it builds a fresh model with a new id at the drop point;
path / polygon belts keep their geometry because it is stored relative to
the belt's x/y.

Libraries are JSON files, so they can be shared between layouts and people:

    {
      "format": "conveyor-component-library",
      "version": 1,
      "components": [
        {"name": "Merge 45", "kind": "belt", "folder": "Merges", "model": {...}},
        ...
      ]
    }

The layout editor keeps the user's library in default_library_path() and
saves it on every change; File-style Import/Export read and write any other
library file. No Qt here.
"""
import json
import os
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

from pydantic import BaseModel, ValidationError

from config_parser import Belt, ControlButton, LedIndicator, Sensor, Sink, Source

FORMAT = "conveyor-component-library"
VERSION = 1

MODEL_FOR_KIND: Dict[str, type] = {
    "belt": Belt,
    "sensor": Sensor,
    "source": Source,
    "sink": Sink,
    "button": ControlButton,
    "led": LedIndicator,
}
KIND_FOR_MODEL = {cls: kind for kind, cls in MODEL_FOR_KIND.items()}

# Runtime state that must not travel with a template.
_RUNTIME_FIELDS = {"id", "isActive", "sensor_states", "detectedWord", "isOn", "actualSpeed", "encoderCount",
                   "outputFrequency", "motorSpeed", "risingEdges"}


@dataclass
class LibraryEntry:
    name: str
    kind: str
    model: dict = field(default_factory=dict)  # model fields without id/runtime state, x = y = 0
    folder: str = ""  # where it sits in the library tree, e.g. "Merges/45 deg" ("" = top level)

    def instantiate(self, new_id: str, x: float = 0.0, y: float = 0.0) -> BaseModel:
        """A fresh component of this template with `new_id` at (x, y)."""
        data = dict(self.model)
        data.update(id=new_id, x=x, y=y)
        return MODEL_FOR_KIND[self.kind].model_validate(data)

    def to_json(self) -> dict:
        data = {"name": self.name, "kind": self.kind, "model": self.model}
        if self.folder:
            data["folder"] = self.folder
        return data


def kind_of(model: BaseModel) -> str:
    kind = KIND_FOR_MODEL.get(type(model))
    if kind is None:
        raise ValueError(f"{type(model).__name__} can't be a library component.")
    return kind


def entry_from_model(name: str, model: BaseModel, kind: Optional[str] = None) -> LibraryEntry:
    """Snapshot a placed component as a template (id, position and runtime
    state dropped; everything else - size, rotation, type, path, colour,
    speed, bindings - kept)."""
    name = name.strip()
    if not name:
        raise ValueError("A library component needs a name.")
    kind = kind or kind_of(model)
    data = model.model_dump(mode="json", exclude=_RUNTIME_FIELDS)
    data["x"] = 0.0
    data["y"] = 0.0
    entry = LibraryEntry(name, kind, data)
    entry.instantiate("check")  # fails now, not when it is placed, if the snapshot is unusable
    return entry


def clean_folder(folder: str) -> str:
    """'a / b//c ' -> 'a/b/c'."""
    return "/".join(part.strip() for part in folder.replace("\\", "/").split("/") if part.strip())


def unique_name(name: str, taken: Iterable[str]) -> str:
    taken = set(taken)
    if name not in taken:
        return name
    n = 2
    while f"{name} ({n})" in taken:
        n += 1
    return f"{name} ({n})"


def save_library(path: str, entries: Iterable[LibraryEntry]) -> None:
    data = {"format": FORMAT, "version": VERSION, "components": [e.to_json() for e in entries]}
    folder = os.path.dirname(os.path.abspath(path))
    os.makedirs(folder, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)  # never leave a half-written library behind


def load_library(path: str) -> Tuple[List[LibraryEntry], List[str]]:
    """(entries, problems): every valid component in the file, and one
    message per entry that was skipped. Raises ValueError if the file is
    not a component library at all."""
    with open(path, "r", encoding="utf-8") as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path} is not valid JSON: {exc}") from None
    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise ValueError(f"{path} is not a component library (expected \"format\": \"{FORMAT}\").")
    if int(data.get("version", 0)) > VERSION:
        raise ValueError(f"{path} was written by a newer version (library version {data.get('version')}).")
    entries, problems = [], []
    for i, raw in enumerate(data.get("components", []), 1):
        try:
            name, kind, model = str(raw["name"]), str(raw["kind"]), dict(raw["model"])
            if kind not in MODEL_FOR_KIND:
                raise ValueError(f"unknown kind '{kind}'")
            entry = LibraryEntry(name, kind, model, clean_folder(str(raw.get("folder", "") or "")))
            entry.instantiate("check")
        except (KeyError, TypeError, ValueError, ValidationError) as exc:
            label = raw.get("name", f"#{i}") if isinstance(raw, dict) else f"#{i}"
            problems.append(f"component {label}: {str(exc).splitlines()[0]}")
            continue
        entries.append(entry)
    return entries, problems


def default_library_path() -> str:
    """The user's own library: $CONVEYOR_COMPONENT_LIBRARY, else
    ~/.conveyor_sim/component_library.json."""
    env = os.environ.get("CONVEYOR_COMPONENT_LIBRARY")
    if env:
        return env
    return os.path.join(os.path.expanduser("~"), ".conveyor_sim", "component_library.json")
