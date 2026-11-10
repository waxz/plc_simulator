"""
component_library.py - reusable component templates, stored as data.

A library entry is a named snapshot of one component (any kind: a belt of
any type/shape, a sensor, a machine, a control panel, ...) without its id and
position. Placing it builds a fresh model with a new id at the drop point;
path / polygon belts keep their geometry because it is stored relative to
the belt's x/y.

A station entry (kind "station") is a group of components saved as one: a
turntable with its platform belt, an accumulation line with its zones, a
safety relay with its e-stops and doors. Its members keep their ids and
their links to each other (REFERENCE_FIELDS) and the belt connections
between them; placing it gives every member a fresh id and points the links
to the new copies (LibraryEntry.place_station).

Libraries are JSON files, so they can be shared between layouts and people:

    {
      "format": "conveyor-component-library",
      "version": 1,
      "components": [
        {"name": "Merge 45", "kind": "belt", "folder": "Merges", "model": {...}},
        {"name": "Pick cell", "kind": "station", "folder": "Stations",
         "components": [{"kind": "belt", "model": {...}}, ...], "connections": [...]},
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
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from pydantic import BaseModel, ValidationError

from config_parser import (Accumulator, Belt, BeltConnection, ControlButton, ControlPanel, Diverter, Elevator,
                           GuardDoor, LedIndicator, Pallet, RobotArm, SafetyCircuit, Scanner, Sensor, Sink, Source,
                           Stopper, Transfer, Turntable)

FORMAT = "conveyor-component-library"
VERSION = 1

MODEL_FOR_KIND: Dict[str, type] = {
    "belt": Belt,
    "sensor": Sensor,
    "source": Source,
    "sink": Sink,
    "button": ControlButton,
    "led": LedIndicator,
    "pallet": Pallet,
    "stopper": Stopper,
    "robot": RobotArm,
    "elevator": Elevator,
    "scanner": Scanner,
    "diverter": Diverter,
    "turntable": Turntable,
    "transfer": Transfer,
    "accumulator": Accumulator,
    "door": GuardDoor,
    "safety": SafetyCircuit,
    "panel": ControlPanel,
}
KIND_FOR_MODEL = {cls: kind for kind, cls in MODEL_FOR_KIND.items()}
STATION = "station"

# Runtime state that must not travel with a template.
_RUNTIME_FIELDS = {"id", "isActive", "sensor_states", "detectedWord", "isOn", "actualSpeed", "encoderCount",
                   "outputFrequency", "motorSpeed", "risingEdges", "liftVelocity", "turnVelocity"}

# The fields that name other components: kind -> {field: the kind it names}.
# Ids are unique per kind, so a link is (kind, id).
REFERENCE_FIELDS: Dict[str, Dict[str, str]] = {
    "robot": {"pickSensorId": "sensor"},
    "elevator": {"beltId": "belt"},
    "turntable": {"beltId": "belt"},
    "accumulator": {"beltIds": "belt"},
    "safety": {"stopButtonIds": "button", "doorIds": "door", "curtainIds": "sensor", "beltIds": "belt",
               "resetButtonId": "button"},
}


def shift_model(model, dx: float, dy: float) -> None:
    """Move a component by (dx, dy) - a robot's pick / place points too
    (everything else is stored relative to x/y)."""
    model.x += dx
    model.y += dy
    if isinstance(model, RobotArm):
        model.pickX += dx
        model.pickY += dy
        model.placeX += dx
        model.placeY += dy


def remap_references(kind: str, model, id_map: Dict[Tuple[str, str], str], keep_outside: bool = True) -> List[str]:
    """Point `model`'s links (REFERENCE_FIELDS) through id_map, (kind, old
    id) -> new id. A link to a component not in the map stays as it is
    (keep_outside) or is dropped. Returns the dropped links as "field -> id"."""
    dropped = []
    for name, target in REFERENCE_FIELDS.get(kind, {}).items():
        value = getattr(model, name)
        if value is None:
            continue
        values = value if isinstance(value, list) else [value]
        kept = []
        for v in values:
            if (target, v) in id_map:
                kept.append(id_map[(target, v)])
            elif keep_outside:
                kept.append(v)
            else:
                dropped.append(f"{name} -> {v}")
        setattr(model, name, kept if isinstance(value, list) else (kept[0] if kept else None))
    return dropped


@dataclass
class LibraryEntry:
    name: str
    kind: str
    model: dict = field(default_factory=dict)  # model fields without id/runtime state, x = y = 0
    folder: str = ""  # where it sits in the library tree, e.g. "Merges/45 deg" ("" = top level)
    # A station: its components [{"kind", "model"}] - ids kept, positions
    # relative to the group's centre - and the belt connections between them.
    members: List[dict] = field(default_factory=list)
    connections: List[dict] = field(default_factory=list)

    @property
    def is_station(self) -> bool:
        return self.kind == STATION

    def instantiate(self, new_id: str, x: float = 0.0, y: float = 0.0) -> BaseModel:
        """A fresh component of this template with `new_id` at (x, y)."""
        if self.is_station:
            raise ValueError(f"'{self.name}' is a station: place it with place_station().")
        data = dict(self.model)
        data["id"] = new_id
        model = MODEL_FOR_KIND[self.kind].model_validate(data)
        shift_model(model, x - model.x, y - model.y)
        return model

    def place_station(self, new_id: Callable[[str, str], str], x: float = 0.0, y: float = 0.0
                      ) -> Tuple[List[Tuple[str, BaseModel]], List[BeltConnection]]:
        """A fresh copy of the station with its centre at (x, y): ([(kind,
        model)], connections). new_id(kind, old id) gives each member its id;
        the links and connections point to the new copies."""
        id_map = {}
        for member in self.members:
            key = (member["kind"], member["model"]["id"])
            id_map[key] = new_id(*key)
        placed = []
        for member in self.members:
            kind = member["kind"]
            model = MODEL_FOR_KIND[kind].model_validate(member["model"])
            model.id = id_map[(kind, model.id)]
            remap_references(kind, model, id_map, keep_outside=False)
            shift_model(model, x, y)
            placed.append((kind, model))
        connections = [BeltConnection(fromBelt=id_map[("belt", c["fromBelt"])], toBelt=id_map[("belt", c["toBelt"])])
                       for c in self.connections]
        return placed, connections

    def check(self) -> None:
        """Raise now, not when it is placed, if the entry can't be built."""
        if not self.is_station:
            self.instantiate("check")
            return
        if not self.members:
            raise ValueError("a station without components")
        ids = [(m["kind"], m["model"].get("id")) for m in self.members]
        if len(set(ids)) != len(ids):
            raise ValueError("two station components with the same id")
        for c in self.connections:
            if ("belt", c.get("fromBelt")) not in ids or ("belt", c.get("toBelt")) not in ids:
                raise ValueError(f"connection {c} names a belt that is not in the station")
        self.place_station(lambda kind, old: old)

    def to_json(self) -> dict:
        data = {"name": self.name, "kind": self.kind}
        if self.is_station:
            data.update(components=self.members, connections=self.connections)
        else:
            data["model"] = self.model
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
    model = model.model_copy(deep=True)
    shift_model(model, -model.x, -model.y)
    data = model.model_dump(mode="json", exclude=_RUNTIME_FIELDS)
    entry = LibraryEntry(name, kind, data)
    entry.check()  # fails now, not when it is placed, if the snapshot is unusable
    return entry


def is_panel_child(kind: str, model) -> bool:
    """A control panel's button / lamp: rebuilt from its panel, never saved
    on its own."""
    return kind in ("button", "led") and bool(getattr(model, "panel", None))


def station_from_models(name: str, components: Iterable[Tuple[str, BaseModel]],
                        connections: Iterable[BeltConnection] = ()) -> Tuple[LibraryEntry, List[str]]:
    """(entry, dropped): the components as one station entry - positions
    relative to their centre, the belt connections among them - and the
    links to components outside the group, which it can't keep (as
    "<id>.field -> id")."""
    name = name.strip()
    if not name:
        raise ValueError("A library station needs a name.")
    # In a fixed order (a canvas selection has none): the same station, the same fresh ids when placed.
    components = sorted(((kind, model) for kind, model in components if not is_panel_child(kind, model)),
                        key=lambda c: (c[0], c[1].id))
    if len(components) < 2:
        raise ValueError("A station needs two or more components.")
    for kind, _ in components:
        if kind not in MODEL_FOR_KIND:
            raise ValueError(f"A {kind} can't be a library component.")
    cx = sum(m.x for _, m in components) / len(components)
    cy = sum(m.y for _, m in components) / len(components)
    inside = {(kind, m.id): m.id for kind, m in components}
    members, dropped = [], []
    for kind, model in components:
        copy = model.model_copy(deep=True)
        shift_model(copy, -cx, -cy)
        dropped += [f"{model.id}.{d}" for d in remap_references(kind, copy, inside, keep_outside=False)]
        data = copy.model_dump(mode="json", exclude=_RUNTIME_FIELDS - {"id"})
        members.append({"kind": kind, "model": data})
    belts = {i for kind, i in inside if kind == "belt"}
    links = [{"fromBelt": c.fromBelt, "toBelt": c.toBelt} for c in connections
             if c.fromBelt in belts and c.toBelt in belts]
    entry = LibraryEntry(name, STATION, {}, members=members, connections=links)
    entry.check()
    return entry, dropped


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
            name, kind = str(raw["name"]), str(raw["kind"])
            folder = clean_folder(str(raw.get("folder", "") or ""))
            if kind == STATION:
                members = [{"kind": str(m["kind"]), "model": dict(m["model"])} for m in raw["components"]]
                for m in members:
                    if m["kind"] not in MODEL_FOR_KIND:
                        raise ValueError(f"unknown kind '{m['kind']}'")
                entry = LibraryEntry(name, kind, {}, folder, members, [dict(c) for c in raw.get("connections", [])])
            else:
                if kind not in MODEL_FOR_KIND:
                    raise ValueError(f"unknown kind '{kind}'")
                entry = LibraryEntry(name, kind, dict(raw["model"]), folder)
            entry.check()
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
