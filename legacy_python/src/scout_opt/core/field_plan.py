from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from scout_opt.core.models import FieldMap, OperationZone, Point


@dataclass(frozen=True)
class PlannedOperationZone:
    id: int
    name: str
    center: Point
    area_hectares: float
    risk: float = 0.0
    notes: list[str] | None = None


def apply_manual_field_plan(field: FieldMap, zones: list[PlannedOperationZone]) -> None:
    if not zones:
        return
    field.operation_zones = [
        OperationZone(
            id=zone.id,
            name=zone.name,
            center=zone.center,
            area_hectares=zone.area_hectares,
            scanned=False,
            risk=zone.risk,
            notes=list(zone.notes or []),
        )
        for zone in zones
    ]
    field.area_hectares = sum(zone.area_hectares for zone in zones)


def load_manual_field_plan(path: str | Path) -> list[PlannedOperationZone]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    zones_data = data.get("zones", data if isinstance(data, list) else [])
    return [_zone_from_dict(item) for item in zones_data]


def _zone_from_dict(data: dict[str, Any]) -> PlannedOperationZone:
    center = data["center"]
    return PlannedOperationZone(
        id=int(data["id"]),
        name=str(data.get("name", f"zone-{data['id']}")),
        center=Point(float(center["x"]), float(center["y"])),
        area_hectares=float(data["area_hectares"]),
        risk=float(data.get("risk", 0.0)),
        notes=[str(note) for note in data.get("notes", [])],
    )
