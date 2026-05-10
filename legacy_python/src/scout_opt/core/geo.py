from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi

from scout_opt.core.models import Point


@dataclass(frozen=True)
class GeoOrigin:
    latitude_deg: float
    longitude_deg: float


def local_to_latlon(point: Point, origin: GeoOrigin) -> tuple[float, float]:
    meters_per_degree_lat = 111_320.0
    meters_per_degree_lon = meters_per_degree_lat * cos(origin.latitude_deg * pi / 180.0)
    latitude = origin.latitude_deg + point.y / meters_per_degree_lat
    longitude = origin.longitude_deg + point.x / meters_per_degree_lon
    return latitude, longitude


def latlon_to_local(latitude_deg: float, longitude_deg: float, origin: GeoOrigin) -> Point:
    meters_per_degree_lat = 111_320.0
    meters_per_degree_lon = meters_per_degree_lat * cos(origin.latitude_deg * pi / 180.0)
    return Point(
        x=(longitude_deg - origin.longitude_deg) * meters_per_degree_lon,
        y=(latitude_deg - origin.latitude_deg) * meters_per_degree_lat,
    )
