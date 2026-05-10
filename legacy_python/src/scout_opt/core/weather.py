from __future__ import annotations

import math
from dataclasses import dataclass

from scout_opt.core.models import DroneSpec, Point, WeatherSeverity, WeatherSnapshot


WEATHER_UPDATE_INTERVAL_SECONDS = 120.0


@dataclass(frozen=True)
class WeatherAdjustedSpec:
    scout_rate_hectares_per_hour: float
    spray_rate_hectares_per_hour: float
    cruise_speed_mps: float
    battery_work_multiplier: float
    battery_scout_multiplier: float
    spray_effectiveness: float
    spray_allowed: bool
    flight_allowed: bool


class MothershipWeatherStation:
    def __init__(self, interval_seconds: float = WEATHER_UPDATE_INTERVAL_SECONDS) -> None:
        self.base_interval_seconds = interval_seconds
        self.interval_seconds = interval_seconds
        self.last_update_seconds = -interval_seconds

    def should_update(self, now_seconds: float) -> bool:
        return now_seconds - self.last_update_seconds >= self.interval_seconds

    def update(self, now_seconds: float) -> WeatherSnapshot:
        self.last_update_seconds = now_seconds
        # Deterministic wave pattern for local simulation. Real SITL integration
        # can replace this provider with telemetry from a weather station.
        wind = 2.8 + 1.4 * math.sin(now_seconds / 900.0)
        gust = wind + 1.8 + 0.8 * math.sin(now_seconds / 420.0)
        humidity = 0.55 + 0.12 * math.sin(now_seconds / 1800.0)
        precipitation = 0.0
        snapshot = WeatherSnapshot(
            wind_speed_mps=max(0.0, wind),
            wind_gust_mps=max(wind, gust),
            temperature_c=26.0 + 2.0 * math.sin(now_seconds / 2400.0),
            humidity=max(0.2, min(0.95, humidity)),
            precipitation_mmph=precipitation,
            visibility_m=5000.0,
            updated_at_seconds=now_seconds,
        )
        self.interval_seconds = update_interval_for_severity(snapshot.severity, self.base_interval_seconds)
        return snapshot


def adjust_spec_for_weather(spec: DroneSpec, weather: WeatherSnapshot) -> WeatherAdjustedSpec:
    wind_penalty = min(0.45, max(0.0, weather.wind_speed_mps - 2.0) * 0.055)
    gust_penalty = min(0.25, max(0.0, weather.wind_gust_mps - 5.0) * 0.04)
    humidity_bonus = 0.04 if 0.45 <= weather.humidity <= 0.75 else -0.04
    spray_factor = max(0.35, min(1.05, 1.0 - wind_penalty - gust_penalty + humidity_bonus))
    cruise_factor = max(0.55, min(1.0, 1.0 - wind_penalty * 0.55 - gust_penalty * 0.35))
    battery_multiplier = 1.0 + wind_penalty * 0.9 + gust_penalty * 0.5
    rain_penalty = min(0.7, weather.precipitation_mmph * 0.18)
    humidity_penalty = max(0.0, weather.humidity - 0.82) * 0.8
    spray_effectiveness = max(0.0, min(1.0, spray_factor - rain_penalty - humidity_penalty))

    if not weather.spray_allowed:
        spray_factor = 0.0
        spray_effectiveness = 0.0
    if not weather.flight_allowed:
        cruise_factor = 0.0

    return WeatherAdjustedSpec(
        scout_rate_hectares_per_hour=spec.scout_rate_hectares_per_hour * max(0.3, cruise_factor),
        spray_rate_hectares_per_hour=spec.spray_rate_hectares_per_hour * spray_factor,
        cruise_speed_mps=spec.cruise_speed_mps * cruise_factor,
        battery_work_multiplier=battery_multiplier,
        battery_scout_multiplier=1.0 + wind_penalty * 0.6,
        spray_effectiveness=spray_effectiveness,
        spray_allowed=weather.spray_allowed,
        flight_allowed=weather.flight_allowed,
    )


def update_interval_for_severity(
    severity: WeatherSeverity,
    base_interval_seconds: float = WEATHER_UPDATE_INTERVAL_SECONDS,
) -> float:
    if severity == WeatherSeverity.EMERGENCY:
        return 15.0
    if severity == WeatherSeverity.SEVERE:
        return 30.0
    if severity == WeatherSeverity.WARNING:
        return 60.0
    if severity == WeatherSeverity.WATCH:
        return 90.0
    return base_interval_seconds


def design_emergency_landing_spots(
    center: Point,
    drone_count: int,
    spacing_m: float = 22.0,
) -> list[Point]:
    cols = max(1, math.ceil(math.sqrt(drone_count)))
    rows = max(1, math.ceil(drone_count / cols))
    spots: list[Point] = []
    for idx in range(drone_count):
        row = idx // cols
        col = idx % cols
        spots.append(
            Point(
                x=center.x + (col - (cols - 1) / 2.0) * spacing_m,
                y=center.y + (row - (rows - 1) / 2.0) * spacing_m,
            )
        )
    return spots
