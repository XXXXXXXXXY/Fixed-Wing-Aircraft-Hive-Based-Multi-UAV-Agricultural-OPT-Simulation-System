from __future__ import annotations

import unittest

from scout_opt.core.config import default_simulation_config
from scout_opt.core.charging import ChargingQueue
from scout_opt.core.engine import SimulationEngine
from scout_opt.core.models import DroneState, Point, WeatherSeverity, WeatherSnapshot
from scout_opt.core.scheduler import MissionScheduler
from scout_opt.core.weather import (
    adjust_spec_for_weather,
    design_emergency_landing_spots,
    update_interval_for_severity,
)


class WeatherTests(unittest.TestCase):
    def test_weather_updates_every_two_minutes(self) -> None:
        config = default_simulation_config()
        engine = SimulationEngine(config, dt_seconds=60.0)
        for _ in range(5):
            engine.step()

        weather_events = [event for event in engine.events if event.kind == "weather"]
        self.assertEqual([event.time_seconds for event in weather_events[:3]], [0.0, 120.0, 240.0])

    def test_weather_changes_spray_and_flight_speed(self) -> None:
        config = default_simulation_config()
        calm = adjust_spec_for_weather(config.drone_spec, WeatherSnapshot(wind_speed_mps=2.0, wind_gust_mps=3.0))
        windy = adjust_spec_for_weather(config.drone_spec, WeatherSnapshot(wind_speed_mps=6.0, wind_gust_mps=8.0))

        self.assertLess(windy.spray_rate_hectares_per_hour, calm.spray_rate_hectares_per_hour)
        self.assertLess(windy.cruise_speed_mps, calm.cruise_speed_mps)
        self.assertGreater(windy.battery_work_multiplier, calm.battery_work_multiplier)

    def test_weather_severity_changes_update_interval(self) -> None:
        self.assertEqual(update_interval_for_severity(WeatherSeverity.NORMAL), 120.0)
        self.assertEqual(update_interval_for_severity(WeatherSeverity.WARNING), 60.0)
        self.assertEqual(update_interval_for_severity(WeatherSeverity.SEVERE), 30.0)
        self.assertEqual(update_interval_for_severity(WeatherSeverity.EMERGENCY), 15.0)

    def test_emergency_landing_spots_are_separated(self) -> None:
        spots = design_emergency_landing_spots(Point(0.0, 0.0), drone_count=8, spacing_m=22.0)

        self.assertEqual(len(spots), 8)
        self.assertGreaterEqual(spots[0].distance_to(spots[1]), 22.0)

    def test_severe_weather_recovers_active_drones(self) -> None:
        config = default_simulation_config()
        config.field.scanned = True
        config.mothership.weather = WeatherSnapshot(wind_gust_mps=14.0, precipitation_mmph=2.5)
        for idx, drone in enumerate(config.drones[:3]):
            drone.state = DroneState.WORKING
            drone.position = Point(100.0 + idx * 50.0, 0.0)

        scheduler = MissionScheduler(
            ChargingQueue(
                fast_chargers=2,
                refill_ports=2,
                charging_slots=[None, None],
                refill_slots=[None, None],
            )
        )
        events = scheduler.step(
            now_seconds=0.0,
            dt_seconds=60.0,
            field=config.field,
            mothership=config.mothership,
            drones=config.drones,
            spec=config.drone_spec,
        )

        self.assertTrue(any(event.kind == "emergency" for event in events))
        self.assertTrue(all(drone.state == DroneState.RETURNING for drone in config.drones[:3]))

    def test_emergency_weather_assigns_landing_spots(self) -> None:
        config = default_simulation_config()
        config.field.scanned = True
        config.mothership.weather = WeatherSnapshot(wind_gust_mps=17.0, precipitation_mmph=6.0)
        for idx, drone in enumerate(config.drones[:3]):
            drone.state = DroneState.WORKING
            drone.position = Point(600.0 + idx * 60.0, 0.0)
            drone.battery = 0.4

        scheduler = MissionScheduler(
            ChargingQueue(
                fast_chargers=2,
                refill_ports=2,
                charging_slots=[None, None],
                refill_slots=[None, None],
            )
        )
        scheduler.step(
            now_seconds=0.0,
            dt_seconds=60.0,
            field=config.field,
            mothership=config.mothership,
            drones=config.drones,
            spec=config.drone_spec,
        )

        self.assertTrue(config.field.emergency_landing_spots)
        self.assertTrue(all(drone.state == DroneState.EMERGENCY_LANDING for drone in config.drones[:3]))
        self.assertEqual(len({drone.target for drone in config.drones[:3]}), 3)


if __name__ == "__main__":
    unittest.main()
