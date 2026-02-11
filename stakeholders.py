# stakeholders.py
from __future__ import annotations
from typing import List, Tuple, Dict, ClassVar, Optional
import math

# ---------- Regulators ----------

class Geofence:
    def __init__(self, name: str, center: Tuple[float, float], radius_m: float):
        self.name = name
        self.center = center      # (lat, lon)
        self.radius_m = radius_m

    def contains(self, lat: float, lon: float) -> bool:
        R = 6371000.0
        lat1 = math.radians(self.center[0])
        lon1 = math.radians(self.center[1])
        lat2 = math.radians(lat)
        lon2 = math.radians(lon)
        x = (lon2 - lon1) * math.cos(0.5 * (lat2 + lat1))
        y = (lat2 - lat1)
        d = R * math.sqrt(x * x + y * y)
        return d <= self.radius_m


class RegulatorPolicy:
    def __init__(self, max_altitude_m: float = 120.0, max_speed_m_s: float = 10.0):
        self.max_altitude_m = max_altitude_m
        self.max_speed_m_s = max_speed_m_s
        self.geofences: List[Geofence] = []

    def add_geofence(self, name: str, center: Tuple[float, float], radius_m: float):
        self.geofences.append(Geofence(name, center, radius_m))

    def check_mission(self, waypoints: List[Tuple[float, float, float]]) -> List[str]:
        msgs: List[str] = []
        for i, (lat, lon, alt) in enumerate(waypoints):
            if alt > self.max_altitude_m:
                msgs.append(
                    f"Waypoint {i}: alt {alt:.1f}m > max {self.max_altitude_m}m"
                )
            for gf in self.geofences:
                if gf.contains(lat, lon):
                    msgs.append(
                        f"Waypoint {i}: inside geofence '{gf.name}'"
                    )
        return msgs

    def check_inflight(self, lat: float, lon: float, alt: float, speed: float) -> List[str]:
        msgs: List[str] = []
        if alt > self.max_altitude_m:
            msgs.append(f"Alt {alt:.1f}m > max {self.max_altitude_m}m")
        if speed > self.max_speed_m_s:
            msgs.append(f"Speed {speed:.1f} m/s > max {self.max_speed_m_s} m/s")
        for gf in self.geofences:
            if gf.contains(lat, lon):
                msgs.append(f"Inside geofence '{gf.name}'")
        return msgs


# ---------- Vendors ----------

class VendorSafetyPolicy:
    def __init__(
        self,
        drone: Optional[object] = None,
        battery_drop_threshold: float = 15.0,
        low_battery_threshold: float = 20.0,
    ):
        self.drone = drone
        self.battery_drop_threshold = battery_drop_threshold
        self.low_battery_threshold = low_battery_threshold
        self._last_batt: Optional[float] = None
        self._parachute_deployed = False

    def _deploy_parachute(self, log):
        if self._parachute_deployed:
            return
        self._parachute_deployed = True
        log("VENDOR: *** Deploying parachute ***")
        # If you wire this to real parachute logic, call MAVSDK / MAVLink here.

    def handle_motor_failure(self, log):
        log("VENDOR: Motor failure detected!")
        self._deploy_parachute(log)

    def check_battery(self, batt_percent: float, log):
        if self._last_batt is not None:
            drop = self._last_batt - batt_percent
            if drop >= self.battery_drop_threshold and batt_percent < 90.0:
                log(
                    f"VENDOR: Battery dropped {drop:.1f}% quickly (now {batt_percent:.1f}%)"
                )
                self._deploy_parachute(log)

        if batt_percent <= self.low_battery_threshold:
            log(
                f"VENDOR: Battery low ({batt_percent:.1f}%), requesting Return-to-Home"
            )
            if self.drone:
                # In async context you’d: await self.drone.action.return_to_launch()
                # Here we just log; actual RTL should be wired by you.
                pass

        self._last_batt = batt_percent


# ---------- Operators ----------

class OperatorMissionPlanner:
    assigned_altitudes: ClassVar[Dict[str, float]] = {}

    def __init__(self, operator_name: str, base_alt_m: float = 100.0, lane_step_m: float = 20.0):
        self.operator_name = operator_name
        if operator_name in OperatorMissionPlanner.assigned_altitudes:
            self.cruise_alt = OperatorMissionPlanner.assigned_altitudes[operator_name]
        else:
            if not OperatorMissionPlanner.assigned_altitudes:
                self.cruise_alt = base_alt_m
            else:
                self.cruise_alt = max(OperatorMissionPlanner.assigned_altitudes.values()) + lane_step_m
            OperatorMissionPlanner.assigned_altitudes[operator_name] = self.cruise_alt

    def plan_mission(
        self, start: Tuple[float, float], end: Tuple[float, float]
    ) -> List[Tuple[float, float, float]]:
        lat1, lon1 = start
        lat2, lon2 = end
        alt = self.cruise_alt
        # Simple: up to lane, straight to target, down to 5 m
        return [
            (lat1, lon1, alt),
            (lat2, lon2, alt),
            (lat2, lon2, 5.0),
        ]
