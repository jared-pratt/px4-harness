# regulator_policy.py

from typing import List, Tuple, Optional
import math

class Geofence:
    """Represents a circular no-fly zone (geofence)."""
    def __init__(self, name: str, center: Tuple[float, float], radius_m: float):
        """
        Initialize a geofence.
        :param name: Identifier for the geofence (e.g., "Airport NFZ").
        :param center: (lat, lon) center of the geofence in decimal degrees.
        :param radius_m: Radius of the no-fly zone in meters.
        """
        self.name = name
        self.center = center  # (latitude, longitude)
        self.radius_m = radius_m

    def contains(self, lat: float, lon: float) -> bool:
        """
        Check if a given position (lat, lon) lies within this geofence.
        Uses a simple equirectangular approximation for small distances.
        """
        # Earth radius in meters
        R = 6371000.0
        # Convert degrees to radians
        lat1 = math.radians(self.center[0])
        lon1 = math.radians(self.center[1])
        lat2 = math.radians(lat)
        lon2 = math.radians(lon)
        # Equirectangular approximation (accurate for small areas)
        x = (lon2 - lon1) * math.cos(0.5 * (lat2 + lat1))
        y = (lat2 - lat1)
        distance = R * math.sqrt(x*x + y*y)
        return distance <= self.radius_m

class RegulatorPolicy:
    """Policy enforcing regulatory constraints like geofences and flight limits."""
    def __init__(self, max_altitude_m: float = 120.0, max_speed_m_s: float = 10.0):
        """
        :param max_altitude_m: Global altitude limit in meters (AGL).
        :param max_speed_m_s: Global speed limit in m/s.
        """
        self.max_altitude_m = max_altitude_m
        self.max_speed_m_s = max_speed_m_s
        self.geofences: List[Geofence] = []

    def add_geofence(self, name: str, center: Tuple[float, float], radius_m: float) -> None:
        """Add a no-fly zone defined by a circle (center lat/lon, radius in m)."""
        self.geofences.append(Geofence(name, center, radius_m))

    def check_mission(self, waypoints: List[Tuple[float, float, float]]) -> List[str]:
        """
        Check a planned mission (list of waypoints) for any regulatory violations.
        Each waypoint is a tuple (lat, lon, alt).
        :return: List of violation messages, empty if mission complies with rules.
        """
        violations: List[str] = []
        for i, (lat, lon, alt) in enumerate(waypoints):
            # Check altitude limit
            if alt > self.max_altitude_m:
                violations.append(f"Waypoint {i}: Altitude {alt:.1f}m exceeds limit of {self.max_altitude_m}m.")
            # Check geofence violations
            for gf in self.geofences:
                if gf.contains(lat, lon):
                    violations.append(f"Waypoint {i}: lies within no-fly zone '{gf.name}'.")
        return violations

    def check_inflight(self, lat: float, lon: float, alt: float, speed: float) -> List[str]:
        """
        Check a drone's current telemetry for regulatory compliance.
        :param lat: Current latitude.
        :param lon: Current longitude.
        :param alt: Current altitude (m).
        :param speed: Current speed (m/s).
        :return: List of violations detected at the current state (empty if none).
        """
        violations: List[str] = []
        # Altitude violation
        if alt > self.max_altitude_m:
            violations.append(f"In-flight altitude {alt:.1f}m exceeds limit of {self.max_altitude_m}m.")
        # Speed violation
        if speed > self.max_speed_m_s:
            violations.append(f"In-flight speed {speed:.1f} m/s exceeds limit of {self.max_speed_m_s} m/s.")
        # Geofence violation
        for gf in self.geofences:
            if gf.contains(lat, lon):
                violations.append(f"In-flight position is inside no-fly zone '{gf.name}'.")
        return violations

# Example usage (for testing or illustration):
# reg = RegulatorPolicy(max_altitude_m=120.0, max_speed_m_s=10.0)
# reg.add_geofence("Airport", center=(37.512, -122.250), radius_m=1000.0)  # e.g., 1km no-fly around KSQL airport
# mission = [(37.510, -122.260, 80.0), (37.520, -122.240, 130.0)]
# issues = reg.check_mission(mission)
# if issues:
#     print("Mission plan violations:")
#     for issue in issues:
#         print(" -", issue)
# else:
#     print("Mission plan is compliant with regulations.")
# # During flight, call reg.check_inflight(lat, lon, alt, speed) periodically to monitor compliance.
