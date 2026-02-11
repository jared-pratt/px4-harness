# operator_mission_planner.py

from typing import List, Tuple, ClassVar

class OperatorMissionPlanner:
    """
    Mission planner for drone operators that assigns routes with operator-specific altitude lanes.
    """
    # Class variable to keep track of assigned altitude lanes for each operator (to avoid conflicts)
    assigned_altitudes: ClassVar[dict] = {}

    def __init__(self, operator_name: str, default_altitude_m: float = 100.0, altitude_increment: float = 20.0):
        """
        :param operator_name: Name/identifier of the operator (e.g., "Amazon", "Walmart").
        :param default_altitude_m: Base altitude (in meters) for the first operator if not already assigned.
        :param altitude_increment: Increment (m) for altitude lanes when assigning new operators.
        """
        self.operator_name = operator_name
        # Assign a unique altitude lane for this operator if not already assigned
        if operator_name in OperatorMissionPlanner.assigned_altitudes:
            self.cruise_altitude = OperatorMissionPlanner.assigned_altitudes[operator_name]
        else:
            # If not assigned, choose the next available altitude (base or base+increments)
            if not OperatorMissionPlanner.assigned_altitudes:
                # No operators yet, use default_altitude_m for this operator
                self.cruise_altitude = default_altitude_m
            else:
                # Calculate a new altitude that is different from all existing ones by at least altitude_increment
                max_assigned = max(OperatorMissionPlanner.assigned_altitudes.values())
                self.cruise_altitude = max_assigned + altitude_increment
            OperatorMissionPlanner.assigned_altitudes[operator_name] = self.cruise_altitude

    def plan_mission(self, start: Tuple[float, float], end: Tuple[float, float]) -> List[Tuple[float, float, float]]:
        """
        Generate a mission plan from start to end coordinates using the operator's altitude lane.
        :param start: (lat, lon) of launch/origin point.
        :param end: (lat, lon) of destination point (delivery location).
        :return: List of waypoints (lat, lon, alt) for the mission.
        """
        lat1, lon1 = start
        lat2, lon2 = end
        alt = self.cruise_altitude
        waypoints: List[Tuple[float, float, float]] = []
        # Takeoff: ascend from ground to cruise altitude at the launch point
        waypoints.append((lat1, lon1, alt))
        # Cruise: fly from launch point to destination at cruise altitude
        waypoints.append((lat2, lon2, alt))
        # Descent: descend near the destination (e.g., to 5m for delivery drop or landing)
        waypoints.append((lat2, lon2, 5.0))
        return waypoints

    @classmethod
    def get_assigned_altitude(cls, operator_name: str) -> float:
        """Get the altitude lane assigned to a given operator (if any)."""
        return cls.assigned_altitudes.get(operator_name)

# Example usage:
# planner1 = OperatorMissionPlanner("Amazon")
# mission1 = planner1.plan_mission((37.5100, -122.2600), (37.5200, -122.2400))
# print("Amazon mission waypoints:", mission1)
# # Amazon might be assigned the default altitude, e.g., 100m.
# planner2 = OperatorMissionPlanner("Walmart")
# mission2 = planner2.plan_mission((37.5090, -122.2610), (37.5210, -122.2390))
# print("Walmart mission waypoints:", mission2)
# # Walmart would get a different altitude lane, e.g., 120m if using 20m increments.
# # We can verify the assigned altitudes:
# print("Assigned altitudes:", OperatorMissionPlanner.assigned_altitudes)
