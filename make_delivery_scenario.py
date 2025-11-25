#!/usr/bin/env python3
import json
from stakeholders import OperatorMissionPlanner, RegulatorPolicy

def main():
    # Rough coordinates near KSQL (you can tune these)
    home = (37.5120, -122.2500)      # launch pad
    drop = (37.5140, -122.2460)      # delivery destination

    planner = OperatorMissionPlanner("Amazon")  # will get lane e.g. 100m
    waypoints = planner.plan_mission(home, drop)

    # Regulator check: 120 m ceiling + 10 m/s, geofence around the runway
    reg = RegulatorPolicy(max_altitude_m=120.0, max_speed_m_s=10.0)
    reg.add_geofence("KSQL core", center=home, radius_m=500.0)
    violations = reg.check_mission(waypoints)
    if violations:
        print("Regulator violations, adjust waypoints first:")
        for v in violations:
            print(" -", v)
        return

    actions = [
        {"t": 0.0, "cmd": "arm"},
        {"t": 3.0, "cmd": "takeoff", "alt": 30.0},
    ]
    t = 10.0
    for (lat, lon, alt) in waypoints:
        actions.append(
            {"t": t, "cmd": "goto_abs", "lat": lat, "lon": lon, "alt": alt}
        )
        t += 10.0
    actions.append({"t": t + 5.0, "cmd": "land"})

    scenario = [
        {
            "name": "amazon_delivery_ksql",
            "instance": 1,
            "actions": actions,
            "expect": {"final_mode": "LAND"},
        }
    ]
    print(json.dumps(scenario, indent=2))

if __name__ == "__main__":
    main()
