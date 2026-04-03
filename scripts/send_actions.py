#!/usr/bin/env python3
import subprocess

STRIKE_CMD = [
    "ros2", "action", "send_goal",
    "/strike_action",
    "px4_ros2_striker/action/Strike",
    "{latitude: 47.4095637063143, longitude: 8.551258227132053, altitude: 0.0}"
]

RECOVER_CMD = [
    "ros2", "action", "send_goal",
    "/recover_action",
    "px4_ros2_striker/action/Recover",
    "{latitude: 47.397971, longitude: 8.546164, altitude: 100.0, final_mode: 'hold', arrival_radius_m: 150.0}"
]

def main():
    print("Press:")
    print("1 → strike_goal")
    print("2 → recovery_goal")
    print("q → quit")

    while True:
        key = input("> ").strip()

        if key == "1":
            print("1")
            subprocess.Popen(STRIKE_CMD)

        elif key == "2":
            print("2")
            subprocess.Popen(RECOVER_CMD)

        elif key.lower() == "q":
            break

if __name__ == "__main__":
    main()