#!/usr/bin/env python3

import sys
import subprocess

def print_usage():
    print("Usage: straight_line.py <distance> <num_waypoints>")
    print("  distance:      Total distance to travel in meters")
    print("  num_waypoints: Number of waypoints to generate")
    print("Example: straight_line.py 5.0 10")

def main():
    if len(sys.argv) < 3:
        print_usage()
        sys.exit(1)
    
    try:
        distance = float(sys.argv[1])
        num_waypoints = int(sys.argv[2])
    except ValueError:
        print("Error: distance must be a number and num_waypoints must be an integer")
        print_usage()
        sys.exit(1)
    
    print(f"Sending robot on a straight path for {distance}m using {num_waypoints} waypoints")
    
    # Launch waypoint follower with specified parameters
    cmd = [
        "ros2", "launch", "clearo", "straight_line_waypoints.launch.py",
        f"distance:={distance}",
        f"num_waypoints:={num_waypoints}"
    ]
    
    subprocess.run(cmd)

if __name__ == "__main__":
    main()