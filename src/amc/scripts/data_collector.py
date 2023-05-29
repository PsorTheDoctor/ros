#!/usr/bin/env python3
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import argparse
import sys

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import numpy as np


def parse_args(args):
    """Parse command line parameters

    Args:
      args ([str]): command line parameters as list of strings

    Returns:
      :obj:`argparse.Namespace`: command line parameters namespace
    """
    parser = argparse.ArgumentParser(
        description="Record data example")
    parser.add_argument(
        "-ip",
        "--robot_ip",
        dest="ip",
        help="IP address of the UR robot",
        type=str,
        default='192.168.1.131',
        metavar="<IP address of the UR robot>")
    parser.add_argument(
        "-o",
        "--output",
        dest="output",
        help="data output (.csv) file to write to (default is \"data.csv\"",
        type=str,
        default="data.csv",
        metavar="<data output file>")
    parser.add_argument(
        "-f",
        "--frequency",
        dest="frequency",
        help="the frequency at which the data is recorded (default is 500Hz)",
        type=float,
        default=500.0,
        metavar="<frequency>")

    return parser.parse_args(args)


rtde_frequency = 500.0
flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002
robot_ip = "192.168.1.131"
lookahead_time = 0.1
gain = 100
rt_receive_priority = 90
rt_control_priority = 85
#rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)


def main(args):
    """Main entry point allowing external calls

    Args:
      args ([str]): command line parameter list
    """
    args = parse_args(args)
    dt = 1 / args.frequency
    rtde_r = RTDEReceive(args.ip, args.frequency)
    rtde_r.startFileRecording(args.output)
    print("Data recording started, press [Ctrl-C] to end recording.")
    i = 0
    try:
        while True:
            start = time.time()
            pos = rtde_r.getActualTCPPose()[0:3]
            velocity = rtde_r.getActualTCPSpeed()[0:3]
            pos = np.array(pos)
            velocity = np.array(velocity)
            print(velocity)
            spring = 5
            damper = 160
            mass = 1.45
            actual_force = rtde_r.getActualTCPForce()[0:3]
            actual_velocity = rtde_r.getActualTCPSpeed()[3:6]
            actual_pos = rtde_r.getActualTCPPose()[3:6]
            print((- damper * velocity + actual_force)/mass)
            acceleration = (- damper * velocity + actual_force) / mass

            velocity = velocity + acceleration * dt
            pos = pos + velocity * dt
            # rtde_c.servoL([pos[0],pos[1],pos[2],actual_pos[0],actual_pos[1],actual_pos[2]], 0.5, 0.5, 0.002, 0.1, 300)
            rtde_c.speedL([velocity[0], velocity[1], velocity[2], actual_velocity[0], actual_velocity[1],actual_velocity[2]], 0.9, 0.002)
            if i % 10 == 0:
                sys.stdout.write("\r")
                sys.stdout.write("{:3d} samples.".format(i))
                sys.stdout.flush()
            end = time.time()
            duration = end - start

            if duration < dt:
                time.sleep(dt - duration)
            i += 1

    except KeyboardInterrupt:
        rtde_r.stopFileRecording()
        rtde_c.speedStop()
        rtde_c.servoStop()
        rtde_c.stopScript()
        rtde_r.stopFileRecording()
        print("\nData recording stopped.")


if __name__ == "__main__":
    main(sys.argv[1:])
