#! /usr/bin/env python3
# -*- coding:utf-8 -*-
from p1_robcom import *
import sys

def main(args):
    robo = Robot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)