#!/usr/bin/env python3

import rospy


def main(args=None):
    rospy.init_node('example_node')
    
    print("Write new node here!")

    rospy.spin()


if __name__ == '__main__':
    main()
