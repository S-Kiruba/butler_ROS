#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal_pose():

    rospy.init_node('goal_pose_publisher', anonymous=True)

    goal_pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.sleep(1)

    goal_pose = PoseStamped()

    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = "map"

    # Define the goal position (x, y, z) in meters
    goal_pose.pose.position.x = 2.0  # Change to the desired x-coordinate
    goal_pose.pose.position.y = 3.0  # Change to the desired y-coordinate
    goal_pose.pose.position.z = 0.0  # Typically 0 in a 2D plane


    from tf.transformations import quaternion_from_euler
    yaw = 1.57
    quaternion = quaternion_from_euler(0, 0, yaw)
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo("Publishing goal pose...")
    goal_pose_pub.publish(goal_pose)

    rospy.loginfo("Goal pose published successfully!")
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_goal_pose()
    except rospy.ROSInterruptException:
        pass
