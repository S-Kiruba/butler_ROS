#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import yaml
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult

class ButlerRobot():
    def __init__(self):
        rospy.init_node("goal_publisher")
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.Subscriber("/new_order", String, self.order_callback)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.goal_result_callback)

        # Load goals from YAML file
        self.goals = self.load_goals("/home/sk/ws/goat_robotics/src/butler/src/goals.yaml")
        
        self.task_queue = []
        self.current_task = None
        self.status = None
        
        # Initialize robot at home
        self.publish_goal("home")

    def load_goals(self, file_path):
        try:
            with open(file_path, "r") as file:
                data = yaml.safe_load(file)
            rospy.loginfo("Goals loaded successfully.")
            return data["goals"]
        except Exception as e:
            rospy.logerr(f"Failed to load goals: {e}")
            return {}

    def order_callback(self, msg):
        new_orders = msg.data.split()
        self.task_queue.extend(new_orders)
        rospy.loginfo(f"New orders received: {new_orders}")
        if self.current_task is None:
            self.handle_kitchen_task()


    def goal_result_callback(self, msg):
        self.status = msg.status.status
        if self.status == 3:  # Goal reached successfully
            rospy.loginfo("Navigation completed successfully.")
            
            if self.current_task == "kitchen":
                rospy.loginfo("Reached kitchen.")
                self.current_task = table
                self.publish_goal(table)
            
            elif self.current_task.startswith("table"):
                rospy.loginfo(f"Reached {self.current_task}.")
                self.current_task = home
                self.publish_goal(home)

    def publish_goal(self, location):
        if location not in self.goals:
            rospy.logerr(f"Unknown location: {location}")
            return False
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.goals[location]["x"]
        goal.pose.position.y = self.goals[location]["y"]
        quaternion = quaternion_from_euler(0, 0, self.goals[location]["yaw"])
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        self.goal_pub.publish(goal)
        rospy.loginfo(f"Goal published to {location}")
        return True

    def handle_kitchen_task(self):
        if not self.current_task:
            rospy.loginfo("Navigating to kitchen for order pickup.")
            self.current_task = "kitchen"
            self.publish_goal("kitchen")

if __name__ == "__main__":
    robot = ButlerRobot()