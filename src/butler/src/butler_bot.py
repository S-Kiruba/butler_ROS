#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import yaml
from std_msgs.msg import String, Bool
from move_base_msgs.msg import MoveBaseActionResult

class ButlerRobot():
    def __init__(self):
        rospy.init_node("goal_publisher")
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.Subscriber("/new_order", String, self.order_callback)
        rospy.Subscriber("/kitchen_confirm", Bool, self.kitchen_confirm_callback)
        rospy.Subscriber("/table_confirm", Bool, self.table_confirm_callback)
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

    def kitchen_confirm_callback(self, msg):
        self.kitchen_confirmation = msg.data
        rospy.loginfo(f"Kitchen confirmation received: {self.kitchen_confirmation}")

    def table_confirm_callback(self, msg):
        self.table_confirmation = msg.data
        rospy.loginfo(f"Table confirmation received: {self.table_confirmation}")

    def wait_for_confirmation(self, location, timeout=30):
        start_time = rospy.Time.now()
        rate = rospy.Rate(1)  # 1 Hz
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if location == "kitchen" and self.kitchen_confirmation:
                self.kitchen_confirmation = False
                return True
            elif location.startswith("table") and self.table_confirmation:
                self.table_confirmation = False
                return True
            rate.sleep()
        
        return False


    def goal_result_callback(self, msg):
        self.status = msg.status.status
        if self.status == 3:  # Goal reached successfully
            rospy.loginfo("Navigation completed successfully.")
            
            if self.current_task == "kitchen":
                rospy.loginfo("Reached kitchen.")
                if self.wait_for_confirmation("kitchen"):
                    rospy.loginfo("Kitchen confirmation received. Proceeding to table.")
                    if self.task_queue:
                        table = self.task_queue.pop(0)
                        self.current_task = table
                        self.publish_goal(table)
                else:
                    rospy.logwarn("No kitchen confirmation received. Returning to home.")
                    self.current_task = "home"
                    self.publish_goal("home")
            
            elif self.current_task.startswith("table"):
                rospy.loginfo(f"Reached {self.current_task}. Waiting for 30 seconds confirmation...")
                self.wait_for_confirmation(self.current_task)  # Wait regardless of confirmation
                rospy.loginfo("Returning to kitchen to check for new orders.")
                self.current_task = "kitchen"
                self.publish_goal("kitchen")

            elif self.current_task == "home":
                self.current_task = None
                if self.task_queue:
                    self.handle_kitchen_task()

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