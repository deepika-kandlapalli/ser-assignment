import rclpy
from rclpy.node import Node
import smach
import smach_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions"""
    def __init__(self, node, battery_threshold=40.0, collision_threshold=0.4):
        super().__init__(outcomes=["battery_low", "collision_detected", "normal"])
        self.node = node
        self.battery_threshold = battery_threshold
        self.collision_threshold = collision_threshold
        self.battery = 100.0
        self.min_distance = 10.0

        # Subscribers
        self.battery_sub = node.create_subscription(Float32, "/battery_level", self.battery_callback, 10)
        self.scan_sub = node.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

    def battery_callback(self, msg):
        self.battery = msg.data

    def scan_callback(self, msg):
        if msg.ranges:
            self.min_distance = min(msg.ranges)
        else:
            self.min_distance = 10.0

    def execute(self, userdata):
        # Print battery and distance
        self.node.get_logger().info(f"[MONITOR] Battery: {self.battery:.1f}%, Min Distance: {self.min_distance:.2f} m")
        if self.min_distance < self.collision_threshold:
            return "collision_detected"
        elif self.battery < self.battery_threshold:
            return "battery_low"
        else:
            return "normal"



class RotateBase(smach.State):
    """State to rotate the Robile base"""
    def __init__(self, node, battery_threshold=40.0):
        super().__init__(outcomes=["battery_ok"])
        self.node = node
        self.battery_threshold = battery_threshold
        self.battery = 100.0
        self.cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self.battery_sub = node.create_subscription(Float32, "/battery_level", self.battery_callback, 10)

    def battery_callback(self, msg):
        self.battery = msg.data

    def execute(self, userdata):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 1.0
        self.node.get_logger().info(" Rotating due to low battery...")

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.cmd_pub.publish(twist)
            self.node.get_logger().info(f"[ROTATE] Battery: {self.battery:.1f}%")

            if self.battery >= self.battery_threshold:
                self.cmd_pub.publish(Twist())  # stop rotation
                self.node.get_logger().info("Battery restored, resuming forward motion")
                return "battery_ok"



class StopMotion(smach.State):
    """State to stop the robot's motion"""
    def __init__(self, node, collision_threshold=0.4):
        super().__init__(outcomes=["safe_again"])
        self.node = node
        self.collision_threshold = collision_threshold
        self.min_distance = 10.0
        self.cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = node.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.battery = 100.0
        self.battery_sub = node.create_subscription(Float32, "/battery_level", self.battery_callback, 10)

    def scan_callback(self, msg):
        if msg.ranges:
            self.min_distance = min(msg.ranges)

    def battery_callback(self, msg):
        self.battery = msg.data

    def execute(self, userdata):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)
        self.node.get_logger().error("Collision detected! Robot stopped. Move manually.")

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.cmd_pub.publish(stop)
            self.node.get_logger().info(f"[STOP] Battery: {self.battery:.1f}%, Min Distance: {self.min_distance:.2f} m")

            if self.min_distance > self.collision_threshold:
                self.node.get_logger().info("Safe distance restored, resuming forward motion")
                return "safe_again"



class MoveForward(smach.State):
    """State to move the robot forward continuously"""
    def __init__(self, node, collision_threshold=0.4, battery_threshold=40.0):
        super().__init__(outcomes=["battery_low", "collision_detected"])
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self.collision_threshold = collision_threshold
        self.battery_threshold = battery_threshold
        self.min_distance = 10.0
        self.battery = 100.0

        self.scan_sub = node.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.battery_sub = node.create_subscription(Float32, "/battery_level", self.battery_callback, 10)

    def scan_callback(self, msg):
        if msg.ranges:
            self.min_distance = min(msg.ranges)

    def battery_callback(self, msg):
        self.battery = msg.data

    def execute(self, userdata):
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.node.get_logger().info("➡ Moving forward...")

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.cmd_pub.publish(twist)
            self.node.get_logger().info(f"[MOVE] Battery: {self.battery:.1f}%, Min Distance: {self.min_distance:.2f} m")

            if self.battery < self.battery_threshold:
                self.node.get_logger().warn("Battery low! Switching to ROTATE")
                return "battery_low"
            if self.min_distance < self.collision_threshold:
                self.node.get_logger().warn("Collision detected! Switching to STOP")
                return "collision_detected"



def main(args=None):
    rclpy.init(args=args)
    node = Node("safety_state_machine")

    sm = smach.StateMachine(outcomes=["finished"])
    with sm:
        smach.StateMachine.add(
            "MONITOR",
            MonitorBatteryAndCollision(node),
            transitions={
                "battery_low": "ROTATE",
                "collision_detected": "STOP",
                "normal": "MOVE_FORWARD"
            }
        )

        smach.StateMachine.add(
            "MOVE_FORWARD",
            MoveForward(node),
            transitions={
                "battery_low": "ROTATE",
                "collision_detected": "STOP"
            }
        )

        smach.StateMachine.add(
            "ROTATE",
            RotateBase(node),
            transitions={"battery_ok": "MOVE_FORWARD"}
        )

        smach.StateMachine.add(
            "STOP",
            StopMotion(node),
            transitions={"safe_again": "MOVE_FORWARD"}
        )

    # Introspection server for SMACH GUI
    sis = smach_ros.IntrospectionServer("safety_server", sm, "/SAFETY_SM")
    sis.start()
    sm.execute()
    rclpy.spin(node)
    sis.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()