import rclpy
import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import operator
import sys
import py_trees.console as console


class IsCollisionImminent(pt.behaviour.Behaviour):
    """
    Condition check: Returns SUCCESS if 'collision_imminent' is True on the blackboard.
    """
    def __init__(self, name="Is Collision Imminent"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="IsCollisionImminent")
        self.blackboard.register_key(key='collision_imminent', access=pt.common.Access.READ)
        
    def update(self):
        self.logger.info("[CONDITION] Checking if collision_imminent is True.")
        if self.blackboard.collision_imminent:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class IsBatteryLow(pt.behaviour.Behaviour):
    """
    Condition check: Returns SUCCESS if 'battery_low_warning' is True on the blackboard.
    """
    def __init__(self, name="Is Battery Low"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="IsBatteryLow")
        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.READ)
        
    def update(self):
        self.logger.info("[CONDITION] Checking if battery_low_warning is True.")
        if self.blackboard.battery_low_warning:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE



class MoveForward(pt.behaviour.Behaviour):
    """Moves the robot forward along the x-axis
    """
    def __init__(self, name="Move Forward",
                 topic_name="/cmd_vel",
                 lin_vel=0.3):
        super(MoveForward, self).__init__(name)
        

        self.topic_name = topic_name
        self.lin_vel = lin_vel
        self.cmd_vel_pub = None
        self.twist_msg = Twist()
        self.twist_msg.linear.x = self.lin_vel

    def setup(self, **kwargs):
        self.logger.info("[MOVE_FORWARD] setting up move forward behaviour")
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 


        self.cmd_vel_pub = self.node.create_publisher(Twist, self.topic_name, 10)
        return True

    def update(self):
        self.logger.info("[MOVE_FORWARD] update: publishing move forward command")
        
        self.cmd_vel_pub.publish(self.twist_msg)
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """Ensure motion stops upon termination when a higher priority behavior takes over.
        """
        self.logger.info("[MOVE_FORWARD] terminate: stopping forward motion")
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        return super().terminate(new_status)


class Rotate(pt.behaviour.Behaviour):
    """Rotates the robot about the z-axis
    """
    def __init__(self, name="rotate platform",
                 topic_name="/cmd_vel",
                 ang_vel=1.0):
        super(Rotate, self).__init__(name)
        
        self.topic_name = topic_name
        self.ang_vel = ang_vel
        self.cmd_vel_pub = None
        self.twist_msg = Twist()
        self.twist_msg.angular.z = self.ang_vel

    def setup(self, **kwargs):
        """Setting up things which generally might require time to prevent delay in the tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behaviour")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 


        self.cmd_vel_pub = self.node.create_publisher(Twist, self.topic_name, 10)

        return True

    def update(self):
        """Rotates the robot at the maximum allowed angular velocity.
        """
        self.logger.info("[ROTATE] update: publishing rotation command")
        self.logger.debug("%s.update()" % self._class.name_)

        
        self.cmd_vel_pub.publish(self.twist_msg)
        
        return pt.common.Status.RUNNING


    def terminate(self, new_status):
        """Trigerred once the execution of the behaviour finishes,
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")

        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        return super().terminate(new_status)

class StopMotion(pt.behaviour.Behaviour):
    """Stops the robot when it is controlled using a joystick or with a cmd_vel command
    """
    def __init__(self, name="Stop Motion", topic_name="/cmd_vel"):
        super(StopMotion, self).__init__(name)
        
        self.topic_name = topic_name
        self.cmd_vel_pub = None
        
        self.stop_msg = Twist() 

    def setup(self, **kwargs):
        self.logger.info("[STOP_MOTION] setting up stop motion behaviour")
        try:
            
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e
        
        
        self.cmd_vel_pub = self.node.create_publisher(Twist, self.topic_name, 10)
        return True

    def update(self):
        self.logger.info("[STOP_MOTION] update: publishing stop command")
        self.cmd_vel_pub.publish(self.stop_msg)
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.info("[STOP_MOTION] terminate: nothing to do, publishing zero velocity is continuous")
        return super().terminate(new_status)

class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    """Checks the battery status and updates the 'battery_low_warning' flag on the blackboard.
    """
    def __init__(self, battery_level_topic_name: str="/battery_level",
                 name: str='Battery2BB',
                 threshold: float=30.0):
        
        super().__init__(name=name,
                         topic_name=battery_level_topic_name,
                         topic_type=Float32,
                         blackboard_variables={'battery': 'data'},
                         initialise_variables={'battery': 100.0},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,
                         qos_profile=ptr.utilities.qos_profile_unlatched())
        

        self.threshold = threshold

        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.WRITE)
        self.blackboard.battery_low_warning = False 

    def update(self):

        self.logger.info('[BATTERY] update: running battery_status2bb update')

        super().update()
        
        if self.blackboard.battery < self.threshold:
            self.blackboard.battery_low_warning = True
            self.logger.info(f"[BATTERY] Low warning! Level: {self.blackboard.battery}")
        else:
            self.blackboard.battery_low_warning = False
        

        return pt.common.Status.SUCCESS


class LaserScan2bb(ptr.subscribers.ToBlackboard):
    """Checks the laser scan measurements to avoid possible collisions and sets 'collision_imminent' on the blackboard.
    """
    def __init__(self, topic_name: str="/scan",
                 name: str='Scan2BB',
                 safe_range: float=0.25):
 
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=LaserScan,
                         blackboard_variables={'laser_scan':'ranges'},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,
                         qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                depth=10))
        

        self.safe_range = safe_range
        self.blackboard.register_key(key='collision_imminent', access=pt.common.Access.WRITE)
        self.blackboard.collision_imminent = False

    def update(self):
        self.logger.info('[LASER] update: running LaserScan2bb update')
        super().update()
        ranges = getattr(self.blackboard, 'laser_scan', None)
        if ranges:
            valid_ranges = [r for r in ranges if not (r == float('inf') or r == float('nan'))]
            
            if valid_ranges:
                min_distance = min(valid_ranges)
                
                if min_distance < self.safe_range:
                    self.blackboard.collision_imminent = True
                    self.logger.info(f"[LASER] Collision imminent! Min distance: {min_distance}")
                else:
                    self.blackboard.collision_imminent = False
            else:
                 self.blackboard.collision_imminent = False

        else:
             self.logger.info("[LASER] No laser scan data received yet.")
             self.blackboard.collision_imminent = False


        return pt.common.Status.SUCCESS



def create_root(node: rclpy.node.Node) -> pt.behaviour.Behaviour:
    """Structures a behaviour tree to monitor the battery status, and start
    to rotate if the battery is low or stop if it detects an obstacle in front of it.
    The default action is to move forward.
    """

    
    root = pt.composites.Parallel(name="root",
                                  policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))    

    
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    topics2BB.add_children([
        BatteryStatus2bb(),
        LaserScan2bb()
    ])

    
    priorities = pt.composites.Selector("Priorities (High to Low)", memory=False)
    
    
    emergency_stop = pt.composites.Sequence("Emergency Stop (Collision)", memory=False)
    
    
    collision_check = IsCollisionImminent()
    
    
    stop_motion = StopMotion()

    emergency_stop.add_children([collision_check, stop_motion])

    low_battery_charge = pt.composites.Sequence("Low Battery Charging", memory=False)
    
    battery_check = IsBatteryLow()
    
    rotate_action = Rotate()
    
    low_battery_charge.add_children([battery_check, rotate_action])

    move_forward = MoveForward(name="Default: Move Forward")

    # Construct the behaviour tree structure
    priorities.add_children([
        emergency_stop,   # 1st priority: Collision Avoidance
        low_battery_charge, # 2nd priority: Battery Low
        move_forward      # 3rd priority: Default operation
    ])

    root.add_children([topics2BB, priorities])

    return root

def main():
    """Initialises and executes the behaviour tree
    """
    rclpy.init(args=None)

    node = rclpy.node.Node('robile_safety_bt_node')

    root = create_root(node)
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

  
    try:
        tree.setup(timeout=30.0, node=node)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    
    tree.tick_tock(period_ms=100)    

    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()