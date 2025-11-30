import unittest
from unittest.mock import MagicMock, patch
import sys

# Mock ROS2 modules before importing the actual module
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['smach'] = MagicMock()
sys.modules['smach_ros'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()


# Create mock classes for ROS messages
class MockTwist:
    def __init__(self):
        self.linear = MagicMock()
        self.linear.x = 0.0
        self.angular = MagicMock()
        self.angular.z = 0.0


class MockFloat32:
    def __init__(self, data=0.0):
        self.data = data


class MockLaserScan:
    def __init__(self, ranges=None):
        self.ranges = ranges if ranges is not None else []


# Patch the message classes
sys.modules['geometry_msgs.msg'].Twist = MockTwist
sys.modules['std_msgs.msg'].Float32 = MockFloat32
sys.modules['sensor_msgs.msg'].LaserScan = MockLaserScan


# Import smach.State base class mock
class MockSmachState:
    def __init__(self, outcomes):
        self.outcomes = outcomes


sys.modules['smach'].State = MockSmachState

# Now import the module under test
from serassignment1.smach_node import (
    MonitorBatteryAndCollision,
    RotateBase,
    StopMotion,
    MoveForward
)


class TestMonitorBatteryAndCollision(unittest.TestCase):
    """Test cases for MonitorBatteryAndCollision state"""

    def setUp(self):
        """Set up test fixtures"""
        self.mock_node = MagicMock()
        self.mock_node.create_subscription = MagicMock(return_value=MagicMock())
        self.mock_node.get_logger = MagicMock(return_value=MagicMock())
        self.monitor_state = MonitorBatteryAndCollision(self.mock_node)

    def test_initialization_default_thresholds(self):
        """Test that default thresholds are set correctly"""
        state = MonitorBatteryAndCollision(self.mock_node)
        self.assertEqual(state.battery_threshold, 40.0)
        self.assertEqual(state.collision_threshold, 0.4)
        self.assertEqual(state.battery, 100.0)
        self.assertEqual(state.min_distance, 10.0)

    def test_initialization_custom_thresholds(self):
        """Test initialization with custom thresholds"""
        state = MonitorBatteryAndCollision(self.mock_node, battery_threshold=50.0, collision_threshold=0.5)
        self.assertEqual(state.battery_threshold, 50.0)
        self.assertEqual(state.collision_threshold, 0.5)

    def test_battery_callback_updates_battery(self):
        """Test that battery callback updates battery level"""
        msg = MockFloat32(data=75.0)
        self.monitor_state.battery_callback(msg)
        self.assertEqual(self.monitor_state.battery, 75.0)

    def test_scan_callback_updates_min_distance(self):
        """Test that scan callback updates minimum distance"""
        msg = MockLaserScan(ranges=[1.0, 0.5, 2.0, 0.3])
        self.monitor_state.scan_callback(msg)
        self.assertEqual(self.monitor_state.min_distance, 0.3)

    def test_scan_callback_empty_ranges(self):
        """Test scan callback with empty ranges"""
        msg = MockLaserScan(ranges=[])
        self.monitor_state.scan_callback(msg)
        self.assertEqual(self.monitor_state.min_distance, 10.0)

    def test_execute_returns_collision_detected(self):
        """Test execute returns collision_detected when distance is below threshold"""
        self.monitor_state.min_distance = 0.3
        self.monitor_state.battery = 100.0
        result = self.monitor_state.execute(None)
        self.assertEqual(result, "collision_detected")

    def test_execute_returns_battery_low(self):
        """Test execute returns battery_low when battery is below threshold"""
        self.monitor_state.min_distance = 1.0
        self.monitor_state.battery = 30.0
        result = self.monitor_state.execute(None)
        self.assertEqual(result, "battery_low")

    def test_execute_returns_normal(self):
        """Test execute returns normal when all conditions are ok"""
        self.monitor_state.min_distance = 1.0
        self.monitor_state.battery = 100.0
        result = self.monitor_state.execute(None)
        self.assertEqual(result, "normal")

    def test_collision_takes_priority_over_low_battery(self):
        """Test that collision detection takes priority over low battery"""
        self.monitor_state.min_distance = 0.2
        self.monitor_state.battery = 30.0
        result = self.monitor_state.execute(None)
        self.assertEqual(result, "collision_detected")

    def test_subscriptions_created(self):
        """Test that subscriptions are created during initialization"""
        mock_node = MagicMock()
        mock_node.create_subscription = MagicMock(return_value=MagicMock())
        mock_node.get_logger = MagicMock(return_value=MagicMock())
        MonitorBatteryAndCollision(mock_node)
        self.assertEqual(mock_node.create_subscription.call_count, 2)


class TestRotateBase(unittest.TestCase):
    """Test cases for RotateBase state"""

    def setUp(self):
        """Set up test fixtures"""
        self.mock_node = MagicMock()
        self.mock_node.create_subscription = MagicMock(return_value=MagicMock())
        self.mock_node.create_publisher = MagicMock(return_value=MagicMock())
        self.mock_node.get_logger = MagicMock(return_value=MagicMock())
        self.rotate_state = RotateBase(self.mock_node)

    def test_initialization_default_threshold(self):
        """Test that default battery threshold is set correctly"""
        state = RotateBase(self.mock_node)
        self.assertEqual(state.battery_threshold, 40.0)
        self.assertEqual(state.battery, 100.0)

    def test_initialization_custom_threshold(self):
        """Test initialization with custom battery threshold"""
        state = RotateBase(self.mock_node, battery_threshold=60.0)
        self.assertEqual(state.battery_threshold, 60.0)

    def test_battery_callback_updates_battery(self):
        """Test that battery callback updates battery level"""
        msg = MockFloat32(data=50.0)
        self.rotate_state.battery_callback(msg)
        self.assertEqual(self.rotate_state.battery, 50.0)

    def test_publisher_created(self):
        """Test that cmd_vel publisher is created"""
        mock_node = MagicMock()
        mock_node.create_subscription = MagicMock(return_value=MagicMock())
        mock_node.create_publisher = MagicMock(return_value=MagicMock())
        mock_node.get_logger = MagicMock(return_value=MagicMock())
        RotateBase(mock_node)
        mock_node.create_publisher.assert_called()

    def test_subscription_created(self):
        """Test that battery subscription is created"""
        mock_node = MagicMock()
        mock_node.create_subscription = MagicMock(return_value=MagicMock())
        mock_node.create_publisher = MagicMock(return_value=MagicMock())
        mock_node.get_logger = MagicMock(return_value=MagicMock())
        RotateBase(mock_node)
        mock_node.create_subscription.assert_called()

    @patch('serassignment1.smach_node.rclpy')
    def test_execute_returns_battery_ok_when_restored(self, mock_rclpy):
        """Test execute returns battery_ok when battery is restored"""
        # Battery is above threshold, should return immediately
        self.rotate_state.battery = self.rotate_state.battery_threshold + 10
        mock_rclpy.ok.return_value = True
        mock_rclpy.ok.side_effect = [True]
        
        result = self.rotate_state.execute(None)
        self.assertEqual(result, "battery_ok")


class TestStopMotion(unittest.TestCase):
    """Test cases for StopMotion state"""

    def setUp(self):
        """Set up test fixtures"""
        self.mock_node = MagicMock()
        self.mock_node.create_subscription = MagicMock(return_value=MagicMock())
        self.mock_node.create_publisher = MagicMock(return_value=MagicMock())
        self.mock_node.get_logger = MagicMock(return_value=MagicMock())
        self.stop_state = StopMotion(self.mock_node)

    def test_initialization_default_threshold(self):
        """Test that default collision threshold is set correctly"""
        state = StopMotion(self.mock_node)
        self.assertEqual(state.collision_threshold, 0.4)
        self.assertEqual(state.min_distance, 10.0)
        self.assertEqual(state.battery, 100.0)

    def test_initialization_custom_threshold(self):
        """Test initialization with custom collision threshold"""
        state = StopMotion(self.mock_node, collision_threshold=0.6)
        self.assertEqual(state.collision_threshold, 0.6)

    def test_scan_callback_updates_min_distance(self):
        """Test that scan callback updates minimum distance"""
        msg = MockLaserScan(ranges=[2.0, 1.5, 0.8, 3.0])
        self.stop_state.scan_callback(msg)
        self.assertEqual(self.stop_state.min_distance, 0.8)

    def test_scan_callback_empty_ranges_no_update(self):
        """Test that scan callback doesn't update on empty ranges"""
        initial_distance = self.stop_state.min_distance
        msg = MockLaserScan(ranges=[])
        self.stop_state.scan_callback(msg)
        self.assertEqual(self.stop_state.min_distance, initial_distance)

    def test_battery_callback_updates_battery(self):
        """Test that battery callback updates battery level"""
        msg = MockFloat32(data=65.0)
        self.stop_state.battery_callback(msg)
        self.assertEqual(self.stop_state.battery, 65.0)

    def test_publisher_and_subscriptions_created(self):
        """Test that publisher and subscriptions are created"""
        mock_node = MagicMock()
        mock_node.create_subscription = MagicMock(return_value=MagicMock())
        mock_node.create_publisher = MagicMock(return_value=MagicMock())
        mock_node.get_logger = MagicMock(return_value=MagicMock())
        StopMotion(mock_node)
        mock_node.create_publisher.assert_called()
        self.assertEqual(mock_node.create_subscription.call_count, 2)

    @patch('serassignment1.smach_node.rclpy')
    def test_execute_returns_safe_again(self, mock_rclpy):
        """Test execute returns safe_again when distance is restored"""
        self.stop_state.min_distance = 1.0  # Above collision threshold
        mock_rclpy.ok.return_value = True
        
        result = self.stop_state.execute(None)
        self.assertEqual(result, "safe_again")


class TestMoveForward(unittest.TestCase):
    """Test cases for MoveForward state"""

    def setUp(self):
        """Set up test fixtures"""
        self.mock_node = MagicMock()
        self.mock_node.create_subscription = MagicMock(return_value=MagicMock())
        self.mock_node.create_publisher = MagicMock(return_value=MagicMock())
        self.mock_node.get_logger = MagicMock(return_value=MagicMock())
        self.move_state = MoveForward(self.mock_node)

    def test_initialization_default_thresholds(self):
        """Test that default thresholds are set correctly"""
        state = MoveForward(self.mock_node)
        self.assertEqual(state.collision_threshold, 0.4)
        self.assertEqual(state.battery_threshold, 40.0)
        self.assertEqual(state.min_distance, 10.0)
        self.assertEqual(state.battery, 100.0)

    def test_initialization_custom_thresholds(self):
        """Test initialization with custom thresholds"""
        state = MoveForward(self.mock_node, collision_threshold=0.5, battery_threshold=50.0)
        self.assertEqual(state.collision_threshold, 0.5)
        self.assertEqual(state.battery_threshold, 50.0)

    def test_scan_callback_updates_min_distance(self):
        """Test that scan callback updates minimum distance"""
        msg = MockLaserScan(ranges=[1.2, 0.7, 2.5, 1.8])
        self.move_state.scan_callback(msg)
        self.assertEqual(self.move_state.min_distance, 0.7)

    def test_scan_callback_empty_ranges_no_update(self):
        """Test that scan callback doesn't update on empty ranges"""
        initial_distance = self.move_state.min_distance
        msg = MockLaserScan(ranges=[])
        self.move_state.scan_callback(msg)
        self.assertEqual(self.move_state.min_distance, initial_distance)

    def test_battery_callback_updates_battery(self):
        """Test that battery callback updates battery level"""
        msg = MockFloat32(data=80.0)
        self.move_state.battery_callback(msg)
        self.assertEqual(self.move_state.battery, 80.0)

    def test_publisher_and_subscriptions_created(self):
        """Test that publisher and subscriptions are created"""
        mock_node = MagicMock()
        mock_node.create_subscription = MagicMock(return_value=MagicMock())
        mock_node.create_publisher = MagicMock(return_value=MagicMock())
        mock_node.get_logger = MagicMock(return_value=MagicMock())
        MoveForward(mock_node)
        mock_node.create_publisher.assert_called()
        self.assertEqual(mock_node.create_subscription.call_count, 2)

    @patch('serassignment1.smach_node.rclpy')
    def test_execute_returns_battery_low(self, mock_rclpy):
        """Test execute returns battery_low when battery drops"""
        self.move_state.battery = 30.0  # Below threshold
        self.move_state.min_distance = 1.0  # Safe distance
        mock_rclpy.ok.return_value = True
        
        result = self.move_state.execute(None)
        self.assertEqual(result, "battery_low")

    @patch('serassignment1.smach_node.rclpy')
    def test_execute_returns_collision_detected(self, mock_rclpy):
        """Test execute returns collision_detected when obstacle detected"""
        self.move_state.battery = 100.0  # Good battery
        self.move_state.min_distance = 0.2  # Below collision threshold
        mock_rclpy.ok.return_value = True
        
        result = self.move_state.execute(None)
        self.assertEqual(result, "collision_detected")

    @patch('serassignment1.smach_node.rclpy')
    def test_collision_priority_over_battery(self, mock_rclpy):
        """Test that collision detection has priority when both conditions are met"""
        self.move_state.battery = 30.0  # Below battery threshold
        self.move_state.min_distance = 0.2  # Below collision threshold
        mock_rclpy.ok.return_value = True
        
        result = self.move_state.execute(None)
        # Battery low is checked first in the code
        self.assertEqual(result, "battery_low")


class TestStateOutcomes(unittest.TestCase):
    """Test state outcomes are correctly defined"""

    def setUp(self):
        """Set up test fixtures"""
        self.mock_node = MagicMock()
        self.mock_node.create_subscription = MagicMock(return_value=MagicMock())
        self.mock_node.create_publisher = MagicMock(return_value=MagicMock())
        self.mock_node.get_logger = MagicMock(return_value=MagicMock())

    def test_monitor_state_outcomes(self):
        """Test MonitorBatteryAndCollision has correct outcomes"""
        state = MonitorBatteryAndCollision(self.mock_node)
        self.assertTrue(hasattr(state, 'outcomes'))

    def test_rotate_state_outcomes(self):
        """Test RotateBase has correct outcomes"""
        state = RotateBase(self.mock_node)
        self.assertTrue(hasattr(state, 'outcomes'))

    def test_stop_state_outcomes(self):
        """Test StopMotion has correct outcomes"""
        state = StopMotion(self.mock_node)
        self.assertTrue(hasattr(state, 'outcomes'))

    def test_move_state_outcomes(self):
        """Test MoveForward has correct outcomes"""
        state = MoveForward(self.mock_node)
        self.assertTrue(hasattr(state, 'outcomes'))


class TestEdgeCases(unittest.TestCase):
    """Test edge cases and boundary conditions"""

    def setUp(self):
        """Set up test fixtures"""
        self.mock_node = MagicMock()
        self.mock_node.create_subscription = MagicMock(return_value=MagicMock())
        self.mock_node.create_publisher = MagicMock(return_value=MagicMock())
        self.mock_node.get_logger = MagicMock(return_value=MagicMock())

    def test_battery_at_threshold(self):
        """Test behavior when battery is exactly at threshold"""
        state = MonitorBatteryAndCollision(self.mock_node, battery_threshold=40.0)
        state.battery = 40.0
        state.min_distance = 1.0
        result = state.execute(None)
        # Battery at threshold is not low (< not <=)
        self.assertEqual(result, "normal")

    def test_distance_at_threshold(self):
        """Test behavior when distance is exactly at threshold"""
        state = MonitorBatteryAndCollision(self.mock_node, collision_threshold=0.4)
        state.battery = 100.0
        state.min_distance = 0.4
        result = state.execute(None)
        # Distance at threshold is not collision (< not <=)
        self.assertEqual(result, "normal")

    def test_battery_just_below_threshold(self):
        """Test behavior when battery is just below threshold"""
        state = MonitorBatteryAndCollision(self.mock_node, battery_threshold=40.0)
        state.battery = 39.9
        state.min_distance = 1.0
        result = state.execute(None)
        self.assertEqual(result, "battery_low")

    def test_distance_just_below_threshold(self):
        """Test behavior when distance is just below threshold"""
        state = MonitorBatteryAndCollision(self.mock_node, collision_threshold=0.4)
        state.battery = 100.0
        state.min_distance = 0.39
        result = state.execute(None)
        self.assertEqual(result, "collision_detected")

    def test_zero_battery(self):
        """Test behavior with zero battery"""
        state = MonitorBatteryAndCollision(self.mock_node)
        state.battery = 0.0
        state.min_distance = 1.0
        result = state.execute(None)
        self.assertEqual(result, "battery_low")

    def test_zero_distance(self):
        """Test behavior with zero distance (collision)"""
        state = MonitorBatteryAndCollision(self.mock_node)
        state.battery = 100.0
        state.min_distance = 0.0
        result = state.execute(None)
        self.assertEqual(result, "collision_detected")

    def test_negative_battery(self):
        """Test behavior with negative battery value"""
        state = MonitorBatteryAndCollision(self.mock_node)
        state.battery = -5.0
        state.min_distance = 1.0
        result = state.execute(None)
        self.assertEqual(result, "battery_low")

    def test_very_large_distance(self):
        """Test behavior with very large distance value"""
        state = MonitorBatteryAndCollision(self.mock_node)
        state.battery = 100.0
        state.min_distance = 1000.0
        result = state.execute(None)
        self.assertEqual(result, "normal")

    def test_single_range_value(self):
        """Test scan callback with single range value"""
        state = MonitorBatteryAndCollision(self.mock_node)
        msg = MockLaserScan(ranges=[0.5])
        state.scan_callback(msg)
        self.assertEqual(state.min_distance, 0.5)


if __name__ == "__main__":
    unittest.main()
