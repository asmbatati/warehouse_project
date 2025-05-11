#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import SetBool
from attach_shelf.srv import GoToLoading
import tf_transformations
import math
import time

# Define warehouse positions
warehouse_positions = {
    "init_position": [0.01592554960126385, -0.0010036087994216704, 0.0],            # [x, y, yaw]
    "loading_position": [5.602146584521654, -0.19586829992020444, 0.0],         # Near shelf
    "shipping_position": [2.4651862793680066, 1.3015561968534988, 0.0]   
}

def main():
    # Initialize ROS and create node
    rclpy.init()
    navigator = BasicNavigator()
    
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    
    # Set initial pose for localization
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = warehouse_positions["init_position"][0]
    initial_pose.pose.position.y = warehouse_positions["init_position"][1]
    initial_pose.pose.position.z = 0.0
    
    # Convert yaw to quaternion
    q = tf_transformations.quaternion_from_euler(0, 0, warehouse_positions["init_position"][2])
    initial_pose.pose.orientation.x = q[0]
    initial_pose.pose.orientation.y = q[1]
    initial_pose.pose.orientation.z = q[2]
    initial_pose.pose.orientation.w = q[3]
    
    # Set initial pose
    navigator.setInitialPose(initial_pose)
    print("Initial pose set")
    
    # Wait for localization
    print("Waiting for localization to converge...")
    time.sleep(3.0)
    
    # Navigate to loading position
    print('Navigating to loading position to pick up shelf...')
    loading_pose = create_pose_stamped(navigator, 
                                      warehouse_positions["loading_position"][0], 
                                      warehouse_positions["loading_position"][1], 
                                      warehouse_positions["loading_position"][2])
    navigator.goToPose(loading_pose)
    
    # Print ETA information while navigating
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Distance remaining to loading position: {feedback.distance_remaining:.2f} meters.')
            print('Estimated time of arrival at loading position: ' + 
                  '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + 
                  ' seconds.')
        time.sleep(1)
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Arrived at loading position")
        
        # Get under the shelf and lift it
        print("Loading the shelf...")
        success = load_shelf()
        
        if success:
            # Change robot footprint to reflect the loaded shelf dimensions
            print("Updating robot footprint...")
            update_robot_footprint(True)  # True = with shelf
            
            # Navigate to shipping position
            print('Navigating to shipping position while carrying shelf...')
            shipping_pose = create_pose_stamped(navigator, 
                                           warehouse_positions["shipping_position"][0], 
                                           warehouse_positions["shipping_position"][1], 
                                           warehouse_positions["shipping_position"][2])
            navigator.goToPose(shipping_pose)
            
            # Print ETA information while navigating
            i = 0
            while not navigator.isTaskComplete():
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print(f'Distance remaining to shipping position: {feedback.distance_remaining:.2f} meters.')
                    print('Estimated time of arrival at shipping position: ' + 
                        '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + 
                        ' seconds.')
                time.sleep(1)
            
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("Arrived at shipping position")
                
                # Unload the shelf
                print("Unloading the shelf...")
                unload_shelf()
                
                # Update robot footprint to original size
                print("Restoring robot footprint...")
                update_robot_footprint(False)  # False = without shelf
                
                # Return to initial position
                print('Returning to initial position...')
                init_pose = create_pose_stamped(navigator, 
                                               warehouse_positions["init_position"][0], 
                                               warehouse_positions["init_position"][1], 
                                               warehouse_positions["init_position"][2])
                navigator.goToPose(init_pose)
                
                # Print ETA information while navigating
                i = 0
                while not navigator.isTaskComplete():
                    i = i + 1
                    feedback = navigator.getFeedback()
                    if feedback and i % 5 == 0:
                        print(f'Distance remaining to initial position: {feedback.distance_remaining:.2f} meters.')
                        print('Estimated time of arrival at initial position: ' + 
                            '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + 
                            ' seconds.')
                    time.sleep(1)
                
                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print("Returned to initial position")
                    print("Mission completed successfully!")
                else:
                    print(f"Failed to return to initial position with result code: {result}")
            else:
                print(f"Failed to reach shipping position with result code: {result}")
                # Try to recover by unloading the shelf where we are
                print("Emergency unloading of the shelf...")
                unload_shelf()
                update_robot_footprint(False)
                # Try to return to initial position
                navigator.goToPose(init_pose)
        else:
            print("Failed to load shelf. Returning to initial position.")
            init_pose = create_pose_stamped(navigator, 
                                           warehouse_positions["init_position"][0], 
                                           warehouse_positions["init_position"][1], 
                                           warehouse_positions["init_position"][2])
            navigator.goToPose(init_pose)
    elif result == TaskResult.CANCELED:
        print("Navigation to loading position was canceled")
        navigator.goToPose(initial_pose)
    else:
        print(f"Failed to reach loading position with result code: {result}")
    
    # Shutdown ROS node
    navigator.lifecycleShutdown()
    rclpy.shutdown()
    print("Navigation mission complete!")

def create_pose_stamped(navigator, x, y, yaw):
    """Create a PoseStamped message with the given position and orientation"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    q = tf_transformations.quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

def load_shelf():
    """Function to get under the shelf and lift it using the approach service from Checkpoint 9"""
    # Create a node for calling the approach service
    node = rclpy.create_node('shelf_loader')
    
    # Create client for the approach service
    approach_client = node.create_client(GoToLoading, 'approach_shelf')
    
    # Wait for service to be available
    while not approach_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Approach service not available, waiting...')
    
    # Create request to approach and attach to shelf
    req = GoToLoading.Request()
    req.attach_to_shelf = True
    
    # Call service
    print("Calling approach service to attach to shelf...")
    future = approach_client.call_async(req)
    
    # Wait for the result
    rclpy.spin_until_future_complete(node, future)
    
    # Check if shelf was loaded successfully
    if future.result() and future.result().complete:
        print("Shelf loaded successfully")
        success = True
    else:
        print("Failed to load shelf using approach service. Trying direct method...")
        success = load_shelf_direct(node)
    
    # Clean up
    node.destroy_node()
    return success

def load_shelf_direct(node):
    """Fallback method to directly control the robot to get under the shelf"""
    try:
        # Create client for elevator service
        elevator_client = node.create_client(SetBool, '/robot/elevator_up')
        
        # Wait for service to be available
        while not elevator_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Elevator service not available, waiting...')
        
        # Create publisher for velocity commands
        cmd_vel_pub = node.create_publisher(Twist, '/robot/cmd_vel', 10)
        
        # Move robot forwards slowly to get under the shelf
        print("Moving forward under the shelf...")
        twist = Twist()
        twist.linear.x = 0.1  # Slow forward speed (m/s)
        twist.angular.z = 0.0
        
        # Move forward for about 3 seconds
        start_time = time.time()
        
        while time.time() - start_time < 3.0:
            cmd_vel_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # Stop the robot
        twist.linear.x = 0.0
        cmd_vel_pub.publish(twist)
        
        # Call elevator service to lift the shelf
        print("Lifting the shelf...")
        req = SetBool.Request()
        req.data = True  # True to raise the elevator
        future = elevator_client.call_async(req)
        
        # Wait for the result
        rclpy.spin_until_future_complete(node, future)
        
        # Check if elevator was raised successfully
        if future.result().success:
            print("Shelf loaded successfully via direct method")
            return True
        else:
            print("Failed to load shelf via direct method")
            return False
    except Exception as e:
        print(f"Exception in direct shelf loading: {e}")
        return False

def unload_shelf():
    """Function to lower the shelf and move out from under it"""
    # Create a node for controlling the elevator
    node = rclpy.create_node('shelf_unloader')
    
    # Create client for elevator service
    elevator_client = node.create_client(SetBool, '/robot/elevator_up')
    
    # Wait for service to be available
    while not elevator_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Elevator service not available, waiting...')
    
    # Call elevator service to lower the shelf
    print("Lowering the shelf...")
    req = SetBool.Request()
    req.data = False  # False to lower the elevator
    future = elevator_client.call_async(req)
    
    # Wait for the result
    rclpy.spin_until_future_complete(node, future)
    
    # Check if elevator was lowered successfully
    if future.result().success:
        print("Shelf lowered successfully")
    else:
        print("Failed to lower shelf")
    
    # Create publisher for velocity commands
    cmd_vel_pub = node.create_publisher(Twist, '/robot/cmd_vel', 10)
    
    # Move robot backwards slowly to get out from under the shelf
    print("Moving backward from under the shelf...")
    twist = Twist()
    twist.linear.x = -0.1  # Slow backward speed (m/s)
    twist.angular.z = 0.0
    
    # Move backward for about 3 seconds
    start_time = time.time()
    
    while time.time() - start_time < 3.0:
        cmd_vel_pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Stop the robot
    twist.linear.x = 0.0
    cmd_vel_pub.publish(twist)
    
    # Clean up
    node.destroy_node()

def update_robot_footprint(with_shelf):
    """Function to update the robot's footprint in the navigation stack"""
    # Create a node for controlling the robot footprint
    node = rclpy.create_node('footprint_updater')
    
    # Create client for footprint service
    # Assuming there's a service to update the robot's footprint
    footprint_client = node.create_client(ManageLifecycleNodes, '/robot/change_footprint')
    
    # Wait for service to be available
    while not footprint_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Footprint service not available, waiting...')
    
    # Create request
    req = ManageLifecycleNodes.Request()
    if with_shelf:
        # When carrying the shelf, use a larger footprint
        req.command = 1  # CONFIGURE
    else:
        # When not carrying the shelf, use the original footprint
        req.command = 2  # CLEANUP
    
    # Call service
    future = footprint_client.call_async(req)
    
    # Wait for the result
    rclpy.spin_until_future_complete(node, future)
    
    if with_shelf:
        print("Robot footprint updated to include shelf")
    else:
        print("Robot footprint restored to original size")
    
    # Clean up
    node.destroy_node()

if __name__ == '__main__':
    main() 