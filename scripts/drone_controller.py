#!/usr/bin/python3
import rospy
import math
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Altitude, State, Waypoint, WaypointList, WaypointReached, PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, WaypointPush, WaypointPushRequest, WaypointPull, WaypointClear, ParamSet
from geometry_msgs.msg import PoseStamped, Point, Vector3

class OffboardController:
    def __init__(self):
        self.current_state = State()
        rospy.init_node('offb_node_py')

        # Subscribe to necessary topics
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.coordinate_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.altitude_sub = rospy.Subscriber('/mavros/altitude', Altitude, self.altitude_callback)
        self.waypoint_sub = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoint_list_callback)
        self.waypoint_reached_sub = rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.waypoint_reached_callback)
        self.local_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Publish to necessary topics
        self.position_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Establish message objects
        self.waypoint_list = WaypointList()
        self.reached_waypoint = WaypointReached()
        self.current_pose = PoseStamped()
        self.current_global = NavSatFix()
        self.current_altitude = Altitude()

    def state_callback(self, msg):
        """Update the vehicle state.

        Args:
            msg (State): The new state message.
        """
        self.current_state = msg

    def gps_callback(self, msg):
        """Update the vehicle global position.

        Args:
            msg (NavSatFix): The new GPS message.
        """
        self.current_global = msg

    def altitude_callback(self, msg):
        """Update the vehicle altitude.

        Args:
            msg (Altitude): The new altitude message.
        """
        self.current_altitude = msg

    def waypoint_list_callback(self, msg):
        """Update the waypoint list.

        Args:
            msg (WaypointList): The new waypoint list message.
        """
        self.waypoint_list = msg

    def waypoint_reached_callback(self, msg):
        """Update the reached waypoint.

        Args:
            msg (WaypointReached): The new reached waypoint message.
        """
        self.reached_waypoint = msg

    def pose_callback(self, msg):
        """Update the current local pose.

        Args:
            msg (PoseStamped): The new pose message.
        """
        self.current_pose = msg

    def set_parameter(self, parameter, value):
        rospy.wait_for_service('/mavros/param/set')
        try:
            # Request service and get response
            param_set_srv = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            response = param_set_srv(param_id=parameter, value=value)

            # Check if service request was successful
            if response.success:
                rospy.loginfo('Parameter: {} set to: {}'.format(parameter, value))
                return True
            else:
                rospy.logwarn('Failed to set parameter {}'.format(parameter))
                return False
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
            return False

    def set_arm(self, arm):
        """Set the arm status of the vehicle.

        Args:
            arm (bool): True to arm the vehicle, False to disarm.

        Returns:
            bool: True if the operation is successful, False otherwise.
        """
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            # Request service and get response
            arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = arm
            response = arm_srv(arm_cmd)

            # Check if service request was successful and drone is armed
            if response.success and self.current_state.armed == arm:
                rospy.loginfo('Vehicle armed: {}'.format(arm))
                return True
            else:
                rospy.logwarn('Failed to arm vehicle')
                return False
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
            return False

    def set_mode(self, mode):
        """Set the flight mode of the vehicle.

        Args:
            mode (str): The desired flight mode.

        Returns:
            bool: True if the operation is successful, False otherwise.
        """
        rospy.wait_for_service('/mavros/set_mode')
        try:
            # Request service and get response
            set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            offb_set_mode = SetModeRequest()
            offb_set_mode.custom_mode = mode
            response = set_mode_srv(offb_set_mode)

            # Check if service request was successful and drone mode is set
            if response.mode_sent and self.current_state.mode == mode:
                rospy.loginfo('Mode set to: {}'.format(mode))
                return True
            else:
                rospy.logwarn('Failed to set mode: {}'.format(mode))
                return False
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
            return False
        
    def create_waypoint(self, frame, command, is_current, p1, p2, p3, p4, x, y, z):
        """Create a new waypoint object.

        Args:
            frame (int): The coordinate frame of the waypoint.
            command (int): The command associated with the waypoint.
            is_current (bool): True if the waypoint is the current one, False otherwise.
            p1 (float): The first parameter of the waypoint.
            p2 (float): The second parameter of the waypoint.
            p3 (float): The third parameter of the waypoint.
            p4 (float): The fourth parameter of the waypoint.
            x (float): The latitude or x-coordinate of the waypoint.
            y (float): The longitude or y-coordinate of the waypoint.
            z (float): The altitude or z-coordinate of the waypoint.

        Returns:
            Waypoint: The created waypoint object.
        """
        waypoint = Waypoint()
        waypoint.frame = frame
        waypoint.command = command
        waypoint.is_current = is_current
        waypoint.autocontinue = True
        waypoint.param1 = float(p1)
        waypoint.param2 = float(p2)
        waypoint.param3 = float(p3)
        waypoint.param4 = float(p4)
        waypoint.x_lat = x
        waypoint.y_long = y
        waypoint.z_alt = z
        return waypoint

    def compare_waypoints(self, waypoint1, waypoint2):
        """Compare two waypoints for equality.

        Args:
            waypoint1 (Waypoint): The first waypoint.
            waypoint2 (Waypoint): The second waypoint.

        Returns:
            bool: True if the waypoints are equal, False otherwise.
        """
        if waypoint1.frame != waypoint2.frame:
            return False
        if waypoint1.command != waypoint2.command:
            return False
        if waypoint1.is_current != waypoint2.is_current:
            return False
        if waypoint1.autocontinue != waypoint2.autocontinue:
            return False
        if waypoint1.param1 != waypoint2.param1 and math.isnan(waypoint1.param4) != math.isnan(waypoint2.param4):
            return False
        if waypoint1.param2 != waypoint2.param2 and math.isnan(waypoint1.param4) != math.isnan(waypoint2.param4):
            return False
        if waypoint1.param3 != waypoint2.param3 and math.isnan(waypoint1.param4) != math.isnan(waypoint2.param4):
            return False
        if waypoint1.param4 != waypoint2.param4 and math.isnan(waypoint1.param4) != math.isnan(waypoint2.param4):
            return False
        if waypoint1.x_lat != waypoint2.x_lat:
            return False
        if waypoint1.y_long != waypoint2.y_long:
            return False
        if waypoint1.z_alt != waypoint2.z_alt:
            return False
        return True

    def compare_waypoint_lists(self, waypoints1, waypoints2):
        """Compare two lists of waypoints for equality.

        Args:
            waypoints1 (List[Waypoint]): The first list of waypoints.
            waypoints2 (List[Waypoint]): The second list of waypoints.

        Returns:
            bool: True if the lists are equal, False otherwise.
        """
        # Check if the lists have the same length
        if len(waypoints1) != len(waypoints2) or not waypoints1 or not waypoints2:
            return False
        
        # Iterate over each waypoint in the lists
        for i in range(len(waypoints1)):
            waypoint1 = waypoints1[i]
            waypoint2 = waypoints2[i]

            # Compare the waypoints using the compare_waypoints() function
            if not self.compare_waypoints(waypoint1, waypoint2):
                return False

        # If all waypoints are equal, the lists are equal
        return True
    
    def clear_waypoints(self):
        """Clear the current list of waypoints.

        Returns:
            bool: True if the operation is successful, False otherwise.
        """
        rospy.wait_for_service('/mavros/mission/clear')
        try:
            # Request service and get response
            waypoint_clear_srv = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            response = waypoint_clear_srv()

            # Check if service request was successful and waypoints list is empty
            if response.success and not self.waypoint_list.waypoints:
                rospy.loginfo('Waypoint clear successful')
                return True
            else:
                rospy.logwarn('Waypoint clear failed')
                return False
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
            return False
        
    def send_waypoints(self, waypoints):
        """Send a list of waypoints to the vehicle.

        Args:
            waypoints (List[Waypoint]): The waypoints to be sent.

        Returns:
            bool: True if the operation is successful, False otherwise.
        """
        rospy.wait_for_service('/mavros/mission/push')
        try:
            # Request service and get response
            waypoint_push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            waypoint_push_req = WaypointPushRequest()
            waypoint_push_req.waypoints = waypoints
            response = waypoint_push_srv(waypoint_push_req)

            # Check if service request was successful and waypoint lists match
            if response.success and self.compare_waypoint_lists(self.waypoint_list.waypoints, waypoints):
                rospy.loginfo('Waypoint push successful')
                return True
            else:
                rospy.logwarn('Waypoint push failed')
                return False
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
            return False
                
    def is_mission_completed(self):
        """Check if the mission has been completed.

        Returns:
            bool: True if the mission is completed, False otherwise.
        """
        rospy.wait_for_service('/mavros/mission/pull')
        try:
            # Request service and get response
            waypoint_pull_srv = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
            response = waypoint_pull_srv()

            # Check if service request was successful
            if response.success:
                # Get the total waypoint sequence count
                num_waypoints = response.wp_received - 1
                # Check if all waypoints have been completed and reached waypoint time stamp is current
                if self.reached_waypoint.wp_seq == num_waypoints and self.reached_waypoint.header.stamp.secs == self.current_state.header.stamp.secs:
                    return True
                else:
                    return False
            else:
                rospy.logwarn('Failed to pull waypoints')
                return False
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
            return False
    
    def target_reached(self, current, target, acceptance):
        """Check if the target has been reached.

        Args:
            current (Pose): The current position.
            target (Pose): The target position.
            acceptance (float): The acceptable distance to consider the target reached.

        Returns:
            bool: True if the target is reached, False otherwise.
        """
        x_1, y_1, z_1 = current.x, current.y, current.z
        x_2, y_2, z_2 = target.x, target.y, target.z

        distance = math.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2 + (z_1 - z_2)**2)
        if distance <= acceptance:
            return True
        else:
            return False
        
    def send_position(self, position):
        """Send the desired position to the vehicle.

        Args:
            position (geometry_msgs.msg.Point): The desired position as a 3D point (x, y, z).
        """
        position_msg = PositionTarget()
        position_msg.header.stamp = rospy.Time.now()
        position_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                                PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        position_msg.position = position
        self.position_target_pub.publish(position_msg)

    def go_to_position(self, x, y, z):
        """Move the vehicle to the specified position.

        Args:
            x (float): The x-coordinate of the target position.
            y (float): The y-coordinate of the target position.
            z (float): The z-coordinate of the target position.
        """
        position = Point()
        position.x = x
        position.y = y
        position.z = z
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.target_reached(self.current_pose.pose.position, position, 0.25):
            self.send_position(position)
            if self.current_state.mode != 'OFFBOARD':
                self.set_mode('OFFBOARD')
            rate.sleep()
            
    def send_velocity(self, velocity):
        """Send the desired velocity to the vehicle.

        Args:
            velocity (geometry_msgs.msg.Vector3): The desired velocity as a 3D vector (vx, vy, vz).
        """
        velocity_msg = PositionTarget()
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_msg.coordiante_frame = PositionTarget.FRAME_LOCAL_NED
        velocity_msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                                PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        velocity_msg.velocity = velocity
        self.position_target_pub.publish(velocity_msg)

    def magical_shape(self):
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        start_z = self.current_pose.pose.position.z
        scale = 10
        point_list = [[-2, 0, 0],
                      [-1, 0, 0],
                      [-1, -3, 0],
                      [0, -3, 0],
                      [0, 0, 0],
                      [1, -3, 0],
                      [2, 0, 0],
                      [2, -3, 0],
                      [3, 0, 0],
                      [3.5, -1.5, 0],
                      [2.5, -1.5, 0],
                      [3.5, -1.5, 0],
                      [4, -3, 0],
                      [5, -3, 0],
                      [4, 0, 0],
                      [5, -3, 0],
                      [6, 0, 0]]
        for point in point_list:
            self.go_to_position(start_x + scale*point[0], start_y + scale*point[1], start_z + scale*point[2])

if __name__ == "__main__":
    rospy.init_node('offb_node_py')
    drone = OffboardController()
    
    rospy.loginfo('Waiting for FCU connection...')
    while not rospy.is_shutdown() and not drone.current_state.connected:
        rospy.sleep(0.1)
    rospy.loginfo('FCU connected')

    # # Set maximum velocity
    # while not rospy.is_shutdown() and not drone.set_parameter('MPC_XY_VEL_MAX', ParamValue(real=10.0)):
    #     rospy.sleep(1.0)

    # # Set maximum pitch rate
    # while not rospy.is_shutdown() and not drone.set_parameter('MC_ROLLRATE_MAX', ParamValue(integer=0, real=50.0)):
    #     rospy.sleep(1.0)

    # # Set maximum roll rate
    # while not rospy.is_shutdown() and not drone.set_parameter('MC_PITCHRATE_MAX', ParamValue(integer=0, real=50.0)):
    #     rospy.sleep(1.0)

    # # Set maximum acceleration
    # while not rospy.is_shutdown() and not drone.set_parameter('MPC_ACC_HOR', ParamValue(integer=0, real=3)):
    #     rospy.sleep(1.0)

    waypoints = [drone.create_waypoint(3, 22, True, 0, 0, 0, float('nan'), 47.398290, 8.545723, 10.0),
                 drone.create_waypoint(3, 16, False, 0, 10, 0, float('nan'), 47.398186, 8.547263, 10.0)]
    
    rospy.loginfo('Clearing Old Waypoints...')
    while not rospy.is_shutdown() and not drone.clear_waypoints():
        rospy.sleep(0.1)

    rospy.loginfo('Sending New Waypoints...')
    while not rospy.is_shutdown() and not drone.send_waypoints(waypoints):
        rospy.sleep(0.1)

    rospy.loginfo('Entering Mission Mode...')
    while not rospy.is_shutdown() and not drone.set_mode('AUTO.MISSION'):
        rospy.sleep(0.1)

    rospy.loginfo('Arming...')
    while not rospy.is_shutdown() and not drone.set_arm(True):
        rospy.sleep(0.1)

    rospy.loginfo('Waiting for mission to complete...')
    while not rospy.is_shutdown() and not drone.is_mission_completed():
        rospy.sleep(0.1)

    rospy.loginfo('Waypoint mission complete...')
    
    drone.magical_shape()

    waypoints = [drone.create_waypoint(3, 16, False, 0, 10, 0, float('nan'), 47.398109, 8.548957, 10.0),
                 drone.create_waypoint(3, 16, False, 0, 10, 0, float('nan'), 47.397478, 8.548749, 10.0),
                 drone.create_waypoint(3, 21, False, 0, 0, 0, float('nan'), 47.398290, 8.545723, 0.0)]
    
    rospy.loginfo('Clearing Old Waypoints...')
    while not rospy.is_shutdown() and not drone.clear_waypoints():
        rospy.sleep(0.1)

    rospy.loginfo('Sending New Waypoints...')
    while not rospy.is_shutdown() and not drone.send_waypoints(waypoints):
        rospy.sleep(0.1)

    rospy.loginfo('Entering Mission Mode...')
    while not rospy.is_shutdown() and not drone.set_mode('AUTO.MISSION'):
        rospy.sleep(0.1)

    rospy.loginfo('Waiting for mission to complete...')
    while not rospy.is_shutdown() and not drone.is_mission_completed():
        rospy.sleep(0.1)

    rospy.loginfo('Waypoint mission complete...')