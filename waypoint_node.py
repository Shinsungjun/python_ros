import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode\
from pymavlink import mavutil



class WayPointNode():
    def __init__(self):
        rospy.init_node('waypoint_node', anonymous = True)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.local_pos_pub = rospy.Publisher('mavros/local_position/pose', PoseStamped, queue_size=10)

        self.current_state = State()
        self.rate = rospy.Rate(10)

    def mission(self):
        #wait for FCU connection
        while not rospy.is_shutdown() and not(self.current_state.connected):
            rospy.spin()
        
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2

        self.wait_msg_srv()
        
        offb_set_mode = SetMode()
        offb_set_mode.request.custom_mode = "OFFBOARD"

        arm_cmd = CommandBool()
        arm_cmd.request.value = True
        last_request = rospy.get_rostime()

        while not rospy.is_shutdown():
            if (self.current_state.mode != "OFFBOARD" and (rospy.get_rostime() - last_request > rospy.Duration.from_sec(5.0))):
                if (self.set_mode_client(offb_set_mode) and offb_set_mode.response.mode_sent):
                    rospy.loginfo("OFFBOARD enabled")
                last_request = rospy.get_rostime()
            else:
                if((not self.current_state.armed) and (rospy.get_rostime() - last_request > rospy.Duration.from_sec(5.0))):
                    if (self.arming_client(arm_cmd) and arm_cmd.response.success):
                        rospy.loginfo("Vehicle armed")
                    last_request = rospy.get_rostime()

        self.local_pos_pub.publish(pose)

        self.wait_msg_srv()

    def wait_msg_srv(self):
        try:
            rospy.wait_for_message('mavros/state', State, timeout=None)
            rospy.wait_for_service('mavros/set_mode', 30)
            rospy.wait_for_service('mavros/cmd/arming', 30)
        except rospy.ROSException:
            rospy.loginfo("failed to connect to service or message")

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.current_state = data
    


if __name__ == "__main__":
    waypoint_node = WayPointNode
    waypoint_node.mission()