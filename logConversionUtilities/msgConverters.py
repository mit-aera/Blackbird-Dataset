"""Classes for converting LCM messages to ROS messages."""
__author__ = "Amado Antonini"

import rospy
# import tf
import numpy as np
import quaternion
import sys
sys.path.append('../../build/lcmtypes/python')
import agile
# ros messages
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu, CameraInfo
from tf.msg import tfMessage
# custom messages
from msg import MotorRPM
from msg import MotorPWM


class Converter(object):
    """
    Parent class that implements common functions for converter classes.

    Attributes:
        count (int): The number of times a message has been seen (for use in Header.seq of ros msg)
        frame_id (str): The frame_id for the ros msg
        rostopic (str): The topic for the ros msg
    """

    def __init__(self, channel_settings):
        self.count = 0
        self.frame_id = channel_settings.get("frame_id", "0")
        self.rostopic = channel_settings["rostopic"]

    def update_count(self):
        self.count += 1
        return self.count

    def make_header(self, utime):
        header = Header()
        header.seq = self.update_count()
        header.stamp = self.utime_to_stamp(utime)
        header.frame_id = self.frame_id
        
        return header

    @staticmethod
    def utime_to_stamp(utime):
        nsecs = int(utime*1000)
        
        stamp = rospy.Time()
        stamp.secs = int(nsecs*10**-9)
        stamp.nsecs = int(nsecs - (stamp.secs*10**9)) 

        return stamp


class PoseMoCapConverter(Converter):
    """
    Converter for lcm 'poseMoCap' messages into ros PoseStamped messages.
    Should publish pose message for each mocap pose.
    Should publish TF tree message for each mocap pose.
    Should publish camera info message for each mocap pose that matches a camera frame.

    """

    def __init__(self, channel_settings, image_timestamps):
        super(PoseMoCapConverter, self).__init__(channel_settings)
        self.image_timestamps = image_timestamps
        
    def __call__(self, channel, data):
        lcm_msg = agile.state_t.decode(data)

        # List of messages and topics to publish
        topics_to_publish = []
        msgs_to_publish = []

        # Populate pose message
        topics_to_publish.append(self.rostopic)
        ros_msg = PoseStamped()
        ros_msg.header = self.make_header(lcm_msg.utime)
        ros_msg.pose.position.x = lcm_msg.position[0]
        ros_msg.pose.position.y = lcm_msg.position[1]
        ros_msg.pose.position.z = lcm_msg.position[2]
        ros_msg.pose.orientation.w = lcm_msg.orient[0]
        ros_msg.pose.orientation.x = lcm_msg.orient[1]
        ros_msg.pose.orientation.y = lcm_msg.orient[2]
        ros_msg.pose.orientation.z = lcm_msg.orient[3]
        msgs_to_publish.append(ros_msg)

        # Populate TF tree message
        tf_packet = tfMessage()

        # Mocap NED <> Mocap ENU
        # topics_to_publish.append("/tf_static")
        # tf_ned_2_enu_msg = TransformStamped()
        # tf_ned_2_enu_msg.header = self.make_header(lcm_msg.utime)
        # tf_ned_2_enu_msg.header.frame_id = "mocap_ENU"
        # tf_ned_2_enu_msg.child_frame_id = "mocap_NED"
        # tf_ned_2_enu_msg.transform.translation = ros_msg.pose.position
        # tf_ned_2_enu_msg.transform.rotation = ros_msg.pose.orientation
        # msgs_to_publish.append(tf_ned_2_enu_msg)


        # [ 0, 0.7071068, 0.7071068, 0 ]

        # Mocap NED <> drone_body message
        tf_drone_msg = TransformStamped()
        tf_drone_msg.header = self.make_header(lcm_msg.utime)
        tf_drone_msg.header.frame_id = "mocap_NED"
        tf_drone_msg.child_frame_id = "body_frame"
        tf_drone_msg.transform.translation = ros_msg.pose.position
        tf_drone_msg.transform.rotation = ros_msg.pose.orientation
        
        # Append message to transform list
        tf_packet.transforms.append(tf_drone_msg)

        
        # Static TFs
        # drone_body <> IMU message
        # topics_to_publish.append("/tf_static")
        # tf_imu_msg = TransformStamped()
        # tf_imu_msg.header = self.make_header(lcm_msg.utime)
        # tf_imu_msg.header.frame_id = "body_frame"
        # tf_imu_msg.child_frame_id = "imu"
        # tf_imu_msg.transform.translation.x = 0.0
        # tf_imu_msg.transform.translation.y = 0.0
        # tf_imu_msg.transform.translation.z = 0.0
        # # [ 0.707479362748676   0.002029830683065  -0.007745228390866   0.706688646087723 ]
        # tf_imu_msg.transform.rotation.w = 0.707479362748676
        # tf_imu_msg.transform.rotation.x = 0.002029830683065
        # tf_imu_msg.transform.rotation.y = -0.007745228390866
        # tf_imu_msg.transform.rotation.z = 0.706688646087723
        # msgs_to_publish.append(tf_imu_msg)

        # # drone_body <> camera_l
        # topics_to_publish.append("/tf_static")
        # tf_cam_l_msg = TransformStamped()
        # tf_cam_l_msg.header = self.make_header(lcm_msg.utime)
        # tf_cam_l_msg.header.frame_id = "body_frame"
        # tf_cam_l_msg.child_frame_id = "camera_l"
        # tf_cam_l_msg.transform.translation.x = 0.0
        # tf_cam_l_msg.transform.translation.y = -0.05
        # tf_cam_l_msg.transform.translation.z = 0.0
        # tf_cam_l_msg.transform.rotation.w = 1.0
        # tf_cam_l_msg.transform.rotation.x = 0.0
        # tf_cam_l_msg.transform.rotation.y = 0.0
        # tf_cam_l_msg.transform.rotation.z = 0.0
        # msgs_to_publish.append(tf_cam_l_msg)

        # # drone_body <> camera_r
        # topics_to_publish.append("/tf_static")
        # tf_cam_r_msg = TransformStamped()
        # tf_cam_r_msg.header = self.make_header(lcm_msg.utime)
        # tf_cam_r_msg.header.frame_id = "body_frame"
        # tf_cam_r_msg.child_frame_id = "camera_r"
        # tf_cam_r_msg.transform.translation.x = 0.0
        # tf_cam_r_msg.transform.translation.y = 0.05
        # tf_cam_r_msg.transform.translation.z = 0.0
        # tf_cam_r_msg.transform.rotation.w = 1.0
        # tf_cam_r_msg.transform.rotation.x = 0.0
        # tf_cam_r_msg.transform.rotation.y = 0.0
        # tf_cam_r_msg.transform.rotation.z = 0.0
        # msgs_to_publish.append(tf_cam_r_msg)

        # # drone_body <> camera_d
        # topics_to_publish.append("/tf_static")
        # tf_cam_d_msg = TransformStamped()
        # tf_cam_d_msg.header = self.make_header(lcm_msg.utime)
        # tf_cam_d_msg.header.frame_id = "body_frame"
        # tf_cam_d_msg.child_frame_id = "camera_d"
        # tf_cam_d_msg.transform.translation.x = 0.0
        # tf_cam_d_msg.transform.translation.y = 0.0
        # tf_cam_d_msg.transform.translation.z = 0.0
        # # 0.707,0,-0.707,0
        # tf_cam_d_msg.transform.rotation.w = 0.707
        # tf_cam_d_msg.transform.rotation.x = 0.0
        # tf_cam_d_msg.transform.rotation.y = -0.707
        # tf_cam_d_msg.transform.rotation.z = 0.0
        # msgs_to_publish.append(tf_cam_d_msg)

        # Queue TF tree message for publication 
        topics_to_publish.append("/tf")
        msgs_to_publish.append(tf_packet)


        # Populate camera info message (if needed)
        if (lcm_msg.utime in self.image_timestamps):
            cam_info = CameraInfo()
            cam_info.header = self.make_header(lcm_msg.utime)
            cam_info.height = 768
            cam_info.width  = 1024
            cam_info.distortion_model = 'plumb_bob'
            cam_info.K = [665.107510106, 0., 511.5, 0., 665.107510106, 383.5, 0., 0., 1.]
            cam_info.P = [665.107510106, 0., 511.5, 0., 0., 665.107510106, 383.5, 0., 0., 0., 1., 0]

            for cam_topic in ["/camera_l/camera_info","/camera_r/camera_info","/camera_d/camera_info"]:
                topics_to_publish.append(cam_topic)
                msgs_to_publish.append(cam_info)



        return topics_to_publish, msgs_to_publish


class ImuConverter(Converter):
    """
    Converter for lcm 'imuRaw' messages into ros Imu messages.
    """

    def __init__(self, channel_settings):
        super(ImuConverter, self).__init__(channel_settings)
        self.angular_velocity_cov = channel_settings["covariances"]["angular_velocity"]
        self.linear_acceleration_cov = channel_settings["covariances"]["linear_acceleration"]

    def __call__(self, channel, data):
        lcm_msg = agile.imuRaw_t.decode(data)
        
        ros_msg = Imu()
        ros_msg.header = self.make_header(lcm_msg.utime)
        ros_msg.angular_velocity.x = lcm_msg.gyro[0]
        ros_msg.angular_velocity.y = lcm_msg.gyro[1]
        ros_msg.angular_velocity.z = lcm_msg.gyro[2]
        ros_msg.angular_velocity_covariance = self.angular_velocity_cov
        ros_msg.linear_acceleration.x = lcm_msg.accel[0]
        ros_msg.linear_acceleration.y = lcm_msg.accel[1]
        ros_msg.linear_acceleration.z = lcm_msg.accel[2]
        ros_msg.linear_acceleration_covariance = self.linear_acceleration_cov
        return [self.rostopic], [ros_msg]


class PoseRefConverter(Converter):
    """
    Converter for lcm 'poseRef' messages into ros PoseStamped messages.
    """

    def __init__(self, channel_settings):
        super(PoseRefConverter, self).__init__(channel_settings)

    def __call__(self, channel, data):
        lcm_msg = agile.poseRef_t.decode(data)

        ros_msg = PoseStamped()
        ros_msg.header = self.make_header(lcm_msg.utime)
        ros_msg.pose.position.x = lcm_msg.position[0]
        ros_msg.pose.position.y = lcm_msg.position[1]
        ros_msg.pose.position.z = lcm_msg.position[2]


        # quat_orient = tf.transformations.quaternion_from_euler(*lcm_msg.orientEuler)
        quat_orient = quaternion.from_euler_angles(*lcm_msg.orientEuler)
        ros_msg.pose.orientation.x = quat_orient.x
        ros_msg.pose.orientation.y = quat_orient.y
        ros_msg.pose.orientation.z = quat_orient.z
        ros_msg.pose.orientation.w = quat_orient.w
        return [self.rostopic], [ros_msg]


class RpmConverter(Converter):
    """Converter for lcm 'rpmFeedback' messages into ros MotorRPM messages."""
    def __init__(self, channel_settings):
        super(RpmConverter, self).__init__(channel_settings)

    def __call__(self, channel, data):
        lcm_msg = agile.motorsWs_t.decode(data)

        ros_msg = MotorRPM()
        ros_msg.header = self.make_header(lcm_msg.utime)
        ros_msg.sample_stamp = [self.utime_to_stamp(utime) for utime in lcm_msg.sample_us]
        ros_msg.rpm = lcm_msg.rpms
        return [self.rostopic], [ros_msg]


class PwmConverter(Converter):
    """Converter for lcm 'motorsPwms' messages into ros MotorPWM messages."""
    def __init__(self, channel_settings):
        super(PwmConverter, self).__init__(channel_settings)

    def __call__(self, channel, data):
        lcm_msg = agile.motorsPwms_t.decode(data)

        ros_msg = MotorPWM()
        ros_msg.header = self.make_header(lcm_msg.utime)
        ros_msg.pwm = lcm_msg.pwms
        return [self.rostopic], [ros_msg]


if __name__ == '__main__':
    print("Nothing to be done here.")
