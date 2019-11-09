import rospy
import sensor_msgs.msg as s_msgs


def callback(data):
  rospy.loginfo(data.range)


def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, callback)
  rospy.spin()


if __name__ == '__main__':
  listener()