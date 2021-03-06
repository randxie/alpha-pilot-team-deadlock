import rospy
import mav_msgs.msg as mav_msgs
import sensor_msgs.msg as s_msgs


def simple_commander():
  pub = rospy.Publisher('/uav/input/rateThrust', mav_msgs.RateThrust, queue_size=10)
  rospy.init_node('commander', anonymous=True)
  rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, callback)
  rate = rospy.Rate(1)  # 10hz
  while not rospy.is_shutdown():
    tmp_msg = mav_msgs.RateThrust()
    tmp_msg.thrust.z = 100
    rospy.loginfo(tmp_msg)
    pub.publish(tmp_msg)
    rate.sleep()

def callback(data):
  rospy.loginfo(data.range)

def listener():
  rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', s_msgs.Range, callback)
  rospy.spin()

if __name__ == '__main__':
  try:
    simple_commander()
  except rospy.ROSInterruptException:
    pass
