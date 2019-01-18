import rospy
import mav_msgs.msg as mav_msgs


def simple_commander():
  pub = rospy.Publisher('/firefly/command/motor_speed', mav_msgs.Actuators, queue_size=10)
  rospy.init_node('commander', anonymous=True)
  rate = rospy.Rate(1)  # 10hz
  while not rospy.is_shutdown():
    tmp_msg = mav_msgs.Actuators()
    tmp_msg.angular_velocities = [500, 100, 100, 100, 100, 100]
    # rospy.loginfo(tmp_msg)
    pub.publish(tmp_msg)
    rate.sleep()


if __name__ == '__main__':
  try:
    simple_commander()
  except rospy.ROSInterruptException:
    pass
