#!/usr/bin/env python

import rospy
import roslaunch

from std_msgs.msg import Bool





#THIS IS NOT A GOOD ENOUGH SOLUTION FOR THIS PROBLEM AS THE MAIN THREAD 
#GETS LOCKED BY THE LAUNCH, MAKING ALL OTHER TASKS LATER UNAVAILABLE!!!!




#class responsible for all ORB-SLAM functionality
class ORBSLAM_Control():

  def __init__(self):
 
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nek/aeolus_phase2_ws/src/orbslam-map-saving-extension/orb_slam2_ros/launch/orb_slam_d455_stereo.launch"])
    launch.start()
    rospy.loginfo("STARTED LAUNCH FROM PY-NODE!!")

    '''
    cli_args = ['aeolus2', 'orbslam_control.launch', 'trigger_sub:=/fbow_ros/nav_trigger', 'reloc_sub:=/test']
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    roslaunch_args = cli_args[2:]
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, roslaunch_args=[roslaunch_args])
    launch.start()
    rospy.loginfo("started")
    '''

    rospy.loginfo("WAITING SOME SECONDS!!")
    rospy.sleep(5)#sec

    rospy.loginfo("SHUTTING DOWN LAUNCH!!")
    launch.shutdown()



  #Callback for navigating back trigger 
  def triggerCb(self, msg):
    self.trigger_msg = msg
    rospy.loginfo("AAAAAAAAAAAAAAAAAAAAAA!!")

  #initialize subscribers
  def run(self):
    trigger_sub = rospy.Subscriber('/fbow_ros/nav_trigger', Bool, self.triggerCb)
    rospy.spin()

if __name__ == '__main__':

  rospy.init_node('orbslam_control_node', anonymous=True)

  try:
    oc = ORBSLAM_Control()
    oc.run()
  except rospy.ROSInterruptException:
    pass
