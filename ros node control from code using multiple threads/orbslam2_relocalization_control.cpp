#include <ros/ros.h>

#include <thread>
#include <atomic>
#include <chrono>
#include <future> // std::async, std::future

#include <map>

#include "std_msgs/Bool.h"

#include <aeolus2/navigate_back.h>


using namespace std;




class ORBSLAM2_Control
{
	//Create a node handler
	ros::NodeHandle nh_;
	ros::Subscriber goback_trigger_sub_;

  private:
    map<int, string> orbslam_states= {{1, "SLAM"}, {2, "Localization"}};//map of states for ORB-SLAM
    string topic_to_sub;

	public:

    aeolus2::navigate_back goback_trigger_msg_cached;
    std::atomic<int> goback_trigger_data_{0};//bool about going back shared between threads
    std::atomic<int> relocalization_trigger_data_{0};//bool about relocalization shared between threads

    ORBSLAM2_Control()
    {

      //get node name and namespace
      string node_name =  ros::this_node::getName();
      string node_namespace =  ros::this_node::getNamespace();
      string path_to_file = node_name + node_namespace;

      //get args from parameter server
      //nh_.param<std::string>((path_to_file+"topic_to_sub").c_str(), topic_to_sub, "");

      //trigger to navigate back
      goback_trigger_sub_ = nh_.subscribe("/fbow_ros/goback_trigger", 10, &ORBSLAM2_Control::navigateBackCb, this);

      //initial value
      goback_trigger_msg_cached.navigate_back.data = 0;

    }

//---------------------------- Topic callbacks ---------------------------//

    void navigateBackCb(const aeolus2::navigate_back::ConstPtr& msg)
    {

      goback_trigger_msg_cached = *msg;

      //update goback trigger based on message
      //once it goes to 1 it locks to that value
      if(goback_trigger_msg_cached.navigate_back.data==1 && goback_trigger_data_==0)
        goback_trigger_data_ = 1;


      //update relocalization trigger based on message
      if(goback_trigger_msg_cached.relocalization.data==1)
        relocalization_trigger_data_ = 1;
      else
        relocalization_trigger_data_ = 0;

    }

//-----------------------------------------------------------------------//


    //start ORB-SLAM node for SLAM mode
    void orbSlamStartSLAM()
    {

      int sys_state = system("roslaunch orb_slam2_ros orb_slam_d455_stereo_SLAM.launch");  //start SLAM
      if(sys_state==-1)
        ROS_WARN("Something went wrong with launching!!");

    }

    //start ORB-SLAM node for relocalization mode
    void orbSlamStartRelocalization()
    {

      int sys_state = system("roslaunch orb_slam2_ros orb_slam_d455_stereo_relocalization.launch");  //start relocalization
      if(sys_state==-1)
        ROS_WARN("Something went wrong with launching!!");

    }


    //stop ORB-SLAM node
    void orbSlamStop()
    {
    
      int sys_state = system("rosnode kill orb_slam2"); //stop
      if(sys_state==-1)
          ROS_WARN("Something went wrong with stopping the launch!!");

    }


    //make thread to sleep/wait for some seconds
    void threadSleep(int sec_num)
    {
      std::this_thread::sleep_for(std::chrono::seconds(sec_num));
    }



//-----------------------------------------------------------------------//

};



//--------------------//
//------  MAIN  ------//
//--------------------//
int main(int argc, char **argv)
{

	ros::init(argc, argv, "orbslam2_relocalization_control_node");


  ros::AsyncSpinner spinner(3); // Use # threads

  ORBSLAM2_Control oc;//create main class object

  spinner.start();


  //declaring 2 threads, one for stopping and one for starting ORBSLAM
  //detaching lets me reinitialize the already declared thread but keep in mind that the previous thread doesn't die!! It will stay alive on the background!
  std::thread ORBSLAM_start_thread;
  std::thread ORBSLAM_stop_thread;

  //this flag is controlling a small state machine which has the following transitions: 0->1, 1->2, 2->1
  int goback_trigger_flag = 0;//-1=initialized, 0=SLAM, 1=stop ORBSLAM, 2=relocalization

  ros::Time cached_nav_back_timestamp_;//to check if we received a new navigate-back msg


  ros::Rate rate(30);//rate in Hz


  while(ros::ok())
  {

    //if go-back trigger is 0, we use ORB-SLAM for mapping and pose tracking (SLAM)
    if(oc.goback_trigger_data_==0)
    {
      //coming just from initialization of the flag
      if(goback_trigger_flag==0)
      {
        //cout << "111" << endl;
        ORBSLAM_start_thread = std::thread(&ORBSLAM2_Control::orbSlamStartSLAM, &oc);
        //cout << "after 111 was called" << endl;
        ORBSLAM_start_thread.detach();

        goback_trigger_flag=1;//update flag so next state is STOP
        oc.threadSleep(1);//make thread to sleep/wait for some seconds so we can wait for the process to initialize safely
      }  
    }
    else//if go-back trigger msg is 1, we need to navigate back, and use ORB-SLAM for relocalization
    {
      if(goback_trigger_flag==1)
      { 
        //cout << "222" << endl;
        ORBSLAM_stop_thread = std::thread(&ORBSLAM2_Control::orbSlamStop, &oc);
        //cout << "after 222 was called" << endl;
        ORBSLAM_stop_thread.detach();

        goback_trigger_flag=2;//update flag so next state is RELOCALIZE
        oc.threadSleep(2);//make thread to sleep/wait for some seconds so we can wait for the process to initialize safely
        continue;//go to next iteration
      }

      //if we got a message to relocalize
      if(goback_trigger_flag==2 && oc.relocalization_trigger_data_==1 && cached_nav_back_timestamp_!=oc.goback_trigger_msg_cached.header.stamp)
      {
        //cout << "333" << endl;
        ORBSLAM_start_thread = std::thread(&ORBSLAM2_Control::orbSlamStartRelocalization, &oc);
        //cout << "after 333 was called" << endl;
        ORBSLAM_start_thread.detach();

        goback_trigger_flag=1;//update flag so next state is STOP
        cached_nav_back_timestamp_ = oc.goback_trigger_msg_cached.header.stamp;//update last used msg's stamp
        oc.threadSleep(4);//make thread to sleep/wait for some seconds. This will be the time allowed by the system to do the relocalization with ORBSLAM. After that we stop the thread
      }
      
    }


    // Spin and Sleep
    ros::spinOnce();
    rate.sleep();
  }




  ros::waitForShutdown();

	return 0;
}
