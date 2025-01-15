#include <ros/ros.h>

#include <thread>
#include <atomic>
#include <chrono>

#include <future> // std::async, std::future

#include <map>

#include "std_msgs/Bool.h"


using namespace std;




class ORBSLAM2_Control
{
	//Create a node handler it is reference assigned to a new node
	ros::NodeHandle nh_;
	ros::Subscriber goback_trigger_flag;


  private:
    map<int, string> orbslam_states= {{1, "SLAM"}, {2, "Localization"}};//map of states for ORB-SLAM

    string topic_to_sub;

	public:

    std_msgs::Bool goback_trigger_msg_cached;
    std::atomic<int> goback_trigger_data_{0};

    ORBSLAM2_Control()
    {

      //get node name and namespace
      string node_name =  ros::this_node::getName();
      string node_namespace =  ros::this_node::getNamespace();
      string path_to_file = node_name + node_namespace;

      //get args from parameter server
      //nh_.param<std::string>((path_to_file+"topic_to_sub").c_str(), topic_to_sub, "");

      //trigger to navigate back
      goback_trigger_flag = nh_.subscribe("/fbow_ros/goback_trigger", 10, &ORBSLAM2_Control::gobackTriggerCb, this);

      //initial value
      goback_trigger_msg_cached.data = 0;

    }

//---------------------------- Topic callbacks ---------------------------//


    void gobackTriggerCb(const std_msgs::Bool::ConstPtr& msg)
    {

      goback_trigger_msg_cached = *msg;
      if(goback_trigger_msg_cached.data==1)
        {
          goback_trigger_data_ = 1;
          cout << "GOT in callback: bbbbbb" << endl;
          //check if you can relocalise with ORB-SLAM
        }
        else
        {
          goback_trigger_data_ = 0;
          cout << "GOT in callback: aaaaaa" << endl;
        }


    }

//-----------------------------------------------------------------------//

  /*
    //publish new transfored ar poses as ROS msg
    void controlOrbSlam()
    {

      ros::Rate rate(30);//rate in Hz

      //
      int current_orbslam_state = 1;
      int flag = -1;

      while(ros::ok())
      {

        //if the trigger goes to 1 we need to navigate back, and thus we do ORB-SLAM's relocalization
        if(goback_trigger_data_==1)
        {
          if(flag!=1)
          {
            
            cout << "aaaaaa" << endl;

            int sys_state = system("rosnode kill orb_slam2"); //stop
            if(sys_state==-1)
                ROS_WARN("Something went wrong with stopping the launch!!");
          
          }  

          flag=1;
          //check if you can relocalise with ORB-SLAM
        }
        else
        {
          if(flag!=0)
          { 
             
            cout << "bbbbbb" << endl;
            int sys_state = system("roslaunch orb_slam2_ros orb_slam_d455_stereo.launch");  //start
            if(sys_state==-1)
              ROS_WARN("Something went wrong with launching!!");
          }


          flag=0;
          //we are doing our navigation so we use ORB-SLAM mapping and pose tracking
        }


        // system("roslaunch rplidar_ros rplidar.launch");  //start

        // system("rosnode kill orb_slam2");              //stop



        // Spin and Sleep
        ros::spinOnce();
        rate.sleep();


      }


    }
    */



    //start ORB-SLAM node
    void orbSlamStart()
    {
      // while(true)
      // {
      //   cout << "aaaaaa" << endl;
      //   std::this_thread::sleep_for(std::chrono::seconds(2));
      // }
      int sys_state = system("roslaunch orb_slam2_ros orb_slam_d455_stereo.launch");  //start
      if(sys_state==-1)
        ROS_WARN("Something went wrong with launching!!");

    }

    //stop ORB-SLAM node
    void orbSlamStop()
    {
      // while(true)
      // {
      //   cout << "bbbbbb" << endl;
      //   std::this_thread::sleep_for(std::chrono::seconds(1));
      // }
        
      int sys_state = system("rosnode kill orb_slam2"); //stop
      if(sys_state==-1)
          ROS_WARN("Something went wrong with stopping the launch!!");

    }



};



    void orbSlamStartt()
    {
      while(true)
      {
        cout << "aaaaaa" << endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
      }
      // int sys_state = system("roslaunch orb_slam2_ros orb_slam_d455_stereo.launch");  //start
      // if(sys_state==-1)
      //   ROS_WARN("Something went wrong with launching!!");

    }
    //stop ORB-SLAM node
    void orbSlamStopp()
    {
      while(true)
      {
        cout << "bbbbbb" << endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
        
      // int sys_state = system("rosnode kill orb_slam2"); //stop
      // if(sys_state==-1)
      //     ROS_WARN("Something went wrong with stopping the launch!!");

    }

//--------------------//
//------  MAIN  ------//
//--------------------//
int main(int argc, char **argv)
{

	ros::init(argc, argv, "orbslam2_relocalization_control_node");


  ros::AsyncSpinner spinner(3); // Use # threads

  ORBSLAM2_Control oc;//create main class object


  spinner.start();


  //----------------------------------//
  //--- simple thread test -----//
  //.join()
  // Blocks the current thread until the thread identified by *this finishes its execution.
  // The completion of the thread identified by *this synchronizes with the corresponding successful return from join().
  // No synchronization is performed on *this itself. Concurrently calling join() on the same thread object from multiple threads constitutes a data race that results in undefined behavior.

  //effectively, without the join i dont lock on a thread and i can continue to other threads too


  // cout << "111" << endl;
  // std::thread ORBSLAM_start_thread(orbSlamStartt);
  // std::thread ORBSLAM_stop_thread(orbSlamStopp);
  // cout << "after 111 was called" << endl;
  // cout << "after 22 was called" << endl;

  // //ORBSLAM_start_thread.join(); //care for lock
  // //ORBSLAM_stop_thread.join();


  //----------------------------------//
  //--- simple thread initialization in if/while test -----//
  //if you declare thread in if/while you get core aborted error right away... looks like you need to find another way
  //declaring outside of if/while and then initializing inside can work. However, no reinitialization of the same thread can happen as i get core dumped


  // std::thread ORBSLAM_start_thread;
  // std::thread ORBSLAM_stop_thread;

  // int flag = -1;
  // if(flag!=0)
  // {
  //   cout << "111" << endl;
  //   ORBSLAM_start_thread = std::thread(&ORBSLAM2_Control::orbSlamStart, &oc);
  //   cout << "after 111 was called" << endl;
  // }  



  //----------------------------------//
  //--- while simple thread test1 -----//
  //this thing locks the main while loop below whenever i launch orbslam(which is essentially a big WHILE)... 
  //orbslam starts, i can also get the new messages from the topics, but the loop stucks and i cant go through..

  //NEEDS UPDATE BECAUSE OF JOIN 

  /*
  ros::Rate rate(30);//rate in Hz


  int flag = -1;

  while(ros::ok())
  {

    //if the trigger is 0, we are doing our navigation so we use ORB-SLAM mapping and pose tracking
    if(oc.goback_trigger_data_==0)
    {
      if(flag!=0)
      {
        cout << "111" << endl;
        std::thread ORBSLAM_start_thread(&ORBSLAM2_Control::orbSlamStart, &oc);
        ORBSLAM_start_thread.join();
      }  
      flag=0;
      //check if you can relocalise with ORB-SLAM
    }
    else //if the trigger goes to 1 we need to navigate back, and thus we do ORB-SLAM's relocalization
    {
      if(flag!=1)
      { 
        cout << "222" << endl;
        std::thread ORBSLAM_stop_thread(&ORBSLAM2_Control::orbSlamStop, &oc);
        ORBSLAM_stop_thread.join();
      }
      flag=1;
    }


    // Spin and Sleep
    ros::spinOnce();
    rate.sleep();


  }
  */
  


  //----------------------------------//
  //--- while simple thread test2 -----//
  //declaring outside of if/while and then initializing inside can work. However, no reinitialization of the same thread can happen as i get core dumped

  /*
  std::thread ORBSLAM_start_thread;
  std::thread ORBSLAM_stop_thread;
  ros::Rate rate(30);//rate in Hz


  int flag = -1;

  while(ros::ok())
  {

    //if the trigger is 0, we are doing our navigation so we use ORB-SLAM mapping and pose tracking
    if(oc.goback_trigger_data_==0)
    {
      if(flag!=0)
      {
        cout << "111" << endl;
        ORBSLAM_start_thread = std::thread(&ORBSLAM2_Control::orbSlamStart, &oc);
        cout << "after 111 was called" << endl;
        //ORBSLAM_start_thread.join();
      }  
      flag=0;
      //check if you can relocalise with ORB-SLAM
    }
    else //if the trigger goes to 1 we need to navigate back, and thus we do ORB-SLAM's relocalization
    {
      if(flag!=1)
      { 
        cout << "222" << endl;
        ORBSLAM_stop_thread = std::thread(&ORBSLAM2_Control::orbSlamStop, &oc);
        cout << "after 222 was called" << endl;
        //ORBSLAM_stop_thread.join();
      }
      flag=1;
    }


    // Spin and Sleep
    ros::spinOnce();
    rate.sleep();
  }
  */




  //----------------------------------//
  //--- while simple thread test3 -----//
  //declaring outside of if/while and then initializing inside can work.
  //detaching lets me reinitialize the already declared thread but keep in mind that the previous thread doesn't die!! It will stay alive on the background!

  std::thread ORBSLAM_start_thread;
  std::thread ORBSLAM_stop_thread;
  ros::Rate rate(30);//rate in Hz


  int trigger_flag = -1;

  while(ros::ok())
  {

    //if the trigger is 0, we are doing our navigation so we use ORB-SLAM mapping and pose tracking
    if(oc.goback_trigger_data_==0)
    {
      if(trigger_flag!=0)
      {
        cout << "111" << endl;
        ORBSLAM_start_thread = std::thread(&ORBSLAM2_Control::orbSlamStart, &oc);
        cout << "after 111 was called" << endl;
        ORBSLAM_start_thread.detach();
      }  
      trigger_flag=0;
      //check if you can relocalise with ORB-SLAM
    }
    else //if the trigger goes to 1 we need to navigate back, and thus we do ORB-SLAM's relocalization
    {
      if(trigger_flag!=1)
      { 
        cout << "222" << endl;
        ORBSLAM_stop_thread = std::thread(&ORBSLAM2_Control::orbSlamStop, &oc);
        cout << "after 222 was called" << endl;
        ORBSLAM_stop_thread.detach();
      }
      trigger_flag=1;
    }


    // Spin and Sleep
    ros::spinOnce();
    rate.sleep();
  }








  //----------------------------------//
  //--- while async thread test -----//
  //this thing locks the main while loop below whenever i launch orbslam(which is essentially a big WHILE)... 
  //orbslam starts, i can also get the new messages from the topics, but the loop stucks and i cant go through..


  // ros::Rate rate(10);//rate in Hz


  // int flag = -1;

  // while(ros::ok())
  // {
  //   cout << "looking for next call" << endl;
  //   //if the trigger is 0, we are doing our navigation so we use ORB-SLAM mapping and pose tracking
  //   if(oc.goback_trigger_data_==0)
  //   {
  //     if(flag!=0)
  //     {
  //       cout << "111" << endl;
  //       std::async(launch::async, &ORBSLAM2_Control::orbSlamStart, &oc);
  //       cout << "after 111 was called" << endl;
  //     }  
  //     flag=0;
  //     //check if you can relocalise with ORB-SLAM
  //   }
  //   else //if the trigger goes to 1 we need to navigate back, and thus we do ORB-SLAM's relocalization
  //   {
  //     if(flag!=1)
  //     { 
  //       cout << "222" << endl;
  //       std::async(launch::async, &ORBSLAM2_Control::orbSlamStop, &oc);
  //     }
  //     flag=1;
  //   }


  //   // Spin and Sleep
  //   ros::spinOnce();
  //   rate.sleep();


  // }







  //ORBSLAM_thread.join();


// simulate expensive operation
//std::this_thread::sleep_for(std::chrono::seconds(1));


	//ros::spin();// Enter a loop, pumping callbacks
  ros::waitForShutdown();//async
	return 0;

}
