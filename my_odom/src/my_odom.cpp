// ファイル名　my_odom.cpp
#include <ros/ros.h>  // rosで必要はヘッダーファイル
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h> 
#include <sensor_msgs/JointState.h>
#include <iostream>

using namespace std;

// コールバック関数。並進、回転速度の表示。
void cbVel(const geometry_msgs::Twist::ConstPtr& vel) {
  cout << "Linear :" << vel->linear.x << endl;
  cout << "Angular:" << vel->angular.z << endl;
}


// /odomトピックから位置posと姿勢poseを表示
void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: %d", msg->header.seq);
  ROS_INFO("/create1/odom Pos (x:%f, y:%f, z:%f)", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);  
  // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("/create1/odom Pose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
  ROS_INFO("Vel (Linear:%f, Angular:%f)", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
												  
}

// /gazebo/model_statesトピックから真の位置Pos(x,y,z)と姿勢Pose(roll, pitch ,yaw)を表示
void cbModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  ROS_INFO("Real Pos (x:%f, y:%f, z:%f)", msg->pose[2].position.x,msg->pose[2].position.y, msg->pose[2].position.z);

  tf::Quaternion q(msg->pose[2].orientation.x, msg->pose[2].orientation.y, msg->pose[2].orientation.z, msg->pose[2].orientation.w);  
  // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("Real Pose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
												  
}

// cbMyOdom：この関数に自分のオドメトリを実装しよう！
// /joint_statesトピックから左右のjoint(車輪回転軸)の位置（回転角度)[rad]を表示
// 参考：Turtlebot3の車輪直径0.066 [m]
void cbMyOdom(const sensor_msgs::JointState::ConstPtr& jointstate)
{
  double wheel_right_joint_pos = jointstate->position[0]; // 右車軸の位置[rad]
  double wheel_left_joint_pos  = jointstate->position[1]; // 左車軸の位置[rad]

  // 車軸の位置は積算される
  ROS_INFO("Whell Pos (r:%f, l:%f)", wheel_right_joint_pos,wheel_left_joint_pos);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_odom");
  ros::NodeHandle nh;

  //subscriberの作成。トピック/cmd_velを購読する。
  ros::Subscriber sub  = nh.subscribe("/create1/cmd_vel", 10, cbVel);
  ros::Subscriber sub2 = nh.subscribe("/create1/odom", 100, cbOdom);
  ros::Subscriber sub3 = nh.subscribe("/gazebo/model_states", 100, cbModelStates);
  ros::Subscriber sub4 = nh.subscribe("/create1/joint_states", 100, cbMyOdom);
  
  
  // コールバック関数を繰り返し呼び出す。
  ros::Rate rate(100);

  while (ros::ok()) {

    
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
