#include "ros/ros.h"  // rosで必要はヘッダーファイル
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

using namespace std;

std_msgs::Float64 tmp_joint1, tmp_joint2;
double pos_x, pos_y, pos_z;


void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  //msg->pose.pose.position, msg->pose.pose.orientation, 
  pos_x = msg->pose.pose.position.x;
  pos_y = msg->pose.pose.position.y;
  pos_z = msg->pose.pose.position.z;
  ROS_INFO("Pose: x=%f y=%f \n",pos_x,pos_y);
}

void monitorJointState(const sensor_msgs::JointState::ConstPtr& jointstate)
{
  tmp_joint1.data = jointstate->position[0];
  tmp_joint2.data = jointstate->position[1]; 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_sensor"); 
  // initでROSを初期化して、my_teleopという名前をノードにつける                        
  // 同じ名前のノードが複数あってはいけないので、ユニークな名前をつける

  ros::NodeHandle nh;
  // ノードハンドラの作成。ハンドラは必要になったら起動される。

  ros::Publisher  pub_joint1, pub_joint2;
  // パブリッシャの作成。トピックに対してデータを送信。

  ros::Subscriber sub_joints, sub_sensor;
  // サブスクライバの作成

  ros::Rate rate(10);
  // ループの頻度を設定。この場合は10Hz、1秒間に10回数、1ループ100ms。

  std_msgs::Float64 target_joint1, target_joint2;


  pub_joint1 = nh.advertise<std_msgs::Float64>("/armbot2_sensor/joint1_position_controller/command", 100);
  pub_joint2 = nh.advertise<std_msgs::Float64>("/armbot2_sensor/joint2_position_controller/command", 100);
  sub_sensor = nh.subscribe<nav_msgs::Odometry>("/armbot2_sensor/pose_ground_truth", 100, groundTruthCallback);
  sub_joints = nh.subscribe("/armbot2_sensor/joint_states", 100, monitorJointState);

  target_joint1.data = 0;
  target_joint2.data = 0;

  int loop = 0;
  while (ros::ok()) { // このノードが使える間は無限ループ
    char key;  // 入力キーの値

    ROS_INFO("[Input] j: Joint1++, f: Joint1--, k: Joint2++, d:Joint2--");
    cin >> key; 
    cout << key << endl;

    switch (key) {
    case 'j': target_joint1.data  +=  5 * M_PI/180.0; break;
    case 'f': target_joint1.data  -=  5 * M_PI/180.0; break;
    case 'k': target_joint2.data  +=  5 * M_PI/180.0; break;
    case 'd': target_joint2.data  -=  5 * M_PI/180.0; break;
    default: ROS_INFO("Input j,f,k,d");
    }
      
    pub_joint1.publish(target_joint1); // 角度を送信    
    pub_joint2.publish(target_joint2);
    ROS_INFO("Targe: Joint1=%f Joint2=%f", target_joint1.data, target_joint2.data);

    usleep(1000*1000);
    ros::spinOnce(); // コールバック関数を呼ぶ
    ROS_INFO("Tmp:   Joint1=%f Joint2=%f", tmp_joint1.data,    tmp_joint2.data);
    //rate.sleep();     // 指定した周期でループするよう寝て待つ
  }
  
  return 0;
}
