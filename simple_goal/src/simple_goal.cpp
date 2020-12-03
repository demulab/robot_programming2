// 本プログラムは
// http://wiki.ros.org/ja/navigation/Tutorials/SendingSimpleGoals
// を改変しています。
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct MyPose {
  double x;
  double y;
  double yaw;
};

int main(int argc, char** argv){
  // 座標はmap座標系
  MyPose way_point[] = {{-1, 4, - 0.5 * M_PI}, {-1, 0.5, M_PI}, 
  						{1,  0, - 0.5 * M_PI},{-3, 4, 0.0 * M_PI},{999, 999, 999}};

  ros::Publisher  pub;

  ros::init(argc, argv, "simple_goal");

  ros::NodeHandle nh;
  pub= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2, true);

  // 時間の取得
  ros::Time tmp_time = ros::Time::now();
  //create msg
  geometry_msgs::PoseWithCovarianceStamped initPose;

  // ロボット初期位置の設定
  // ~catkin_ws/src/create_autonomy/ca_gazebo/launch/create_house.launch
  //  ファイル内の2～3行目create1/amcl/initial_pose_x, initial_pose_yに合わせる
  initPose.header.stamp = tmp_time;  //  時間
  initPose.header.frame_id = "map";  //  フレーム
  initPose.pose.pose.position.x =  -3;
  initPose.pose.pose.position.y =  4
  initPose.pose.pose.position.z =  0;
  initPose.pose.pose.orientation.w = 1.0;

  // パブリッシュ amclパッケージに初期位置を送る
  pub.publish(initPose);
  
  // アクションクライアンを作成。1番目の引数は接続するアクションサーバー名。
  // アクションサーバーが立ち上がっていないとだめ。
  // ２番目の引数はtrueならスレッドを自動的に回す(ros::spin()。
  //MoveBaseClient ac("move_base", true);
  MoveBaseClient ac("create1/move_base", true);
  // アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。
  // この例では５秒間待つ(ブロックされる)
  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("The server comes up");
  move_base_msgs::MoveBaseGoal goal;
  // 目標位置姿勢の座標系（map座標系)
  goal.target_pose.header.frame_id = "map";
  // 現在時刻                                                                       
  goal.target_pose.header.stamp = ros::Time::now();


  int i = 0;
  while (ros::ok()) {
    // ROSではロボットの進行方向がx座標、左方向がy座標、上方向がz座標
    goal.target_pose.pose.position.x =  way_point[i].x;
    goal.target_pose.pose.position.y =  way_point[i].y;
    goal.target_pose.pose.orientation.w = 1; 

    ROS_INFO("Sending goal: No.%d", i+1);
    // サーバーにgoalを送信
    ac.sendGoal(goal);

    // 結果が返ってくるまで30.0[s] 待つ。ここでブロックされる。
    bool succeeded = ac.waitForResult(ros::Duration(30.0));

    // 結果を見て、成功ならSucceeded、失敗ならFailedと表示
    actionlib::SimpleClientGoalState state = ac.getState();

    if(succeeded) {
      ROS_INFO("Succeeded: No.%d (%s)",i+1, state.toString().c_str());
    }
    else {
      ROS_INFO("Failed: No.%d (%s)",i+1, state.toString().c_str());
    }
    i++;
  }
  return 0;
}
