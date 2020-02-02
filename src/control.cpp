/*
 * COPYRIGHT Pedro Paulo Ventura Tecchio 2020
*/

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


#include <actionlib/client/simple_action_client.h>
#include <nav2d_navigator/ExploreAction.h>
#include <nav2d_navigator/MoveToPosition2DAction.h>
#include <std_srvs/Trigger.h>
#include <nav2d_navigator/RobotNavigator.h>

#include <nav2d_navigator/commands.h>

#include <tf2/LinearMath/Quaternion.h>



class Control {
  ros::NodeHandle nh_;
  ros::Subscriber subCBD;
  ros::Subscriber subPSD;
  ros::Subscriber subOdom;
  ros::Publisher pubPoseStamped;
  std_msgs::Header cbdH;
  std_msgs::Header psdH;
  std_msgs::Header odomH;
  geometry_msgs::Point cbdP;
  geometry_msgs::Point psdP;
  geometry_msgs::PoseWithCovariance odomP;
  bool cbdState;
  bool psdState;
  bool explorationState;
  int robotState;
  ros::ServiceClient stopClient;
  ros::ServiceClient startClient;
  std_srvs::Trigger trigger;

  void cbdCB(const geometry_msgs::PointStamped& msg);
  void psdCB(const geometry_msgs::PointStamped& msg);
  void odomCB(const nav_msgs::Odometry& msg);

 public:
  Control();
  ~Control();
  void startExploration();
  void stop(void);
  void findBall(void);
};

Control::Control() {
  subCBD = nh_.subscribe("colored_ball_detector/yaw",
                         1,
                         &Control::cbdCB,
                         this);
  subPSD = nh_.subscribe("pcl_sphere_detector/sphere_center",
                         1,
                         &Control::psdCB,
                         this);
  subOdom = nh_.subscribe("odometry/filtered",
                         1,
                         &Control::odomCB,
                         this);
  pubPoseStamped = nh_.advertise<geometry_msgs::PoseStamped>
    ("goal", 1);

  startClient = nh_.serviceClient<std_srvs::Trigger>("StartExploration");
  stopClient = nh_.serviceClient<std_srvs::Trigger>("Stop");

  cbdP.x = 0.0;
  cbdP.y = 0.0;
  cbdP.z = 0.0;

  psdP.x = 0.0;
  psdP.y = 0.0;
  psdP.z = 0.0;

  cbdState = false;
  psdState = false;
  explorationState = false;

  robotState = 0;

}

Control::~Control() {
  // delete gMoveClient;
  ros::shutdown();
}


void Control::cbdCB(const geometry_msgs::PointStamped& msg) {
  cbdH = msg.header;
  cbdP = msg.point;
  cbdState = true;

  // ROS_INFO_STREAM("CBD Header: " << cbdH);
  // ROS_INFO_STREAM("CBD Point: "  << cbdP);
}

void Control::psdCB(const geometry_msgs::PointStamped& msg) {
  psdH = msg.header;
  psdP = msg.point;
  psdState = true;

  // ROS_INFO_STREAM("PSD Header: " << psdH);
  // ROS_INFO_STREAM("PSD Point: "  << psdP);
}

void Control::odomCB(const nav_msgs::Odometry& msg) {
  odomH = msg.header;
  odomP = msg.pose;

  // ROS_INFO_STREAM("Odom Header: " << odomH);
  // ROS_INFO_STREAM("Odom Pose: "   << odomP);
}


void Control::startExploration(void) {
  // gExploreClient->sendGoal(exploreGoal);
  startClient.call(trigger);
  explorationState = true;
}

void Control::stop(void) {
  stopClient.call(trigger);
  explorationState = false;
}


void Control::findBall(void) {
  double x , y;
  std_msgs::Header outputHeaderMsg;
  geometry_msgs::PoseStamped outputPoseStampedMsg;
  tf2::Quaternion odomQ;
  tf2::Quaternion cbdQ;
  tf2::Quaternion q;


  switch (robotState) {
  case 0:
    cbdP.x = 0.0;
    cbdP.y = 0.0;
    cbdP.z = 0.0;

    psdP.x = 0.0;
    psdP.y = 0.0;
    psdP.z = 0.0;

    cbdState = false;
    psdState = false;
    ROS_INFO_STREAM("Robot will start exploration!");
    if (!explorationState)
      startExploration();
    robotState = 1;
    break;
  case 1:
    // ROS_INFO_STREAM("Robot is looking for ball!");
    // if (cbdState && psdState) {
    //   ROS_INFO_STREAM("Robot found ball using CBD and PSD!");
    //   robotState = 2;
    // } else
    if (cbdState) {
      ROS_INFO_STREAM("Robot found ball using CBD!");
      robotState = 3;
    }
    // else if (psdState) {
    //   ROS_INFO_STREAM("Robot found ball using PSD!");
    //   robotState = 4;
    // } else {
    //   robotState = 1;
    // }
    break;
  case 2:
    if (explorationState)
      stop();
    break;
  case 3:
    if (explorationState)
      stop();
    cbdState = false;

    ROS_INFO_STREAM("Robot CBD moving robot!");
    outputHeaderMsg.seq = odomH.seq;
    outputHeaderMsg.stamp = ros::Time::now();
    outputHeaderMsg.frame_id = odomH.frame_id;

    x = odomP.pose.position.x + 1 * cos(cbdP.z);
    y = odomP.pose.position.y + 1 * sin(cbdP.z);
    cbdQ.setRPY(0, 0, cbdP.z);
    // tf2::convert(odomP.pose.orientation, odomQ);
    odomQ.setW(odomP.pose.orientation.w);
    odomQ.setX(odomP.pose.orientation.x);
    odomQ.setY(odomP.pose.orientation.y);
    odomQ.setZ(odomP.pose.orientation.z);
    q = cbdQ * odomQ;
    q.normalize();

    outputPoseStampedMsg.header = outputHeaderMsg;
    outputPoseStampedMsg.pose.position.x = x;
    outputPoseStampedMsg.pose.position.y = y;
    outputPoseStampedMsg.pose.position.z = odomP.pose.position.z;
    // tf2::convert(q, outputPoseStampedMsg.pose.orientation);
    outputPoseStampedMsg.pose.orientation.w = q.getW();
    outputPoseStampedMsg.pose.orientation.x = q.getX();
    outputPoseStampedMsg.pose.orientation.y = q.getY();
    outputPoseStampedMsg.pose.orientation.z = q.getZ();
    pubPoseStamped.publish(outputPoseStampedMsg);


    robotState = 1;
    break;
  case 4:
    if (explorationState)
      stop();
    break;

  default:
    break;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "control");
  Control control;
  control.startExploration();
  while (ros::ok) {
    control.findBall();
    ros::spinOnce();
  }

  return 0;
}