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
  geometry_msgs::PoseStamped oldPoseMsg;
  bool cbdState;
  bool psdState;
  bool explorationState;
  int robotState;
  ros::ServiceClient stopClient;
  ros::ServiceClient startClient;
  std_srvs::Trigger trigger;
  ros::Time time_old;

  void cbdCB(const geometry_msgs::PointStamped& msg);
  void psdCB(const geometry_msgs::PointStamped& msg);
  void odomCB(const nav_msgs::Odometry& msg);

 public:
  Control();
  ~Control();
  void startExploration();
  void stop(void);
  bool findBall(void);
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
  ROS_INFO_STREAM("Iniciou");
  explorationState = true;
}

void Control::stop(void) {
  stopClient.call(trigger);
  explorationState = false;
}


bool Control::findBall(void) {
  double x , y;
  std_msgs::Header outputHeaderMsg;
  geometry_msgs::PoseStamped outputPoseStampedMsg;
  tf::Quaternion odomQ;
  tf::Quaternion cbdQ;
  tf::Quaternion q;
  tf::Matrix3x3* m;
  double roll, pitch, yaw, r;
  double alpha = 0;
  double deltaPose;


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
    if (cbdState) {
      ROS_INFO_STREAM("Robot found ball using CBD!");
      robotState = 2;
    }
    // if (psdState)
    //   robotS
    break;
  case 2:
    if (explorationState) {
      stop();
      ROS_INFO_STREAM("Send stop signal!");
      robotState = 3;
      oldPoseMsg.pose = odomP.pose;
    }
    break;
  case 3:
    cbdState = false;

    // deltaPose = sqrt(
    //   pow(oldPoseMsg.pose.position.x - odomP.pose.position.x, 2) +
    //   pow(oldPoseMsg.pose.position.y - odomP.pose.position.y, 2));

    // if (deltaPose < 0.5) {

    ROS_INFO_STREAM("Robot CBD moving robot!");
    outputHeaderMsg.seq = odomH.seq;
    outputHeaderMsg.stamp = ros::Time::now();
    outputHeaderMsg.frame_id = odomH.frame_id;

    if (cbdP.z > 0)
      cbdP.z = std::min(cbdP.z, M_PI * 10/180);
    else
      cbdP.z = std::max(cbdP.z, - M_PI * 10/180);

    // if (abs(cbdP.z) > M_PI * 15 / 180) {
    //   robotState = 1;
    //   startExploration();
    //   break;
    // }

    cbdQ.setRPY(0, 0, cbdP.z);
    odomQ.setW(odomP.pose.orientation.w);
    odomQ.setX(odomP.pose.orientation.x);
    odomQ.setY(odomP.pose.orientation.y);
    odomQ.setZ(odomP.pose.orientation.z);
    q = cbdQ * odomQ;
    q.normalize();

    m = new tf::Matrix3x3(q);
    m->getRPY(roll, pitch, yaw);

    // alpha = (0.25 - cbdP.x) / 0.25;

    // x = odomP.pose.position.x + alpha * 1 * cos(yaw);
    // y = odomP.pose.position.y + alpha * 1 * sin(yaw);

    x = odomP.pose.position.x + 1 * cos(yaw);
    y = odomP.pose.position.y + 1 * sin(yaw);

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

    oldPoseMsg = outputPoseStampedMsg;

    ROS_INFO_STREAM("Goal msg: " << outputPoseStampedMsg);
  // }
    // if (cbdP.x > 0.3)
    //   robotState = 5;
    // else
    //   robotState = 3;

    if (psdState)
      robotState = 4;
    else
      robotState = 3;
    break;
  case 4:
    ROS_INFO_STREAM("Robot PSD moving robot!");
    outputHeaderMsg.seq = odomH.seq;
    outputHeaderMsg.stamp = ros::Time::now();
    outputHeaderMsg.frame_id = odomH.frame_id;

    cbdQ.setRPY(0, 0, cbdP.z);
    odomQ.setW(odomP.pose.orientation.w);
    odomQ.setX(odomP.pose.orientation.x);
    odomQ.setY(odomP.pose.orientation.y);
    odomQ.setZ(odomP.pose.orientation.z);
    q = cbdQ * odomQ;
    q.normalize();

    m = new tf::Matrix3x3(q);
    m->getRPY(roll, pitch, yaw);

    r = sqrt(psdP.x * psdP.x + psdP.y * psdP.y);

    if (r > 7) {
      robotState = 3;
      break;
    }

    if (r > 2.5)
      r -= 1;

    x = odomP.pose.position.x + r * cos(yaw);
    y = odomP.pose.position.y + r * sin(yaw);

    outputPoseStampedMsg.header = outputHeaderMsg;
    outputPoseStampedMsg.pose.position.x = x;
    outputPoseStampedMsg.pose.position.y = y;
    outputPoseStampedMsg.pose.position.z = odomP.pose.position.z;
    outputPoseStampedMsg.pose.orientation = odomP.pose.orientation;
    pubPoseStamped.publish(outputPoseStampedMsg);

    ROS_INFO_STREAM("Goal msg: " << outputPoseStampedMsg);

    if (r > 2.5)
      robotState = 4;
    else
      robotState = 5;
    break;
  case 5:
    return 1;
    // break;

  default:
    break;
  }
  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "control");
  Control control;
  while (ros::ok) {
    if (control.findBall())
      break;
    ros::spinOnce();
  }
  ros::shutdown();
  return 0;
}
