/*
 * COPYRIGHT Pedro Paulo Ventura Tecchio 2020
*/
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <actionlib_msgs/GoalStatusArray.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>


#include <nav2d_navigator/RobotNavigator.h>
#include <nav2d_navigator/commands.h>

class Control {
  ros::NodeHandle nh_;
  ros::Subscriber subCBD;
  ros::Subscriber subPSD;
  ros::Subscriber subOdom;
  ros::Subscriber subNav2Exploration;
  ros::Publisher pubPoseStamped;
  std_msgs::Header cbdH;
  std_msgs::Header psdH;
  std_msgs::Header odomH;
  actionlib_msgs::GoalStatusArray goalMsg;
  geometry_msgs::Point cbdP;
  geometry_msgs::Point psdP;
  std::vector<geometry_msgs::Point> psdPv;
  geometry_msgs::PoseWithCovariance odomP;
  // geometry_msgs::Pose2D desiredPose;
  tf::Stamped<tf::Pose> desiredPose;
  bool cbdState;
  bool psdState;
  bool explorationState;
  int robotState;
  ros::ServiceClient stopClient;
  ros::ServiceClient startClient;
  std_srvs::Trigger trigger;
  ros::Time oldTime;

  void cbdCB(const geometry_msgs::PointStamped& msg);
  void psdCB(const geometry_msgs::PointStamped& msg);
  void odomCB(const nav_msgs::Odometry& msg);
  void goalCB(const actionlib_msgs::GoalStatusArray& msg);

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

  subNav2Exploration = nh_.subscribe("Explore/status",
                         1,
                         &Control::odomCB,
                         this);
  pubPoseStamped = nh_.advertise<geometry_msgs::PoseStamped>
    ("move_base_simple/goal", 1);

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

  psdPv.push_back(msg.point);
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

void Control::goalCB(const actionlib_msgs::GoalStatusArray& msg) {
  goalMsg = msg;

  // ROS_INFO_STREAM("Odom Header: " << odomH);
  // ROS_INFO_STREAM("Odom Pose: "   << odomP);
}

void Control::startExploration(void) {
  startClient.call(trigger);
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
  double error;


  ROS_INFO_STREAM("RobotState: " << robotState);
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
    if (!isnan(odomP.pose.position.x)) {
      // if (goalMsg.status_list.
      if (cbdState) {
        ROS_INFO_STREAM("Robot found ball using CBD!");
        robotState = 2;
        break;
      } else {
        if (!explorationState)
          startExploration();
        robotState = 1;
      }
    }
    break;
  case 2:
    if (explorationState) {
      stop();
      ROS_INFO_STREAM("Robot sent stop signal (CBD)!");
    }
    robotState = 3;
    break;
  case 3:

    if (cbdP.z < 7) {  // check if it is near enough the ball to use psd
      robotState = 5;
      break;
    }

    ROS_INFO_STREAM("Robot moving (CBD)!");
    outputHeaderMsg.seq = cbdH.seq;
    outputHeaderMsg.stamp = ros::Time::now();
    outputHeaderMsg.frame_id = cbdH.frame_id;

    outputPoseStampedMsg.header = outputHeaderMsg;
    outputPoseStampedMsg.pose.position.x = cbdP.x;
    outputPoseStampedMsg.pose.position.y = -cbdP.y;
    outputPoseStampedMsg.pose.position.z = 0;
    // tf2::convert(q, outputPoseStampedMsg.pose.orientation);
    outputPoseStampedMsg.pose.orientation.w = 1;
    outputPoseStampedMsg.pose.orientation.x = 0;
    outputPoseStampedMsg.pose.orientation.y = 0;
    outputPoseStampedMsg.pose.orientation.z = 0;
    pubPoseStamped.publish(outputPoseStampedMsg);

    // tf::poseStampedMsgToTF(outputPoseStampedMsg, desiredPose);

    robotState = 3;
    oldTime = ros::Time::now();
    // robotState = 4;
    cbdState = false;
    break;

  case 4:

    if ((ros::Time::now().toSec() - oldTime.toSec()) > 100) {
      robotState = 3;
    }
    // error  = std::pow(desiredPose.getOrigin().getX() -
    //   odomP.pose.position.x, 2);
    // error += std::pow(desiredPose.getOrigin().getY() -
    //   odomP.pose.position.y, 2);
    // error = sqrt(error);

    // if (error < 0.5) {
    //   robotState = 1;
    //   // cbdState = false;
    // } else {
    //   robotState = 4;
    // }
    break;

  case 5:
    psdState = false;
    psdPv.clear();
    oldTime = ros::Time::now();
    robotState = 6;
    break;

  case 6:
    if ((ros::Time::now().toSec() - oldTime.toSec()) < 10) {
      break;
    } else {
      if (psdPv.size() > 1) {
        x = 0;
        y = 0;
        for (int i = 0; i < psdPv.size(); i++) {
          // x = 0.7*x + 0.3*psdPv[i].x;
          // y = 0.7*y + 0.3*psdPv[i].y;
          x += psdPv[i].x;
          y += psdPv[i].y;
        }
        x /= psdPv.size();
        y /= psdPv.size();
        robotState = 8;
      } else {
        robotState = 0;
      }
    }

    break;

  case 7:
    if (explorationState) {
      stop();
      ROS_INFO_STREAM("Robot sent stop signal (PSD)!");
    }
      robotState = 8;
    break;

  case 8:
    ROS_INFO_STREAM("Robot moving (PSD)!");
    outputHeaderMsg.seq = psdH.seq;
    outputHeaderMsg.stamp = ros::Time::now();
    outputHeaderMsg.frame_id = psdH.frame_id;

    r = sqrt(std::pow(x, 2) + std::pow(y, 2));
    yaw = atan2(y, x);

    r -= 1.5;
    if (r < 1.5)
      r = 0;

    outputPoseStampedMsg.header = outputHeaderMsg;
    outputPoseStampedMsg.pose.position.x = r * cos(yaw);
    outputPoseStampedMsg.pose.position.y = r * sin(yaw);
    outputPoseStampedMsg.pose.position.z = 0;
    // tf2::convert(q, outputPoseStampedMsg.pose.orientation);
    outputPoseStampedMsg.pose.orientation.w = 1;
    outputPoseStampedMsg.pose.orientation.x = 0;
    outputPoseStampedMsg.pose.orientation.y = 0;
    outputPoseStampedMsg.pose.orientation.z = 0;

    robotState = 9;

    psdState = false;
    break;
  case 9:
    ROS_INFO_STREAM("Robot is done!");
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
