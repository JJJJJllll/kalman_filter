#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>

#include <time.h>
#include <math.h>
#include <Eigen/Eigen>
#include "kalman_lib/kalman.hpp"

using namespace std;
using namespace Eigen;

// vrpn server time
ros::Time last_vrpn_server_time, now_vrpn_server_time;
double dynamic_DT = 0;
bool first_vrpn = true;

// Kalman filter Variables Definition Start
double freq = 200;
double dt = 1/freq;

int n = 6; // Number of states
Eigen::MatrixXd A(n, n); // System dynamics matrix

ros::Publisher  pub_pose, pub_vel;
geometry_msgs::Vector3Stamped kf_pos, kf_vel;

std::shared_ptr<KalmanFilter> kf;

double x_now = 0, y_now = 0, z_now = 0;
double x_pre = 0, y_pre = 0, z_pre = 0;
double dx = 0, dy = 0, dz = 0; 
double vel_naive_x = 0, vel_naive_y = 0, vel_naive_z = 0;

void vrpn_pose_callback(const geometry_msgs::PoseStamped &message)
{
  if (first_vrpn) {
    first_vrpn = false;
    x_pre = message.pose.position.x;
    y_pre = message.pose.position.y;
    z_pre = message.pose.position.z;
    last_vrpn_server_time = message.header.stamp;
    return;
  }

  x_now = message.pose.position.x;
  y_now = message.pose.position.y;
  z_now = message.pose.position.z;
  now_vrpn_server_time = message.header.stamp;

  dynamic_DT = now_vrpn_server_time.toSec() - last_vrpn_server_time.toSec();

  vel_naive_x = (x_now - x_pre) / dynamic_DT;
  vel_naive_y = (y_now - y_pre) / dynamic_DT;
  vel_naive_z = (z_now - z_pre) / dynamic_DT;
  
  Eigen::VectorXd y(3);
  y << x_now,y_now,z_now;  
  A.topRightCorner(n/2,n/2) = dynamic_DT * MatrixXd::Identity(n/2,n/2);    
  kf->update(y, dynamic_DT, A);

  VectorXd filtered_pose = kf->state();
  
  kf_pos.header.stamp = now_vrpn_server_time;//ros::Time::now();
  kf_pos.header.frame_id = "naive_vel";
  kf_pos.vector.x = vel_naive_x;//filtered_pose(0);
  kf_pos.vector.y = vel_naive_y;//filtered_pose(1);
  kf_pos.vector.z = vel_naive_z;//filtered_pose(2);
  pub_pose.publish(kf_pos);
  
  kf_vel.header = kf_pos.header;
  kf_vel.header.frame_id = "kf_vel";
  kf_vel.vector.x = filtered_pose(3);
  kf_vel.vector.y = filtered_pose(4);
  kf_vel.vector.z = filtered_pose(5);
  pub_vel.publish(kf_vel);
  
  x_pre = x_now;
  y_pre = y_now;
  z_pre = z_now;
  last_vrpn_server_time = message.header.stamp;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_filter_pose3D");
  ros::NodeHandle nh;

  string QUAD_NAME;
  nh.param<std::string>("QUAD_NAME", QUAD_NAME, "jackBi");
  char DEVICE_NAME[100];
  strcpy(DEVICE_NAME, "/vrpn_client_node/");
  strcat(DEVICE_NAME, QUAD_NAME.c_str());
  strcat(DEVICE_NAME, "/pose");
  cout<<DEVICE_NAME<<endl;
	
  ros::Rate loop_rate(freq);
  
  ros::Subscriber sub_pose   = nh.subscribe(DEVICE_NAME, 1, vrpn_pose_callback);
  pub_pose   = nh.advertise<geometry_msgs::Vector3Stamped>("naive_vel", 5);
  pub_vel    = nh.advertise<geometry_msgs::Vector3Stamped>("kf_vel", 5);

  int m = 3; // Number of measurements

  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd R(n, n); // Process noise covariance
  Eigen::MatrixXd Q(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance
  Eigen::VectorXd x0(n);

  A.topLeftCorner(n/2,n/2) = MatrixXd::Identity(n/2,n/2);
  A.topRightCorner(n/2,n/2) = dt*MatrixXd::Identity(n/2,n/2);
  A.bottomLeftCorner(n/2,n/2) = MatrixXd::Zero(n/2,n/2);
  A.bottomRightCorner(n/2,n/2) = MatrixXd::Identity(n/2,n/2);

  C.topLeftCorner(m,m) = MatrixXd::Identity(m,m);
  C.topRightCorner(n-m,n-m) = MatrixXd::Zero(n-m,n-m);

  R = 0.001*MatrixXd::Identity(n,n); // Trust position updates because x=x+t*v is simple
  R.bottomRightCorner(n/2,n/2) = 10 * MatrixXd::Identity(n/2,n/2); // Very doubt velocity update because it's totally wrong
  Q = 0.001*MatrixXd::Identity(m,m); // Trust position observation because Optitrack result is accurate, it has to be position because measurement is position

  P = 10 * MatrixXd::Identity(n,n);

  kf = std::make_shared<KalmanFilter>(dt, A, C, R, Q, P);

  x0 << 0,0,0,0,0,0;
  kf->init(0, x0);

  while (ros::ok())
  { 
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
