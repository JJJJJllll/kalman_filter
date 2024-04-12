#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>

#include <time.h>
#include <math.h>
#include <Eigen/Eigen>
#include "kalman_lib/kalman.hpp"


using namespace std;
using namespace Eigen;

double freq = 100;
double dt = 1/freq;
string QUAD_NAME;
// vrpn local time
double vrpn_dt_local = 0;
struct timeval vrpn_localtime_last, vrpn_localtime_now;
// vrpn server time
ros::Time last_vrpn_server_time;
double dynamic_DT = 0;
// pose_world form VRPN
bool first_vrpn = true, VRPN_UPDATE = false;
geometry_msgs::PoseStamped pose_world;
/*
KF VARIABLE START
*/
geometry_msgs::Vector3Stamped  kf_pos, kf_vel;
/*
KF VARIABLE END
*/


void vrpn_pose_callback(const geometry_msgs::PoseStamped &message)
{
  VRPN_UPDATE = true;
  // vrpn local dt
  if (first_vrpn) {
    first_vrpn = false;
    gettimeofday(&vrpn_localtime_last, NULL);
  } else {
    gettimeofday(&vrpn_localtime_now, NULL);
    vrpn_dt_local = vrpn_localtime_now.tv_sec - vrpn_localtime_last.tv_sec + 1e-6 * (vrpn_localtime_now.tv_usec - vrpn_localtime_last.tv_usec);
    vrpn_localtime_last = vrpn_localtime_now;
  }
  // vrpn server dt
  dynamic_DT = message.header.stamp.toSec() - last_vrpn_server_time.toSec();
  last_vrpn_server_time = message.header.stamp;

  pose_world = message;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_filter_pose3D");
  ros::NodeHandle nh;

  // set subscriber name
  nh.param<std::string>("QUAD_NAME", QUAD_NAME, "jackQuad");
  char DEVICE_NAME[100];
  strcpy(DEVICE_NAME, "/vrpn_client_node/");
  strcat(DEVICE_NAME, QUAD_NAME.c_str());
  strcat(DEVICE_NAME, "/pose");
  cout<<DEVICE_NAME<<endl;
	
  ros::Rate loop_rate(freq);
  
  ros::Subscriber sub_pose   = nh.subscribe(DEVICE_NAME, 1, vrpn_pose_callback);
  /*
  KF PUBLISHER START
  */
  ros::Publisher  pub_pose   = nh.advertise<geometry_msgs::Vector3Stamped>("kf_pos", 5);
  ros::Publisher  pub_vel    = nh.advertise<geometry_msgs::Vector3Stamped>("kf_vel", 5);
  /*
  KF PUBLISHER END
  */

  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  int count = 0;
  srand((int)time(0));
  bool Start_timer = true;
  /*KF INITIALIZATION START*/
  int n = 6; // Number of states
  int m = 3; // Number of measurements
  // int r = 3; // Number of inputs
  
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  // Eigen::MatrixXd B(n, r); // System inputs matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd R(n, n); // Process noise covariance
  Eigen::MatrixXd Q(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  A.topLeftCorner(n/2,n/2) = MatrixXd::Identity(n/2,n/2);
  A.topRightCorner(n/2,n/2) = dt*MatrixXd::Identity(n/2,n/2);
  A.bottomLeftCorner(n/2,n/2) = MatrixXd::Zero(n/2,n/2);
  A.bottomRightCorner(n/2,n/2) = MatrixXd::Identity(n/2,n/2);

  // B << 0,0,0,
  //   0,0,0,
  //   0,0,0,
  //   dt,0,0,
  //   0,dt,0,
  //   0,0,dt;
  
  C.topLeftCorner(m,m) = MatrixXd::Identity(m,m);
  C.topRightCorner(n-m,n-m) = MatrixXd::Zero(n-m,n-m);

  R = 0.002*MatrixXd::Identity(n,n);
  R.bottomRightCorner(n/2,n/2) = 0.1*MatrixXd::Identity(n/2,n/2);
  Q = 0.0005*MatrixXd::Identity(m,m);

  P = MatrixXd::Constant(n,n, 0.1);

  KalmanFilter kf(dt, A, C, R, Q, P);

  Eigen::VectorXd x0(n);
  x0 << 0,0,0,0,0,0;
  kf.init(0, x0);
  /*
  KF INITIALIZATION END
  */

  while (ros::ok())
  {
    if(VRPN_UPDATE){
      VRPN_UPDATE = false;
      
      /*
      KF UPDATE START
      */
      Eigen::VectorXd y(3);
      y << pose_world.pose.position.x,pose_world.pose.position.y,pose_world.pose.position.z; 
      if(dynamic_DT>0.001 && dynamic_DT<1){
        cout<<dynamic_DT<<endl;
        A.topLeftCorner(n/2,n/2) = MatrixXd::Identity(n/2,n/2);
        A.topRightCorner(n/2,n/2) = dynamic_DT*MatrixXd::Identity(n/2,n/2);
        A.bottomLeftCorner(n/2,n/2) = MatrixXd::Zero(n/2,n/2);
        A.bottomRightCorner(n/2,n/2) = MatrixXd::Identity(n/2,n/2);     
        kf.update(y, dynamic_DT, A);
      } else {
        kf.update(y);
      }
      
      VectorXd filtered_pose = kf.state();
      //cout<<kf.state().transpose()<<endl;
      // Publish position
      kf_pos.header.stamp = pose_world.header.stamp;//ros::Time::now();
      kf_pos.header.seq = count;
      kf_pos.header.frame_id = "kf_pos";
      kf_pos.vector.x = filtered_pose(0);
      kf_pos.vector.y = filtered_pose(1);
      kf_pos.vector.z = filtered_pose(2);
      pub_pose.publish(kf_pos);
      // Publish velocity
      kf_vel.header = kf_pos.header;
      kf_vel.header.frame_id = "kf_vel";
      kf_vel.vector.x = filtered_pose(3);
      kf_vel.vector.y = filtered_pose(4);
      kf_vel.vector.z = filtered_pose(5);
      pub_vel.publish(kf_vel);
      /*
      KF UPDATE END
      */
    }
    
    
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
