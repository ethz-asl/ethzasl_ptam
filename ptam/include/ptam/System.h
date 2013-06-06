// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H
//#include "VideoSource.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ptam_com/PointCloud.h>
#include <ptam_com/KeyFrame_srv.h>
#include <std_msgs/String.h>
#include <queue>

#include "GLWindow2.h"
//#include "ptam/RosNode.h"
#include "ptam/Params.h"

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>

class ATANCamera;
struct Map;
class MapMaker;
class Tracker;
class MapViewer;

class System
{
  typedef std::queue<sensor_msgs::Imu> ImuQueue;
  typedef std::queue<geometry_msgs::PoseWithCovarianceStamped> PoseQueue;

public:
  System();
  void Run();

private:
  ros::NodeHandle nh_, image_nh_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_calibration_;
  ros::Subscriber sub_kb_input_;
  tf::TransformBroadcaster tf_pub_;
  tf::TransformListener tf_sub_;
  image_transport::Subscriber sub_image_;
  image_transport::Publisher pub_preview_image_;
  ros::Publisher pub_pose_;             // world in the camera frame
  ros::Publisher pub_pose_world_;       // camera in the world frame
  ros::Publisher pub_info_;
  ros::ServiceServer srvPC_;
  ros::ServiceServer srvKF_;

  ros::CallbackQueue image_queue_;

  ImuQueue imu_msgs_;

  bool first_frame_;

  GLWindow2 *mGLWindow;
  MapViewer *mpMapViewer;

  CVD::Image<CVD::byte > img_bw_;
  CVD::Image<CVD::Rgb<CVD::byte> > img_rgb_;

  Map *mpMap; 
  MapMaker *mpMapMaker; 
  Tracker *mpTracker; 
  ATANCamera *mpCamera;

//  bool mbDone;

  void init(const CVD::ImageRef & size);

  void publishPoseAndInfo(const std_msgs::Header & header);
  void publishPreviewImage(CVD::Image<CVD::byte> & img, const std_msgs::Header & header);
  bool pointcloudservice(ptam_com::PointCloudRequest & req, ptam_com::PointCloudResponse & resp);
  bool keyframesservice(ptam_com::KeyFrame_srvRequest & req, ptam_com::KeyFrame_srvResponse & resp);

  void imageCallback(const sensor_msgs::ImageConstPtr & img);
  void imuCallback(const sensor_msgs::ImuConstPtr & msg);
  void keyboardCallback(const std_msgs::StringConstPtr & kb_input);

  bool transformQuaternion(const std::string & target_frame, const std_msgs::Header & header, const geometry_msgs::Quaternion & q_in, TooN::SO3<double> & r_out);
  bool transformPoint(const std::string & target_frame, const std_msgs::Header & header, const geometry_msgs::Point & t_in, TooN::Vector<3> & t_out);
  void quaternionToRotationMatrix(const geometry_msgs::Quaternion & q, TooN::SO3<double> & R);

  /// finds object in queue with timestamp closest to timestamp. Requires that T has a std_msgs::header field named "header"
  template<class T> bool findClosest(const ros::Time & timestamp, std::queue<T> & queue, T * obj, const double & max_delay = 0.01);

  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

};



#endif
