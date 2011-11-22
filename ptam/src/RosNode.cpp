/*
 * RosNode.cpp
 *
 *  Created on: Feb 17, 2010
 *      Author: acmarkus
 */

#include <ptam/RosNode.h>
#include <ptam/LevelHelpers.h>

#include <highgui.h>

#include <tf/transform_broadcaster.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <ptam/Params.h>

ParamsAccess test;

namespace RosNode
{

VideoSource::VideoSource()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get topic to subscribe to:
  std::string topic = nh.resolveName("image");
  if (topic == "/image")
  {
    ROS_WARN("video source: image has not been remapped! Typical command-line usage:\n"
        "\t$ ./ptam image:=<image topic>");
  }

  // transport hints:
  image_transport::TransportHints hints =
      image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay(true));
  // TODO: make this changeable by setting parameter(s) ?

  lastSize_ = CVD::ImageRef_zero;
  cvImage_ = NULL;
  firstImageCallback_ = false;
  image_transport::ImageTransport it(nh);
  sub_ = it.subscribe(topic, 1, &VideoSource::imageCallback, this, hints);

  ROS_INFO("ptam: waiting for first image");
  while (!firstImageCallback_)
  {
    usleep(100e3);
  }
  ROS_INFO("ptam: got first image");

}

void VideoSource::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  boost::lock_guard<boost::mutex> guard(imageMutex_);

  IplImage * frame = cvBridge_.imgMsgToCv(msg, sensor_msgs::image_encodings::MONO8);

  size_.x = frame->width;
  size_.y = frame->height;

  if (lastSize_ != size_)
  {
    if (cvImage_ != NULL)
      cvReleaseImage(&cvImage_);
    cvImage_ = cvCloneImage(frame);
  }
  else
  {
    cvCopy(frame, cvImage_);
  }

  lastSize_ = size_;

  firstImageCallback_ = true;
  newImage_ = true;

  frameId_ = msg->header.frame_id;
  timestamp_ = msg->header.stamp;
}

CVD::ImageRef VideoSource::Size()
{
  return size_;
}

bool VideoSource::GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB)
{
  boost::lock_guard<boost::mutex> guard(imageMutex_);

  if (newImage_)
  {
    CVD::BasicImage<CVD::byte> imCaptured((CVD::byte *)cvImage_->imageData, size_);
    CVD::copy(imCaptured, imBW);
    CVD::copy(imCaptured, imRGB);
    newImage_ = false;
    return true;
  }
  else
    return false;
}

void VideoSource::getStampAndFrameId(double &timestamp, std::string & frameId)
{
  boost::lock_guard<boost::mutex> guard(imageMutex_);

  frameId = frameId_;
  timestamp = timestamp_.toSec();
}

inline TooN::Matrix<3, 3, double> quaternionToRotationMatrix(const double * q)
{
  // stolen from Eigen3 and adapted to TooN

  TooN::Matrix<3, 3, double> res;

  const double tx = 2 * q[1];
  const double ty = 2 * q[2];
  const double tz = 2 * q[3];
  const double twx = tx * q[0];
  const double twy = ty * q[0];
  const double twz = tz * q[0];
  const double txx = tx * q[1];
  const double txy = ty * q[1];
  const double txz = tz * q[1];
  const double tyy = ty * q[2];
  const double tyz = tz * q[2];
  const double tzz = tz * q[3];

  res(0, 0) = 1 - (tyy + tzz);
  res(0, 1) = txy - twz;
  res(0, 2) = txz + twy;
  res(1, 0) = txy + twz;
  res(1, 1) = 1 - (txx + tzz);
  res(1, 2) = tyz - twx;
  res(2, 0) = txz - twy;
  res(2, 1) = tyz + twx;
  res(2, 2) = 1 - (txx + tyy);

  return res;
}

Publisher::Publisher()
{
  ros::NodeHandle nh("vslam");
  //	ros::NodeHandle pnh(nh, "");
  ParamsAccess Params;

  pubPose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
  pubInfo_ = nh.advertise<ptam_msgs::ptam_info> ("info", 1);
}

bool Publisher::publishPose(TooN::SE3<> pose, TooN::Matrix<6> covar, int trackingQuality, bool mapQuality,
                            double timestamp, const std::string & frameId)
{

  ParamsAccess Params;
  double scale = Params.varParams->Scale;
  static TooN::Matrix<6> covR = TooN::Identity;
  static geometry_msgs::TransformStamped T;

  if (scale <= 0)
  {
    ROS_WARN_STREAM("scale ("<<scale<<") <= 0, set to 1");
    scale = 1;
  }

  static unsigned int seq = 0;

  if (trackingQuality && mapQuality)
  {
    pose.get_translation() = pose.get_translation() / scale;

    TooN::Matrix<3, 3, double> r = pose.get_rotation().get_matrix();
    TooN::Vector<3, double> & t = pose.get_translation();

    transform_
        = tf::StampedTransform(tf::Transform(btMatrix3x3(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0),
                                                         r(2, 1), r(2, 2)), btVector3(t[0], t[1], t[2])),
                               ros::Time(timestamp), frameId, Params.fixParams->parent_frame);

    tf::transformStampedTFToMsg(transform_, T);
    msgPose_.pose.pose.orientation = T.transform.rotation;
    msgPose_.pose.pose.position.x = T.transform.translation.x;
    msgPose_.pose.pose.position.y = T.transform.translation.y;
    msgPose_.pose.pose.position.z = T.transform.translation.z;

    for (unsigned int i = 0; i < msgPose_.pose.covariance.size(); i++)
      msgPose_.pose.covariance[i] = sqrt(fabs(covar[i % 6][i / 6]));

    msgPose_.header.stamp = ros::Time(timestamp);
    msgPose_.header.seq = seq;
    msgPose_.header.frame_id = frameId;
    seq++;

    pubPose_.publish(msgPose_);
    transformBroadcaster_.sendTransform(transform_);
  }

  return true;
}

bool Publisher::publishPtamInfo(int trackingQuality, bool mapQuality, const std::string & mapViewerMessage,
                                const std::string & trackerMessage, int keyframes, double timestamp,
                                const std::string & frameId)
{
  static unsigned int seq = 0;
  static float fps = 0;
  static double lastTime = 0;
  double diff = timestamp - lastTime;
  if (diff < 1.0 && diff > 0.005)
    fps = fps * 0.8 + 0.2 / diff;
  if (fabs(fps) > 200)
    fps = 200;
  lastTime = timestamp;

  msgInfo_.header.stamp = ros::Time(timestamp);
  msgInfo_.header.frame_id = frameId;
  msgInfo_.header.seq = seq;
  seq++;
  msgInfo_.fps = fps;
  msgInfo_.mapQuality = mapQuality;
  msgInfo_.trackingQuality = trackingQuality;
  msgInfo_.trackerMessage = trackerMessage;
  msgInfo_.mapViewerMessage = mapViewerMessage;
  msgInfo_.keyframes = keyframes;
  pubInfo_.publish(msgInfo_);
  return true;
}


//{slynen
//publishes to keysPressed topic to allow bag based
//replay of keyboard inputs in sync with the video
bool RemoteInterface::publishKbInput(const std::string & _keysPressed)
{
  std_msgs::String msg;
  msg.data = _keysPressed;
  pubKbInput_.publish(msg);
  return true;
}

void RemoteInterface::receiveKbInput(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("command received: [%s]", msg->data.c_str());
  command_ = msg->data.c_str();
  newCommand_ = true;
}
//}

RemoteInterface::RemoteInterface() :
  it_(ros::NodeHandle())
{
  command_ = "";
  newCommand_ = false;
  ros::NodeHandle nh("vslam");
  commandServer_ = nh.advertiseService("command", &RemoteInterface::commandCallback, this);
  it_ = image_transport::ImageTransport(nh);
  previewPublisher_ = it_.advertise("preview", 1);
  //{slynen
  pubKbInput_ = nh.advertise<std_msgs::String> ("keysPressed", 5);
  subKbInput_ = nh.subscribe("keysPressed", 1000, &RemoteInterface::receiveKbInput, this);
  //}
}

bool RemoteInterface::commandCallback(ptam_srvs::ptam_commandRequest & req, ptam_srvs::ptam_commandResponse & res)
{
  command_ = req.command;
  newCommand_ = true;
  //{slynen
  //publish to rostopic too
  publishKbInput(req.command);
  //}

  return true;
}

bool RemoteInterface::newCommand()
{
  return newCommand_;
}

std::string RemoteInterface::getCommand()
{
  newCommand_ = false;
  return command_;
}

void RemoteInterface::publishPreview(CVD::Image<CVD::byte> &imBW, CVD::Image<TooN::Vector<2> > & grid,
                                     std::list<Trail> & trails, bool drawGrid, bool drawTrails)
{
  if (previewPublisher_.getNumSubscribers() > 0)
  {
    sensor_msgs::CvBridge bridge;
    IplImage * tmpImg = cvCreateImageHeader(cvSize(imBW.size().x, imBW.size().y), IPL_DEPTH_8U, 1);
    tmpImg->imageData = (char*)imBW.data();

    //		bridge.fromIpltoRosImage(tmpImg, prevImage_);
    prevImage_ = bridge.cvToImgMsg(tmpImg);
    IplImage * tmpImg2 = bridge.imgMsgToCv(prevImage_);

    int dim0 = grid.size().x;
    int dim1 = grid.size().y;

    if (drawGrid)
    {
      for (int i = 0; i < dim0; i++)
      {
        for (int j = 0; j < dim1 - 1; j++)
          cvLine( tmpImg2, cvPoint(grid[i][j][0], grid[i][j][1]), cvPoint(grid[i][j + 1][0], grid[i][j + 1][1]),
                 CV_RGB(50, 50, 50)
          );

        for (int j = 0; j < dim1 - 1; j++)
          cvLine(tmpImg2, cvPoint(grid[j][i][0], grid[j][i][1]), cvPoint(grid[j + 1][i][0], grid[j + 1][i][1]),
                 CV_RGB(50, 50, 50)
          );
      }
    }

    if (drawTrails)
    {

      ParamsAccess Params;
      int level = Params.fixParams->InitLevel;

      for (std::list<Trail>::iterator i = trails.begin(); i != trails.end(); i++)
      {
        cvLine(tmpImg2, cvPoint(LevelZeroPos(i->irCurrentPos.x, level), LevelZeroPos(i->irCurrentPos.y, level)),
               cvPoint(LevelZeroPos(i->irInitialPos.x, level), LevelZeroPos(i->irInitialPos.y, level)),
               CV_RGB(0, 0, 0), 2);
      }
    }

    previewPublisher_.publish(prevImage_);

    cvReleaseImageHeader(&tmpImg);
  }
}

Visualization::Visualization() :
  camera_("Camera")
{
  ros::NodeHandle nh("vslam");
  trailPublisher_ = nh.advertise<visualization_msgs::Marker> ("trails", 1);
  gridPublisher_ = nh.advertise<visualization_msgs::Marker> ("grid", 1);

  msgTrails_.header.stamp = ros::Time::now();
  msgTrails_.header.frame_id = "/world";
  msgTrails_.type = visualization_msgs::Marker::LINE_LIST;
  msgTrails_.scale.x = 0.01;
  msgTrails_.scale.y = 0.01;
  msgTrails_.scale.z = 0.01;
  msgTrails_.pose.position.x = 0;
  msgTrails_.pose.position.y = 0;
  msgTrails_.pose.position.z = 0;
  msgTrails_.pose.orientation.w = 1;
  msgTrails_.pose.orientation.x = 0;
  msgTrails_.pose.orientation.y = 0;
  msgTrails_.pose.orientation.z = 0;
  msgTrails_.color.r = 1.0;
  msgTrails_.color.g = 0;
  msgTrails_.color.b = 0;
  msgTrails_.color.a = 1.0;
  msgTrails_.lifetime = ros::Duration(1.0);
  msgTrails_.action = visualization_msgs::Marker::ADD;
  msgTrails_.ns = "ptam";
  msgTrails_.id = 0;
}

void Visualization::publishTrails(std::list<Trail> & trails)
{
  msgTrails_.points.resize(trails.size() * 2);

  TooN::Vector<2> tmp;

  int j = 0;

  for (std::list<Trail>::iterator i = trails.begin(); i != trails.end(); i++)
  {
    geometry_msgs::Point& point1 = msgTrails_.points.at(j * 2);
    geometry_msgs::Point& point2 = msgTrails_.points.at(j * 2 + 1);

    tmp = camera_.UnProject(i->irInitialPos);
    point1.x = tmp[0];
    point1.y = tmp[1];
    point1.z = 0;

    tmp = camera_.UnProject(i->irCurrentPos);
    point2.x = tmp[0];
    point2.y = tmp[1];
    point2.z = 0;
    j++;
  }

  trailPublisher_.publish(msgTrails_);
}

}
