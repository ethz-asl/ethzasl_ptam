// Copyright 2008 Isis Innovation Limited
#include "ptam/System.h"
#include "ptam/OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ptam/ATANCamera.h"
#include "ptam/MapMaker.h"
#include "ptam/Tracker.h"
#include "ptam/ARDriver.h"
#include "ptam/MapViewer.h"
#include "ptam/LevelHelpers.h"
#include "ptam/MapPoint.h"

#include <ptam_com/ptam_info.h>
#include <cvd/vision.h>

using namespace CVD;
using namespace std;
using namespace GVars3;


System::System() :
  nh_("vslam"), image_nh_(""), first_frame_(true)
{

  pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
  pub_info_ = nh_.advertise<ptam_com::ptam_info> ("info", 1);
  srvPC_ = nh_.advertiseService("pointcloud", &System::pointcloudservice,this);
  srvKF_ = nh_.advertiseService("keyframes", &System::keyframesservice,this);

  sub_imu_ = nh_.subscribe("imu", 100, &System::imuCallback, this);
  sub_predicted_pose_ = nh_.subscribe("predicted_pose", 100, &System::pposeCallback, this);
  sub_kb_input_ = nh_.subscribe("key_pressed", 100, &System::keyboardCallback, this);

  image_nh_.setCallbackQueue(&image_queue_);
  // get topic to subscribe to:
  std::string topic = image_nh_.resolveName("image");
  if (topic == "/image")
  {
    ROS_WARN("video source: image has not been remapped! Typical command-line usage:\n"
        "\t$ ./ptam image:=<image topic>");
  }

  image_transport::ImageTransport it(image_nh_);
  sub_image_ = it.subscribe(topic, 1, &System::imageCallback, this, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay(true)));
  pub_preview_image_ = it.advertise("vslam/preview", 1);
}


void System::init(const CVD::ImageRef & size)
{
  img_bw_.resize(size);
  img_rgb_.resize(size);

  mpCamera = new ATANCamera("Camera");

  mpMap = new Map;
  mpMapMaker = new MapMaker(*mpMap, *mpCamera);
  mpTracker = new Tracker(size, *mpCamera, *mpMap, *mpMapMaker);

  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);

#ifndef NO_GUI
  mGLWindow = new GLWindow2(size, "PTAM");
  mpARDriver = new ARDriver(*mpCamera, size, *mGLWindow);
  mpARDriver->Init();
  mpMapViewer = new MapViewer(*mpMap, *mGLWindow);

  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GUI.ParseLine("DrawAR=0");
  GUI.ParseLine("DrawMap=0");
  GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");
#endif
}


void System::Run()
{
  while(ros::ok()){
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
//    image_queue_.callAvailable();

    ros::getGlobalCallbackQueue()->callAvailable();
    image_queue_.callAvailable(ros::WallDuration(0.01));
  }
}

void System::imageCallback(const sensor_msgs::ImageConstPtr & img)
{
//	static ros::Time t = img->header.stamp;


  ROS_ASSERT(img->encoding == sensor_msgs::image_encodings::MONO8 && img->step == img->width);

  VarParams *varParams = ParamsAccess::varParams;

  if(first_frame_){
    init(CVD::ImageRef(img->width, img->height));
    first_frame_ = false;
  }

  TooN::SO3<double> imu_orientation;
  if (varParams->MotionModelSource == ptam::PtamParams_MM_IMU)
  {
    sensor_msgs::Imu imu;


    if (!findClosest(img->header.stamp, imu_msgs_, &imu, 0.01))
    {
      ROS_WARN("no imu match, skipping frame");
      return;
    }
    if (!transformQuaternion(img->header.frame_id, imu.header, imu.orientation, imu_orientation))
    {
      return;
    }
  }


//  -------------------
  // TODO: avoid copy
  CVD::BasicImage<CVD::byte> img_tmp((CVD::byte *)&img->data[0], CVD::ImageRef(img->width, img->height));
  CVD::copy(img_tmp, img_bw_);

  bool tracker_draw = false;

#ifndef NO_GUI
  CVD::copy(img_tmp, img_rgb_);

  mGLWindow->SetupViewport();
  mGLWindow->SetupVideoOrtho();
  mGLWindow->SetupVideoRasterPosAndZoom();

  if (!mpMap->IsGood())
    mpARDriver->Reset();

  static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN | SILENT);
  static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN | SILENT);

  bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
  bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;
  tracker_draw = !bDrawAR && !bDrawMap;
#endif

  mpTracker->TrackFrame(img_bw_, tracker_draw, imu_orientation);


  publishPoseAndInfo(img->header);

  publishPreviewImage(img_bw_, img->header);
  std::cout << mpMapMaker->getMessageForUser();

#ifndef NO_GUI
  string sCaption;

  if (bDrawMap)
    mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
  else if (bDrawAR)
    mpARDriver->Render(img_rgb_, mpTracker->GetCurrentPose());

  if (bDrawMap)
    sCaption = mpMapViewer->GetMessageForUser();
  else
    sCaption = mpTracker->GetMessageForUser();
  mGLWindow->DrawCaption(sCaption);
  mGLWindow->DrawMenus();
  mGLWindow->swap_buffers();
  mGLWindow->HandlePendingEvents();
#endif

//	usleep(50000);
//
//  ros::Time t1 = img->header.stamp;
//
//  static unsigned long c=0;
//  if((c+1)<(img->header.seq))
//  {
//	  ROS_WARN_STREAM("missed " << img->header.seq-c+1<< " frame(s)");
//	  ROS_WARN_STREAM("time: " <<  (t1.toSec()-t.toSec()));
//  }
//  ROS_WARN_STREAM("time: " <<  1/(t1.toSec()-t.toSec()));
//  c=img->header.seq;
//  t = img->header.stamp;

}


void System::imuCallback(const sensor_msgs::ImuConstPtr & msg)
{
  imu_msgs_.push(*msg);
  if (imu_msgs_.size() > 100)
    imu_msgs_.pop();
}

void System::pposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{
  predicted_poses_.push(*msg);
  if (predicted_poses_.size() > 100)
    predicted_poses_.pop();
}

template<class T>
  bool System::findClosest(const ros::Time & timestamp, std::queue<T> & queue, T * obj, const double & max_delay)
  {
    double best_dt(1e9);
    double tmp_dt;
    //  size_t qs_before = queue.size();
    //  int i = 0;
    while (!queue.empty())
    {
      const T & curr_obj = queue.front();
      tmp_dt = (timestamp - curr_obj.header.stamp).toSec();

      if (tmp_dt < -max_delay)
        break;
      if (std::abs(tmp_dt) < best_dt)
      {
        best_dt = std::abs(tmp_dt);
        *obj = curr_obj;
        //      i++;
      }
      queue.pop();
    }
    if (best_dt > max_delay)
    {
      //    ROS_WARN("dt(%f) > 0.01 qs:%d, %d/%d", best_dt, queue.size(), qs_before, i);
      return false;
    }
    else
    {
      return true;
    };
  }


void System::keyboardCallback(const std_msgs::StringConstPtr & kb_input){
  mpTracker->command(kb_input->data);
}

bool System::transformQuaternion(const std::string & target_frame, const std_msgs::Header & header,
                                 const geometry_msgs::Quaternion & _q_in, TooN::SO3<double> & r_out)
{
  geometry_msgs::QuaternionStamped q_in, q_out;
  q_in.header = header;
  q_in.quaternion = _q_in;
  try
  {
    tf_sub_.transformQuaternion(target_frame, q_in, q_out);
    quaternionToRotationMatrix(q_out.quaternion, r_out);
    return true;
  }
  catch (tf::TransformException & e)
  {
    ROS_WARN_STREAM("could not transform quaternion: "<<e.what());
    return false;
  }
  return true;
}

bool System::transformPoint(const std::string & target_frame, const std_msgs::Header & header,
                            const geometry_msgs::Point & _p_in, TooN::Vector<3> & _p_out)
{
  geometry_msgs::PointStamped p_in, p_out;
  p_in.header = header;
  p_in.point = _p_in;
  try
  {
    tf_sub_.transformPoint(target_frame, p_in, p_out);
    _p_out[0] = p_out.point.x;
    _p_out[1] = p_out.point.y;
    _p_out[2] = p_out.point.z;
    return true;
  }
  catch (tf::TransformException & e)
  {
    ROS_WARN_STREAM("could not transform point: "<<e.what());
    return false;
  }
  return true;
}

void System::publishPoseAndInfo(const std_msgs::Header & header)
{
  ParamsAccess Params;
  double scale = Params.varParams->Scale;

  static float fps = 0;
  static double last_time = 0;

  if (scale <= 0)
  {
    ROS_WARN_STREAM("scale ("<<scale<<") <= 0, set to 1");
    scale = 1;
  }

  if (mpTracker->getTrackingQuality() && mpMap->IsGood())
  {
    TooN::SE3<double> pose = mpTracker->GetCurrentPose();
    TooN::Matrix<3, 3, double> r = pose.get_rotation().get_matrix();
    TooN::Vector<3, double> & t = pose.get_translation();

    tf::StampedTransform transform(tf::Transform(btMatrix3x3(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2),
                                                             r(2, 0), r(2, 1), r(2, 2)), btVector3(t[0] / scale, t[1]
        / scale, t[2] / scale)), header.stamp, header.frame_id, Params.fixParams->parent_frame);

    tf_pub_.sendTransform(transform);

    if (pub_pose_.getNumSubscribers() > 0)
    {
      geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(new geometry_msgs::PoseWithCovarianceStamped);
      const tf::Quaternion & q = transform.getRotation();
      const tf::Vector3 & t = transform.getOrigin();
      msg_pose->pose.pose.orientation.w = q.w();
      msg_pose->pose.pose.orientation.x = q.x();
      msg_pose->pose.pose.orientation.y = q.y();
      msg_pose->pose.pose.orientation.z = q.z();
      msg_pose->pose.pose.position.x = t.x();
      msg_pose->pose.pose.position.y = t.y();
      msg_pose->pose.pose.position.z = t.z();

      TooN::Matrix<6> covar = mpTracker->GetCurrentCov();
      for (unsigned int i = 0; i < msg_pose->pose.covariance.size(); i++)
        msg_pose->pose.covariance[i] = sqrt(fabs(covar[i % 6][i / 6]));

      msg_pose->header = header;

      pub_pose_.publish(msg_pose);
    }

    if (pub_info_.getNumSubscribers() > 0)
    {
    	ptam_com::ptam_infoPtr msg_info(new ptam_com::ptam_info);
      double diff = header.stamp.toSec() - last_time;
      if (diff < 1.0 && diff > 0.005)
        fps = fps * 0.8 + 0.2 / diff;
      if (fabs(fps) > 200)
        fps = 200;
      last_time = header.stamp.toSec();

      msg_info->header = header;
      msg_info->fps = fps;
      msg_info->mapQuality = mpMap->bGood;
      msg_info->trackingQuality = mpTracker->getTrackingQuality();
      msg_info->trackerMessage = mpTracker->GetMessageForUser();
//      msg_info->mapViewerMessage = mpMapViewer->GetMessageForUser();
      msg_info->keyframes = mpMap->vpKeyFrames.size();
      pub_info_.publish(msg_info);
    }
  };
}


void System::publishPreviewImage(CVD::Image<CVD::byte> & img, const std_msgs::Header & header)
{
  CVD::Image<TooN::Vector<2> > & grid = mpTracker->ComputeGrid();
  std::list<Trail> & trails = mpTracker->getTrails();
  bool drawGrid = mpTracker->getTrailTrackingComplete();
  bool drawTrails = mpTracker->getTrailTrackingStarted();

  if (pub_preview_image_.getNumSubscribers() > 0)
  {
    CVD::ImageRef sub_size(img.size()/2);
    sensor_msgs::ImagePtr img_msg(new sensor_msgs::Image);
    img_msg->header = header;
    img_msg->encoding = sensor_msgs::image_encodings::MONO8;
    img_msg->width = sub_size.x;
    img_msg->height = sub_size.y;
    img_msg->step = sub_size.x;
    img_msg->is_bigendian = 0;
    img_msg->data.resize(sub_size.x * sub_size.y);

    // subsample image
    CVD::BasicImage<CVD::byte> img_sub((CVD::byte *)&img_msg->data[0], sub_size);
    CVD::halfSample(img, img_sub);

    // set opencv pointer to image
    IplImage * ocv_img = cvCreateImageHeader(cvSize(img_sub.size().x, img_sub.size().y), IPL_DEPTH_8U, 1);
    ocv_img->imageData = (char*)&img_msg->data[0];

    int dim0 = grid.size().x;
    int dim1 = grid.size().y;

    if (drawGrid)
    {
      for (int i = 0; i < dim0; i++)
      {
        for (int j = 0; j < dim1 - 1; j++)
          cvLine( ocv_img, cvPoint(grid[i][j][0]/2, grid[i][j][1]/2), cvPoint(grid[i][j + 1][0]/2, grid[i][j + 1][1]/2),
                 CV_RGB(50, 50, 50)
          );

        for (int j = 0; j < dim1 - 1; j++)
          cvLine(ocv_img, cvPoint(grid[j][i][0]/2, grid[j][i][1]/2), cvPoint(grid[j + 1][i][0]/2, grid[j + 1][i][1]/2),
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
        cvLine(ocv_img, cvPoint(LevelZeroPos(i->irCurrentPos.x, level)/2, LevelZeroPos(i->irCurrentPos.y, level)/2),
               cvPoint(LevelZeroPos(i->irInitialPos.x, level)/2, LevelZeroPos(i->irInitialPos.y, level)/2),
               CV_RGB(0, 0, 0), 2);
      }
    }

    pub_preview_image_.publish(img_msg);
    cvReleaseImageHeader(&ocv_img);
  }
}

//Weiss{
bool System::pointcloudservice(ptam_com::PointCloudRequest & req, ptam_com::PointCloudResponse & resp)
{
	static unsigned int seq=0;
	int dimension   = 4;

	resp.pointcloud.header.seq=seq;
	seq++;
	resp.pointcloud.header.stamp = ros::Time::now();
	resp.pointcloud.height = 1;
	resp.pointcloud.header.frame_id = "/world";
	if(mpMap->bGood)
	{
		resp.pointcloud.width = mpMap->vpPoints.size();
		resp.pointcloud.fields.resize(dimension);
		resp.pointcloud.fields[0].name = "x";
		resp.pointcloud.fields[0].offset = 0*sizeof(float);
		resp.pointcloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[0].count = 1;
		resp.pointcloud.fields[1].name = "y";
		resp.pointcloud.fields[1].offset = 1*sizeof(float);
		resp.pointcloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[1].count = 1;
		resp.pointcloud.fields[2].name = "z";
		resp.pointcloud.fields[2].offset = 2*sizeof(float);
		resp.pointcloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[2].count = 1;
		resp.pointcloud.fields[3].name = "rgb";
		resp.pointcloud.fields[3].offset = 3*sizeof(float);
		resp.pointcloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[3].count = 1;

		resp.pointcloud.point_step = dimension*sizeof(float);
		resp.pointcloud.row_step = resp.pointcloud.point_step * resp.pointcloud.width;
		resp.pointcloud.data.resize(resp.pointcloud.row_step * resp.pointcloud.height);
		resp.pointcloud.is_dense = false;


		unsigned char* dat = &(resp.pointcloud.data[0]);
		unsigned int n=0;
		for(std::vector<MapPoint*>::iterator it=mpMap->vpPoints.begin(); it!=mpMap->vpPoints.end(); ++it,++n)
//		for (unsigned int i = 0; i < mpMap->vpPoints.size(); i++, n++)
		{
			if(n>resp.pointcloud.width-1) break;

//			if(!mpMap->vpPoints[i]->bBad)
//			{
//				MapPoint p = *(mpMap->vpPoints[i]);
//				int lvl = 0xff<<((3-p.nSourceLevel)*8);
//				float* color = reinterpret_cast<float*>(&lvl);
//				float* elementI = (float*) ((char*) resp.pointcloud.data.data() + i*resp.pointcloud.point_step);
//				elementI[0] = (float) p.v3WorldPos[0];
//				elementI[1] = (float) p.v3WorldPos[1];
//				elementI[2] = (float) p.v3WorldPos[2];
//				elementI[3] = *color;
//			}

			MapPoint p = *(*it);

			Vector<3,float> fvec = p.v3WorldPos;
			int lvl = 0xff<<((3-p.nSourceLevel)*8);
			memcpy(dat, &(fvec),3*sizeof(float));
			memcpy(dat+3*sizeof(float),reinterpret_cast<float*>(&lvl),sizeof(float));
			dat+=resp.pointcloud.point_step;
		}
	}
	return true;
}


bool System::keyframesservice(ptam_com::KeyFrame_srvRequest & req, ptam_com::KeyFrame_srvResponse & resp)
{
	// flags: 	negative number = send newest N KeyFrames
	//			zero = send all available KeyFrames
	//			positive number = send all KeyFrames with ID>N

	ParamsAccess Params;
	double scale = Params.varParams->Scale;

	TooN::SE3<double> pose;
	TooN::Matrix<3, 3, double> rot;
	TooN::Vector<3, double> trans;
	tf::Quaternion q;
	tf::Vector3 t;
	geometry_msgs::PoseWithCovarianceStamped buffpose;
	bool takeKF=false;
	int k=0;
	static unsigned int seq=0;

	if(!(mpMap->vpKeyFrames.size()>0) | !mpMap->bGood)
		return false;

	resp.KFids.reserve(mpMap->vpKeyFrames.size());
	resp.KFs.reserve(mpMap->vpKeyFrames.size());

	for(std::vector<KeyFrame*>::reverse_iterator rit=mpMap->vpKeyFrames.rbegin(); rit!=mpMap->vpKeyFrames.rend();++rit)
	{
		if((req.flags>0) & ((*rit)->ID>=req.flags))
			takeKF=true;
		else if((req.flags<0) & (k<=abs(req.flags)))
			takeKF=true;
		else if(req.flags==0)
			takeKF=true;
		else if((req.flags>0) & ((*rit)->ID<req.flags))
			break;
		else if((req.flags<0) & (k>abs(req.flags)))
			break;

		if(takeKF)
		{
			takeKF=false;
			resp.KFids.push_back((*rit)->ID);
			pose = (*rit)->se3CfromW;
			rot =pose.get_rotation().get_matrix();
			trans = pose.get_translation();
			tf::Transform transform(btMatrix3x3(rot(0, 0), rot(0, 1), rot(0, 2),
																	rot(1, 0), rot(1, 1), rot(1, 2),
																	rot(2, 0), rot(2, 1), rot(2, 2)),
																	btVector3(trans[0] / scale, trans[1]/ scale, trans[2] / scale));
			q = transform.getRotation();
			t = transform.getOrigin();
			buffpose.header.seq=seq;
			buffpose.header.stamp=ros::Time::now();
			buffpose.pose.pose.position.x=t[0];
			buffpose.pose.pose.position.y=t[1];
			buffpose.pose.pose.position.z=t[2];
			buffpose.pose.pose.orientation.w=q.w();
			buffpose.pose.pose.orientation.x=q.x();
			buffpose.pose.pose.orientation.y=q.y();
			buffpose.pose.pose.orientation.z=q.z();
			memset(&(buffpose.pose.covariance[0]),0,sizeof(double)*6*6);
			resp.KFs.push_back(buffpose);

			seq++;
		}
		k++;
	}
	return true;
}


//}

void System::quaternionToRotationMatrix(const geometry_msgs::Quaternion & q, TooN::SO3<double> & R)
{
  // stolen from Eigen3 and adapted to TooN

  TooN::Matrix<3, 3, double> res;

  const double tx = 2 * q.x;
  const double ty = 2 * q.y;
  const double tz = 2 * q.z;
  const double twx = tx * q.w;
  const double twy = ty * q.w;
  const double twz = tz * q.w;
  const double txx = tx * q.x;
  const double txy = ty * q.x;
  const double txz = tz * q.x;
  const double tyy = ty * q.y;
  const double tyz = tz * q.y;
  const double tzz = tz * q.z;

  res(0, 0) = 1 - (tyy + tzz);
  res(0, 1) = txy - twz;
  res(0, 2) = txz + twy;
  res(1, 0) = txy + twz;
  res(1, 1) = 1 - (txx + tzz);
  res(1, 2) = tyz - twx;
  res(2, 0) = txz - twy;
  res(2, 1) = tyz + twx;
  res(2, 2) = 1 - (txx + tyy);

  R = res;

//  R = TooN::SO3<double>::exp(TooN::makeVector<double>(q.x, q.y, q.z) * acos(q.w) * 2.0 / sqrt(q.x * q.x + q.y * q.y + q.z * q.z));
}



void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if (sCommand == "quit" || sCommand == "exit")
    ros::shutdown();
}








