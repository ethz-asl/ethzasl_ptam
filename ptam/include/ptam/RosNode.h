/*
 * RosNode.h
 *
 *  Created on: Feb 17, 2010
 *      Author: acmarkus
 */

#ifndef ROSNODE_H_
#define ROSNODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/CvBridge.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ptam_com/ptam_info.h>
#include <ptam_com/ptam_command.h>
#include <visualization_msgs/Marker.h>

//slynen{
#include "std_msgs/String.h"
//}
//#include <boost/bind.hpp>
//#include <boost/thread.hpp>


#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/utility.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>

#include <list>
#include <vector>

#include <ptam/Tracker.h>
#include <ptam/ATANCamera.h>

#include <dynamic_reconfigure/server.h>
#include <ptam/Params.h>

typedef dynamic_reconfigure::Server<ptam::PtamParamsConfig> PtamParamsReconfigureServer;

namespace RosNode {



//class RosNode {
//private:
//	ros::NodeHandle *nh_;
//
//
//	bool rosThreadIsRunning_;
//	bool rosThreadStop_;
//	boost::thread rosThreadInfo_;
//	void rosThread();
//
//public:
//	RosNode(int argc, char** argv);
//	ros::NodeHandle & getNodeHandle(){return *nh_;}
//
//	// TODO: remove this and set ptam parameters in a more clean way
//	std::string getPath(std::string packageName){return ros::package::getPath(packageName);}
//	virtual ~RosNode();
//};

class PtamParameters{
private:
	ptam::PtamParamsConfig mVarParams;
	FixParams mFixParams;

	PtamParamsReconfigureServer *mpPtamParamsReconfigureServer;

	void ptamParamsConfig(ptam::PtamParamsConfig & config, uint32_t level){
		mVarParams = config;
	};
public:
	PtamParameters()
	{
		mpPtamParamsReconfigureServer = new PtamParamsReconfigureServer(ros::NodeHandle("~"));
		PtamParamsReconfigureServer::CallbackType PtamParamCall = boost::bind(&PtamParameters::ptamParamsConfig, this, _1, _2);
		mpPtamParamsReconfigureServer->setCallback(PtamParamCall);

		mFixParams.readFixParams();

		ParamsAccess pAccess(&mVarParams, &mFixParams);
	}
};

class Publisher{
private:
	tf::TransformBroadcaster transformBroadcaster_;
	tf::StampedTransform transform_;

	ros::Publisher pubPose_;
	geometry_msgs::PoseWithCovarianceStamped msgPose_;

	ros::Publisher pubInfo_;
	ptam_com::ptam_info msgInfo_;

public:
	Publisher();
	bool publishPose(TooN::SE3<> cameraPose, TooN::Matrix<6> covariance, int trackingQuality, bool mapQuality, double timestamp, const std::string & frameId);
	bool publishPtamInfo(int trackingQuality, bool mapQuality, const std::string & mapViewerMessage, const std::string & trackerMessage, int nKeyframes, double timestamp, const std::string & frameId);
};

class RemoteInterface{
private:
	bool newCommand_;
	std::string command_;
	std::string response_;
	ros::ServiceServer commandServer_;
	bool commandCallback(ptam_com::ptam_commandRequest & req, ptam_com::ptam_commandResponse & res);

	image_transport::ImageTransport it_;
	image_transport::Publisher previewPublisher_;
	sensor_msgs::ImagePtr prevImage_;
	//slynen{
	ros::Publisher pubKbInput_;
	void receiveKbInput(const std_msgs::String::ConstPtr& msg);
	bool publishKbInput(const std::string & _keysPressed);
	ros::Subscriber subKbInput_;
	//}
public:
	RemoteInterface();
    bool newCommand();
    std::string getCommand();
    void setResponse(const std::string & response);
    void publishPreview(CVD::Image<CVD::byte> &imBW, CVD::Image<TooN::Vector<2> > & grid, std::list<Trail> & trails, bool drawGrid = false, bool drawTrails = false);
};

class Visualization{
private:
	ros::Publisher trailPublisher_;
	visualization_msgs::Marker msgTrails_;
	ros::Publisher gridPublisher_;
	visualization_msgs::Marker msgGrid_;
	ATANCamera camera_;
public:
	Visualization();
	void publishTrails(std::list<Trail> & trails);

};

}

#endif /* ROSNODE_H_ */
