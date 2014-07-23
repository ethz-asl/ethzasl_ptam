/*
 * Octomapinterface.cc
 *
 *  Created on: Dec 26, 2012
 *      Author: slynen
 */

#include <ptam/OctomapInterface.h>
#include <ptam_com/OctoMapScan.h>
#include <ptam_com/OctoMapPointArray.h>
#include <tf/tf.h>
#include <std_msgs/String.h>

OctoMapInterface::OctoMapInterface( ros::NodeHandle& nh):nh_(nh)
{
  pub_scan_= nh_.advertise<ptam_com::OctoMapScan> ("vslam/octomapscan", 10);
  pub_points_= nh_.advertise<ptam_com::OctoMapPointArray> ("vslam/octomappoints", 10);

  kfseq_ = 0;
  pointseq_ = 0;
}

OctoMapInterface::~OctoMapInterface()
{

}

void OctoMapInterface::addKeyFrame(KeyFrame::Ptr k)
{

  ptam_com::OctoMapScanPtr msg(new ptam_com::OctoMapScan);

  //assemble the keyframe pose from the SE3 stored with the PTAM KeyFrame strcut
  TooN::SE3<double> pose;
  TooN::Matrix<3, 3, double> rot;
  TooN::Vector<3, double> trans;
  tf::Quaternion q;
  tf::Vector3 t;
  geometry_msgs::PoseWithCovarianceStamped& buffpose = msg->keyFramePose;

  double scale = 1.0;

  pose = k->se3CfromW;
  rot = pose.get_rotation().get_matrix();
  trans = pose.get_translation();
  tf::Transform transform(tf::Matrix3x3(rot(0, 0), rot(0, 1), rot(0, 2),
                                        rot(1, 0), rot(1, 1), rot(1, 2),
                                        rot(2, 0), rot(2, 1), rot(2, 2)),
                          tf::Vector3(trans[0] / scale, trans[1]/ scale, trans[2] / scale));
  q = transform.getRotation();
  t = transform.getOrigin();
  buffpose.header.seq=kfseq_++;
  buffpose.header.stamp=ros::Time::now();
  buffpose.pose.pose.position.x=t[0];
  buffpose.pose.pose.position.y=t[1];
  buffpose.pose.pose.position.z=t[2];
  buffpose.pose.pose.orientation.w=q.w();
  buffpose.pose.pose.orientation.x=q.x();
  buffpose.pose.pose.orientation.y=q.y();
  buffpose.pose.pose.orientation.z=q.z();
  memset(&(buffpose.pose.covariance[0]),0,sizeof(double)*6*6);

  //add all points that this KF measures
  msg->mapPoints.mapPoints.reserve(k->mMeasurements.size());
  for(std::map<boost::shared_ptr<MapPoint>, Measurement>::const_iterator it = k->mMeasurements.begin() ; it != k->mMeasurements.end() ; ++it){
    ptam_com::OctoMapPointStamped pt;
    pt.header.seq = pointseq_++;
    pt.header.stamp = ros::Time::now();
    pt.action = ptam_com::OctoMapPointStamped::INSERT;
    pt.position.x = it->first->v3WorldPos[0];
    pt.position.y = it->first->v3WorldPos[1];
    pt.position.z = it->first->v3WorldPos[2];
    msg->mapPoints.mapPoints.push_back(pt);
  }

  pub_scan_.publish(msg);
}

void OctoMapInterface::updatePoint(MapPoint::Ptr p){

  localUpdateQueue_.insert(p); //slow down the update rate a little, this function is called at approx 1KHz

  publishPointUpdateFromQueue(); //request publishing of local queue
}

void OctoMapInterface::updatePoints(std::set<MapPoint::Ptr>& updateSet){

  localUpdateQueue_.insert(updateSet.begin(), updateSet.end());

  publishPointUpdateFromQueue(); //request publishing of local queue

}

void OctoMapInterface::publishPointUpdateFromQueue(){

  static double lastTime = 0;
  if(pub_points_.getNumSubscribers()){
    if(ros::Time::now().toSec() - lastTime > 1.0){ //only perform this once a second
      ptam_com::OctoMapPointArrayPtr msg(new ptam_com::OctoMapPointArray);
      for(std::set<MapPoint::Ptr>::const_iterator it = localUpdateQueue_.begin(); it != localUpdateQueue_.end() ; ++it){
        ptam_com::OctoMapPointStamped pt;
        pt.header.seq = pointseq_++;
        pt.header.stamp = ros::Time::now();
        pt.action = ptam_com::OctoMapPointStamped::UPDATE;
        pt.position.x = (*it)->v3WorldPos[0];
        pt.position.y = (*it)->v3WorldPos[1];
        pt.position.z = (*it)->v3WorldPos[2];
        msg->mapPoints.push_back(pt);
      }
      pub_points_.publish(msg);

      lastTime = ros::Time::now().toSec();
      localUpdateQueue_.clear();

      //    std::cout<<"publishing now "<<lastTime - ((int)lastTime)%1000<<std::endl;

    }
  }
}

void OctoMapInterface::deletePoint(MapPoint::Ptr p){
  ptam_com::OctoMapPointArrayPtr msg(new ptam_com::OctoMapPointArray);
  ptam_com::OctoMapPointStamped pt;
  pt.header.seq = pointseq_++;
  pt.header.stamp = ros::Time::now();
  pt.action = ptam_com::OctoMapPointStamped::DELETE;
  pt.position.x = p->v3WorldPos[0];
  pt.position.y = p->v3WorldPos[1];
  pt.position.z = p->v3WorldPos[2];
  msg->mapPoints.push_back(pt);
  pub_points_.publish(msg);
}
