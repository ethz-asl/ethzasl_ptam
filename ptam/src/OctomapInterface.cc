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
#include <ptam/MapMaker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

OctoMapInterface::OctoMapInterface( ros::NodeHandle& nh, Map& map):nh_(nh), map_(map)
{
  //  pub_scan_= nh_.advertise<ptam_com::OctoMapScan> ("octomap/scan", 10);
  //  pub_points_= nh_.advertise<ptam_com::OctoMapPointArray> ("octomap/points", 10);

  pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", 10);

  kfseq_ = 0;
  pointseq_ = 0;
  pclseq_ = 0;
}

OctoMapInterface::~OctoMapInterface()
{

}

void OctoMapInterface::addKeyFrame(KeyFrame::Ptr k)
{

  int minobsperpoint = 3; //how many keyframes must observe a point before it is accepted

  sensor_msgs::PointCloud2Ptr pclmsg(new sensor_msgs::PointCloud2);

  int dimension   = 6;

  pclmsg->header.seq = pclseq_++;
  pclmsg->header.stamp = ros::Time::now();
  pclmsg->height = 1;
  pclmsg->header.frame_id = "/world";

  pclmsg->fields.resize(dimension);
  pclmsg->fields[0].name = "x";
  pclmsg->fields[0].offset = 0*sizeof(uint32_t);
  pclmsg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  pclmsg->fields[0].count = 1;
  pclmsg->fields[1].name = "y";
  pclmsg->fields[1].offset = 1*sizeof(uint32_t);
  pclmsg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  pclmsg->fields[1].count = 1;
  pclmsg->fields[2].name = "z";
  pclmsg->fields[2].offset = 2*sizeof(uint32_t);
  pclmsg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  pclmsg->fields[2].count = 1;
  pclmsg->fields[3].name = "rgb";
  pclmsg->fields[3].offset = 3*sizeof(uint32_t);
  pclmsg->fields[3].datatype = sensor_msgs::PointField::INT32;
  pclmsg->fields[3].count = 1;
  pclmsg->fields[4].name = "KF";
  pclmsg->fields[4].offset = 4*sizeof(uint32_t);
  pclmsg->fields[4].datatype = sensor_msgs::PointField::INT32;
  pclmsg->fields[4].count = 1;
  pclmsg->fields[5].name = "lvl";
  pclmsg->fields[5].offset = 5*sizeof(uint32_t);
  pclmsg->fields[5].datatype = sensor_msgs::PointField::INT32;
  pclmsg->fields[5].count = 1;

  pclmsg->point_step = dimension*sizeof(uint32_t);
  pclmsg->is_dense = false;

//resize for maximum space needed, some points however might not be copied, so this needs to be truncated later
  pclmsg->width = k->mMeasurements.size();
  pclmsg->row_step = pclmsg->point_step * pclmsg->width;
  pclmsg->data.resize(pclmsg->row_step * pclmsg->height);

  unsigned char* dat = &( pclmsg->data[0]);
  unsigned int npts = 0;

  for(std::map<boost::shared_ptr<MapPoint>, Measurement>::const_iterator it = k->mMeasurements.begin() ; it != k->mMeasurements.end() ; ++it){

    mappointCache_[it->first].insert(k); //add the map point to the cache and/or update the keyframe as observation of this point

    int numobservingKFs = mappointCache_[it->first].size();
    if(numobservingKFs < minobsperpoint) //reject point if has not been observed often enough
      continue;

    //calculate the maximum observation angle of this point as reliability measure
    std::vector<double> obsAngles;
    Eigen::Vector3d pointloc(it->first->v3WorldPos[0], it->first->v3WorldPos[1], it->first->v3WorldPos[2]);

    for(std::set<KeyFrame::Ptr>::const_iterator obsscan1it = mappointCache_[it->first].begin() ; obsscan1it != mappointCache_[it->first].end() ; ++obsscan1it){
      const KeyFrame::Ptr& obsscan1 = *obsscan1it;
      if(!obsscan1) continue; //we don't have this scan cached yet, for whatever reason
      std::set<KeyFrame::Ptr>::const_iterator obsscan2it = obsscan1it;
      ++obsscan2it;
      for(; obsscan2it != mappointCache_[it->first].end() ; ++obsscan2it){ //for all other key frames
        const  KeyFrame::Ptr& obsscan2 = *obsscan2it;;
        //calculate the angle between the two observations
        Eigen::Vector3d obs1 = Eigen::Vector3d(obsscan1->se3CfromW.get_translation()[0], obsscan1->se3CfromW.get_translation()[1], obsscan1->se3CfromW.get_translation()[2]) - pointloc;
        Eigen::Vector3d obs2 = Eigen::Vector3d(obsscan2->se3CfromW.get_translation()[0], obsscan2->se3CfromW.get_translation()[1], obsscan2->se3CfromW.get_translation()[2]) - pointloc;
        double angle = acos(obs1.dot(obs2) / (obs1.norm() * obs2.norm()));
        obsAngles.push_back(angle);
      }
    }
    //median angle of observation
//    std::nth_element(obsAngles.begin(), obsAngles.begin() + std::floor(obsAngles.size() / 2), obsAngles.end());
//    double median = *(obsAngles.begin() + std::floor(obsAngles.size() / 2));

    //max angle of observation
    double max = *(std::max_element(obsAngles.begin(), obsAngles.end()));

    if(max < 0.1) //reject if this point is only weakly constrained with the current scans
      continue;

    assert(npts < pclmsg->width-1);

    const MapPoint& p = *it->first;

    Vector<3,float> fvec = p.v3WorldPos;
    uint32_t colorlvl = 0xff<<((3-p.nSourceLevel)*8);
    uint32_t lvl = p.nSourceLevel;
    uint32_t KF = p.pPatchSourceKF->ID;

    memcpy(dat, &(fvec),3*sizeof(float));
    memcpy(dat+3*sizeof(uint32_t),&colorlvl,sizeof(uint32_t));
    memcpy(dat+4*sizeof(uint32_t),&lvl,sizeof(uint32_t));
    memcpy(dat+5*sizeof(uint32_t),&KF,sizeof(uint32_t));
    dat+= pclmsg->point_step;


    npts++; //to set the width of the point cloud correctly

  }

  //truncate and adjust the size of data
  pclmsg->width = npts;
  pclmsg->row_step = pclmsg->point_step * pclmsg->width;
  pclmsg->data.resize(pclmsg->row_step * pclmsg->height);

  pub_pcl_.publish(pclmsg);

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

//  static double lastTime = 0;
//
//  if(ros::Time::now().toSec() - lastTime > 1.0){ //only perform this once a second
//    ptam_com::OctoMapPointArrayPtr msg(new ptam_com::OctoMapPointArray);
//    for(std::set<MapPoint::Ptr>::const_iterator it = localUpdateQueue_.begin(); it != localUpdateQueue_.end() ; ++it){
//      ptam_com::OctoMapPointStamped pt;
//      pt.header.seq = pointseq_++;
//      pt.header.stamp = ros::Time::now();
//      pt.action = ptam_com::OctoMapPointStamped::UPDATE;
//      pt.position.x = (*it)->v3WorldPos[0];
//      pt.position.y = (*it)->v3WorldPos[1];
//      pt.position.z = (*it)->v3WorldPos[2];
//      pt.id = (*it)->iID;
//
//      //find all keyFrames which measure this point and store with point
//      for(std::set<KeyFrame::Ptr>::const_iterator itKF = (*it)->pMMData->sMeasurementKFs.begin();
//          itKF != (*it)->pMMData->sMeasurementKFs.end(); ++itKF){
//        pt.observingScanIDs.push_back((*itKF)->ID);
//      }
//
//      msg->mapPoints.push_back(pt);
//    }
//    pub_points_.publish(msg);
//
//    lastTime = ros::Time::now().toSec();
//    localUpdateQueue_.clear();
//  }
}

void OctoMapInterface::deletePoint(MapPoint::Ptr p){
//  ptam_com::OctoMapPointArrayPtr msg(new ptam_com::OctoMapPointArray);
//  ptam_com::OctoMapPointStamped pt;
//  pt.header.seq = pointseq_++;
//  pt.header.stamp = ros::Time::now();
//  pt.action = ptam_com::OctoMapPointStamped::DELETE;
//  pt.id = p->iID;
//  msg->mapPoints.push_back(pt);
//  pub_points_.publish(msg);
}
