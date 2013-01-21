/*
 * octomapinterface.h
 *
 *  Created on: Dec 26, 2012
 *      Author: slynen
 */

#ifndef PCLINTERFACE_H_
#define PCLINTERFACE_H_
#include <ptam/MapPoint.h>
#include <ptam/KeyFrame.h>
#include <ptam/Map.h>
#include <set>

class OctoMapInterface
{
private:
  ros::NodeHandle& nh_;
//  ros::Publisher pub_scan_;
//  ros::Publisher pub_points_;
  ros::Publisher pub_pcl_;
  int pclseq_;

  Map& map_;
  std::map<MapPoint::Ptr, std::set<KeyFrame::Ptr> > mappointCache_;

  typedef MapPoint::Ptr UpdateRequest_type;
  std::set<UpdateRequest_type> localUpdateQueue_; //to collect updates before publishing them all at once

  unsigned int kfseq_;
  unsigned int pointseq_;
public:
  OctoMapInterface( ros::NodeHandle& nh, Map& map);
  virtual ~OctoMapInterface();

  void addKeyFrame(KeyFrame::Ptr k);
  void updatePoint(MapPoint::Ptr p);
  void updatePoints(std::set<MapPoint::Ptr>& p);
  void deletePoint(MapPoint::Ptr p);

protected:
  void publishPointUpdateFromQueue();

};

#endif /* PCLINTERFACE_H_ */
