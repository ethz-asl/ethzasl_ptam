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
#include <set>

class OctoMapInterface
{
private:
  ros::NodeHandle& nh_;
  ros::Publisher pub_scan_;
  ros::Publisher pub_points_;

  std::set<MapPoint::Ptr> localUpdateQueue_; //to collect updates before publishing them all at once

  unsigned int kfseq_;
  unsigned int pointseq_;
public:
  OctoMapInterface( ros::NodeHandle& nh);
  virtual ~OctoMapInterface();

  void addKeyFrame(KeyFrame::Ptr k);
  void updatePoint(MapPoint::Ptr p);
  void updatePoints(std::set<MapPoint::Ptr>& p);
  void deletePoint(MapPoint::Ptr p);

protected:
  void publishPointUpdateFromQueue();

};

#endif /* PCLINTERFACE_H_ */
