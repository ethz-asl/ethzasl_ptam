// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the MapMaker class
// MapMaker makes and maintains the Map struct
// Starting with stereo initialisation from a bunch of matches
// over keyframe insertion, continual bundle adjustment and 
// data-association refinement.
// MapMaker runs in its own thread, although some functions
// (notably stereo init) are called by the tracker and run in the 
// tracker's thread.

#ifndef __MAPMAKER_H
#define __MAPMAKER_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>

#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include "SmallBlurryImage.h"
#include <queue>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <ptam/OctomapInterface.h>


// Each MapPoint has an associated MapMakerData class
// Where the mapmaker can store extra information

struct MapMakerData
{
  std::set<KeyFrame::Ptr> sMeasurementKFs;   // Which keyframes has this map point got measurements in?
  std::set<KeyFrame::Ptr> sNeverRetryKFs;    // Which keyframes have measurements failed enough so I should never retry?
  inline int GoodMeasCount()            
  {  return sMeasurementKFs.size(); }
};

// MapMaker dervives from CVD::Thread, so everything in void run() is its own thread.
class MapMaker : protected CVD::Thread
{
public:
  MapMaker(Map &m, const ATANCamera &cam, ros::NodeHandle& nh);
  ~MapMaker();

  // Make a map from scratch. Called by the tracker.
  bool InitFromStereo(KeyFrame::Ptr kFirst, KeyFrame::Ptr kSecond,
                      std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                      SE3<> &se3CameraPos);

  bool InitFromStereo_OLD(KeyFrame::Ptr kFirst, KeyFrame::Ptr kSecond,  // EXPERIMENTAL HACK
                          std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                          SE3<> &se3CameraPos);


  void AddKeyFrame(KeyFrame::Ptr k);   // Add a key-frame to the map. Called by the tracker.
  void RequestReset();   // Request that the we reset. Called by the tracker.
  bool ResetDone();      // Returns true if the has been done.
  int  QueueSize() { return mvpKeyFrameQueue.size() ;} // How many KFs in the queue waiting to be added?
  bool NeedNewKeyFrame(KeyFrame::Ptr kCurrent);            // Is it a good camera pose to add another KeyFrame?
  bool IsDistanceToNearestKeyFrameExcessive(KeyFrame::Ptr kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)

  std::string getMessageForUser(){return mMessageForUser.str();};
  void resetMessageForUser(){mMessageForUser.str(""); };

  //Weiss{ moved from protected
  void ApplyGlobalScaleToMap(double dScale);
  void ApplyGlobalTransformationToMap(SE3<> se3NewFromOld);
  KeyFrame::Ptr ClosestKeyFrame(KeyFrame::Ptr k);
  std::vector<KeyFrame::Ptr> NClosestKeyFrames(KeyFrame::Ptr k, unsigned int N);
  //}

protected:

  Map &mMap;               // The map
  ATANCamera mCamera;      // Same as the tracker's camera: N.B. not a reference variable!
  virtual void run();      // The MapMaker thread code lives here

  //slynen octomap_interface{
  OctoMapInterface octomap_interface;
  //}

  // Functions for starting the map from scratch:
  SE3<> CalcPlaneAligner();

  // Map expansion functions:
  void AddKeyFrameFromTopOfQueue();  
  void ThinCandidates(KeyFrame::Ptr k, int nLevel);
  bool AddSomeMapPoints(int nLevel);
  bool AddPointEpipolar(KeyFrame::Ptr kSrc, KeyFrame::Ptr kTarget, int nLevel, int nCandidate);
  // Returns point in ref frame B
  Vector<3> ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B);

  // Bundle adjustment functions:
  void BundleAdjust(std::set<KeyFrame::Ptr>, std::set<KeyFrame::Ptr>, std::set<boost::shared_ptr<MapPoint> >, bool);
  void BundleAdjustAll();
  void BundleAdjustRecent();

  // Data association functions:
  int ReFindInSingleKeyFrame(KeyFrame::Ptr k);
  void ReFindFromFailureQueue();
  void ReFindNewlyMade();
  void ReFindAll();
  bool ReFind_Common(KeyFrame::Ptr k, boost::shared_ptr<MapPoint> p);
  void SubPixelRefineMatches(KeyFrame::Ptr k, int nLevel);

  // General Maintenance/Utility:
  void Reset();
  void HandleBadPoints();
  double DistToNearestKeyFrame(KeyFrame::Ptr kCurrent);
  double KeyFrameLinearDist(KeyFrame::Ptr k1, KeyFrame::Ptr k2);
  bool RefreshSceneDepth(KeyFrame::Ptr pKF);


  // GUI Interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;


  // Member variables:
  std::vector<KeyFrame::Ptr> mvpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
  std::vector<std::pair<KeyFrame::Ptr, boost::shared_ptr<MapPoint> > > mvFailureQueue; // Queue of failed observations to re-find
  std::queue<boost::shared_ptr<MapPoint> > mqNewQueue;   // Queue of newly-made map points to re-find in other KeyFrames

  double mdWiggleScale;  // Metric distance between the first two KeyFrames (copied from GVar)
  // This sets the scale of the map
  //GVars3::gvar3<double> mgvdWiggleScale;   // GVar for above
  double mdWiggleScaleDepthNormalized;  // The above normalized against scene depth, 
  // this controls keyframe separation

  bool mbBundleConverged_Full;    // Has global bundle adjustment converged?
  bool mbBundleConverged_Recent;  // Has local bundle adjustment converged?

  // Thread interaction signalling stuff
  bool mbResetRequested;   // A reset has been requested
  bool mbResetDone;        // The reset was done.
  bool mbBundleAbortRequested;      // We should stop bundle adjustment
  bool mbBundleRunning;             // Bundle adjustment is running
  bool mbBundleRunningIsRecent;     //    ... and it's a local bundle adjustment.


  std::ostringstream mMessageForUser;
};

#endif


















