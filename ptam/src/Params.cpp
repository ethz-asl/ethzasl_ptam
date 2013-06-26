/*
 * Params.cpp
 *
 *  Created on: Sep 17, 2010
 *      Author: asl
 */

#include <ptam/Params.h>


void FixParams::readFixParams()
{
  ros::NodeHandle nh("~");

  // kill program, if we don't have a camera calibration
  bool have_calibration = true;
  have_calibration = have_calibration && nh.getParam("Cam_fx", Cam_fx);
  have_calibration = have_calibration && nh.getParam("Cam_fy", Cam_fy);
  have_calibration = have_calibration && nh.getParam("Cam_cx", Cam_cx);
  have_calibration = have_calibration && nh.getParam("Cam_cy", Cam_cy);
  have_calibration = have_calibration && nh.getParam("Cam_s", Cam_s);

  if (!have_calibration)
  {
    ROS_ERROR("Camera calibration is missing!");
    //    ROS_BREAK();
  }

  nh.param("ImageSizeX", ImageSizeX, 640);
  nh.param("ImageSizeY", ImageSizeY, 480);
  nh.param("ARBuffer_width", ARBuffer_width, 1200);
  nh.param("ARBuffer_height", ARBuffer_height, 900);
  nh.param("WiggleScale", WiggleScale, 0.1);
  nh.param("BundleMEstimator", BundleMEstimator, std::string("Tukey"));
  nh.param("TrackerMEstimator", TrackerMEstimator, std::string("Tukey"));
  nh.param("MinTukeySigma", MinTukeySigma, 0.4);
  nh.param("CandidateMinSTScore", CandidateMinSTScore, 70);
  //	nh.param("Cam_fx", Cam_fx,0.392);
  //	nh.param("Cam_fy", Cam_fy,0.613);
  //	nh.param("Cam_cx", Cam_cx,0.484);
  //	nh.param("Cam_cy", Cam_cy,0.476);
  //	nh.param("Cam_s", Cam_s,0.967);
  nh.param("Calibrator_BlurSigma", Calibrator_BlurSigma, 1.0);
  nh.param("Calibrator_MeanGate", Calibrator_MeanGate, 10.0);
  nh.param("Calibrator_MinCornersForGrabbedImage", Calibrator_MinCornersForGrabbedImage, 20);
  nh.param("Calibrator_Optimize", Calibrator_Optimize, false);
  nh.param("Calibrator_Show", Calibrator_Show, false);
  nh.param("Calibrator_NoDistortion", Calibrator_NoDistortion, false);
  nh.param("CameraCalibrator_MaxStepDistFraction", CameraCalibrator_MaxStepDistFraction, 0.3);
  nh.param("CameraCalibrator_CornerPatchSize", CameraCalibrator_CornerPatchSize, 20);
  nh.param("GLWindowMenu_Enable", GLWindowMenu_Enable, true);
  nh.param("GLWindowMenu_mgvnMenuItemWidth", GLWindowMenu_mgvnMenuItemWidth, 90);
  nh.param("GLWindowMenu_mgvnMenuTextOffset", GLWindowMenu_mgvnMenuTextOffset, 20);
  nh.param("InitLevel", InitLevel, 1);
  nh.param("parent_frame", parent_frame, std::string("world"));
  nh.param("gui", gui, false);
  nh.param("MaxFoV", MaxFoV, 130.0);
}
;
PtamParameters* PtamParameters::inst_ = NULL;
