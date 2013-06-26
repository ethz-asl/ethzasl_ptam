/*
 * Params.h
 *
 *  Created on: Jul 21, 2010
 *      Author: sweiss
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include <ptam/PtamParamsConfig.h>
#include <string.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <dynamic_reconfigure/server.h>
#include <ptam/Params.h>

typedef dynamic_reconfigure::Server<ptam::PtamParamsConfig> PtamParamsReconfigureServer;
typedef ptam::PtamParamsConfig VarParams;

class FixParams
{
public:
  int ImageSizeX;
  int ImageSizeY;
  int ARBuffer_width;
  int ARBuffer_height;
  double WiggleScale;
  std::string BundleMEstimator;
  std::string TrackerMEstimator;
  double MinTukeySigma;
  int CandidateMinSTScore;
  double Cam_fx;
  double Cam_fy;
  double Cam_cx;
  double Cam_cy;
  double Cam_s;
  double MaxFoV;
  double Calibrator_BlurSigma;
  double Calibrator_MeanGate;
  int Calibrator_MinCornersForGrabbedImage;
  bool Calibrator_Optimize;
  bool Calibrator_Show;
  bool Calibrator_NoDistortion;
  double CameraCalibrator_MaxStepDistFraction;
  int CameraCalibrator_CornerPatchSize;
  bool GLWindowMenu_Enable;
  int GLWindowMenu_mgvnMenuItemWidth;
  int GLWindowMenu_mgvnMenuTextOffset;
  int InitLevel;
  std::string parent_frame;
  bool gui;
  void readFixParams();
};

class PtamParameters{
private:
  ptam::PtamParamsConfig mVarParams;
  FixParams mFixParams;

  PtamParamsReconfigureServer *mpPtamParamsReconfigureServer;

  void ptamParamsConfig(ptam::PtamParamsConfig & config, uint32_t level){
    mVarParams = config;
  };
  static PtamParameters* inst_;
  PtamParameters()
  {
    mpPtamParamsReconfigureServer = new PtamParamsReconfigureServer(ros::NodeHandle("~"));
    PtamParamsReconfigureServer::CallbackType PtamParamCall = boost::bind(&PtamParameters::ptamParamsConfig, this, _1, _2);
    mpPtamParamsReconfigureServer->setCallback(PtamParamCall);

    mFixParams.readFixParams();
  }
  ~PtamParameters(){
    delete inst_;
    inst_ = NULL;
  }
public:
  static const ptam::PtamParamsConfig& varparams(){
    if(!inst_){
      inst_ = new PtamParameters;
    }
    return inst_->mVarParams;
  }
  static const FixParams& fixparams(){
    if(!inst_){
      inst_ = new PtamParameters;
    }
    return inst_->mFixParams;
  }
};

#endif /* PARAMS_H_ */
