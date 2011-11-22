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
	void readFixParams();
};

class ParamsAccess
{
public:
	ParamsAccess(){};
	ParamsAccess(ptam::PtamParamsConfig* _varParams, FixParams* _fixParams){
//		static ptam::PtamParamsConfig *s_ptamParams = _varParams;
//		varParams = s_ptamParams;//_varParams;
//
//		static FixParams* s_ptamFixParams = _fixParams;
//		fixParams = s_ptamFixParams;

		varParams = _varParams;
		fixParams = _fixParams;
	};

	// ptam::PtamParamsConfig* varParams{return varParams;};
	// FixParams* getFixParams(){return fixParams;};

	static ptam::PtamParamsConfig* varParams;
	static FixParams* fixParams;
};


#endif /* PARAMS_H_ */
