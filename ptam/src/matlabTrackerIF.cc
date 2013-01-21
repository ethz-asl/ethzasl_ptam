/*
 * matlabTrackerIF.cc
 *
 *  Created on: Jan 21, 2013
 *      Author: slynen
 */

#include "ptam/OpenGL.h"
#include "ptam/Tracker.h"
#include "ptam/MEstimator.h"
#include "ptam/ShiTomasi.h"
#include "ptam/SmallMatrixOpts.h"
#include "ptam/PatchFinder.h"
#include "ptam/TrackerData.h"
#include <ptam/Params.h>

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <TooN/wls.h>
#include <TooN/SVD.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include <cv.h>

#include <fstream>
#include <fcntl.h>
#include <math.h>

#include <mex.h>

using namespace TooN;
using namespace std;

class IF{
public:
  Matrix<6> mmCovariances;          // covariance of current converged pose estimate
  Matrix<6> mmCInv;

  Vector<6> CalcPoseUpdate(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers)
                                                                                        {
    // Which M-estimator are we using?
    int nEstimator = 0;
    //Weiss{
    //static std::string gvsEstimator = "Tukey";
    static std::string gvsEstimator = "Tukey";
    //}
    if(gvsEstimator == "Tukey")
      nEstimator = 0;
    else if(gvsEstimator == "Cauchy")
      nEstimator = 1;
    else if(gvsEstimator == "Huber")
      nEstimator = 2;
    else
    {
      cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
      nEstimator = 0;
      gvsEstimator = "Tukey";
    };

    // Find the covariance-scaled reprojection error for each measurement.
    // Also, store the square of these quantities for M-Estimator sigma squared estimation.
    vector<double> vdErrorSquared;
    for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
        continue;
      TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
      vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
    };

    // No valid measurements? Return null update.
    if(vdErrorSquared.size() == 0)
      return makeVector( 0,0,0,0,0,0);

    // What is the distribution of errors?
    double dSigmaSquared;
    if(dOverrideSigma > 0)
      dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
    else
    {
      if (nEstimator == 0)
        dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
      else if(nEstimator == 1)
        dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
      else
        dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
    }

    // The TooN WLSCholesky class handles reweighted least squares.
    // It just needs errors and jacobians.
    WLS<6> wls;
    wls.add_prior(100.0); // Stabilising prior
    for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
        continue;
      Vector<2> &v2 = TD.v2Error_CovScaled;
      double dErrorSq = v2 * v2;
      double dWeight;

      if(nEstimator == 0)
        dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
      else if(nEstimator == 1)
        dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
      else
        dWeight= Huber::Weight(dErrorSq, dSigmaSquared);

      // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
      if(dWeight == 0.0)
      {
        if(bMarkOutliers)
          TD.Point->nMEstimatorOutlierCount++;
        continue;
      }
      else
        if(bMarkOutliers)
          TD.Point->nMEstimatorInlierCount++;

      Matrix<2,6> &m26Jac = TD.m26Jacobian;
      wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
      wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits

    }

    wls.compute();

    //Weiss{
    if (bMarkOutliers){    //only true for last iteration, see code above... (iter==9)
      mmCovariances=TooN::SVD<6>(wls.get_C_inv()).get_pinv();
      mmCInv = wls.get_C_inv();
    }
    //}
    return wls.get_mu();
                                                                                        }

  void calculateCovariance(std::vector<boost::shared_ptr<MapPoint> >& vpPoints, SE3<>& mse3CamFromWorld, ATANCamera& mCamera, double pixelnoise){

    vector<TrackerData*> avPVS[LEVELS];
    // For all points in the map..
    for(unsigned int i=0; i<vpPoints.size(); i++)
    {
      MapPoint::Ptr p= (vpPoints[i]);
      // Ensure that this map point has an associated TrackerData struct.
      if(!p->pTData) p->pTData = new TrackerData(p);
      TrackerData &TData = *p->pTData;

      // Project according to current view, and if it's not in the image, skip.
      TData.Project(mse3CamFromWorld, mCamera);
      if(!TData.bInImage){
        std::cout<<"point "<<i<<" is invalid. not in the image"<<std::endl;
        continue;
      }

      // Calculate camera projection derivatives of this point.
      TData.GetDerivsUnsafe(mCamera);

      //      // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
      //      TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
      //      if(TData.nSearchLevel == -1){
      //        std::cout<<"point "<<i<<" is invalid. Warpmatrix calc failed"<<std::endl;
      //        continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.
      //      }
      // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
      TData.bSearched = false;
      TData.bFound = false;
      avPVS[TData.nSearchLevel].push_back(&TData);
    };
    std::cout<<"ok, calculated derivs and proj matrices."<<std::endl;

    // Next: A large degree of faffing about and deciding which points are going to be measured!
    // First, randomly shuffle the individual levels of the PVS.
    //    for(int i=0; i<LEVELS; i++)
    //      random_shuffle(avPVS[i].begin(), avPVS[i].end());

    // The next two data structs contain the list of points which will next
    // be searched for in the image, and then used in pose update.
    vector<TrackerData*> vNextToSearch;
    vector<TrackerData*> vIterationSet;

    // Tunable parameters to do with the coarse tracking stage:

    //Weiss{
    //    ParamsAccess Params;
    //    ptam::PtamParamsConfig* pPars = Params.varParams;
    //    unsigned int gvnCoarseMin = pPars->CoarseMin;
    //    unsigned int gvnCoarseMax = pPars->CoarseMax;
    //    unsigned int gvnCoarseRange = pPars->CoarseRange;
    //    int gvnCoarseSubPixIts = pPars->CoarseSubPixIts;
    //    int gvnCoarseDisabled = pPars->DisableCoarse;
    //    double gvdCoarseMinVel = pPars->CoarseMinVelocity;
    //    //static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT);   // Min number of large-scale features for coarse stage
    //    //static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
    //    //static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT);       // Pixel search radius for coarse features
    //    //static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
    //    //static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
    //    //static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, SILENT);  // Speed above which coarse stage is used.
    //    //}
    //    unsigned int nCoarseMax = gvnCoarseMax;
    //    unsigned int nCoarseRange = gvnCoarseRange;
    //
    //    mbDidCoarse = false;
    //
    //    // Set of heuristics to check if we should do a coarse tracking stage.
    //    bool bTryCoarse = true;
    //    if(gvnCoarseDisabled ||
    //        mdMSDScaledVelocityMagnitude < gvdCoarseMinVel  ||
    //        nCoarseMax == 0)
    //      bTryCoarse = false;
    //    if(mbJustRecoveredSoUseCoarse)
    //    {
    //      bTryCoarse = true;
    //      nCoarseMax *=2;
    //      nCoarseRange *=2;
    //      mbJustRecoveredSoUseCoarse = false;
    //    };

    //    // If we do want to do a coarse stage, also check that there's enough high-level
    //    // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
    //    // with preference to LEVELS-1.
    //    if(bTryCoarse && avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size() > gvnCoarseMin )
    //    {
    //      // Now, fill the vNextToSearch struct with an appropriate number of
    //      // TrackerDatas corresponding to coarse map points! This depends on how many
    //      // there are in different pyramid levels compared to CoarseMin and CoarseMax.
    //
    //      if(avPVS[LEVELS-1].size() <= nCoarseMax)
    //      { // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
    //        vNextToSearch = avPVS[LEVELS-1];
    //        avPVS[LEVELS-1].clear();
    //      }
    //      else
    //      { // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
    //        for(unsigned int i=0; i<nCoarseMax; i++)
    //          vNextToSearch.push_back(avPVS[LEVELS-1][i]);
    //        avPVS[LEVELS-1].erase(avPVS[LEVELS-1].begin(), avPVS[LEVELS-1].begin() + nCoarseMax);
    //      }
    //
    //      // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
    //      if(vNextToSearch.size() < nCoarseMax)
    //      {
    //        unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
    //        if(avPVS[LEVELS-2].size() <= nMoreCoarseNeeded)
    //        {
    //          vNextToSearch = avPVS[LEVELS-2];
    //          avPVS[LEVELS-2].clear();
    //        }
    //        else
    //        {
    //          for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
    //            vNextToSearch.push_back(avPVS[LEVELS-2][i]);
    //          avPVS[LEVELS-2].erase(avPVS[LEVELS-2].begin(), avPVS[LEVELS-2].begin() + nMoreCoarseNeeded);
    //        }
    //      }
    //      // Now go and attempt to find these points in the image!
    //      unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, gvnCoarseSubPixIts);
    //      vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
    //      if(nFound >= gvnCoarseMin)  // Were enough found to do any meaningful optimisation?
    //      {
    //        mbDidCoarse = true;
    //        for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
    //        {
    //          if(iter != 0)
    //          { // Re-project the points on all but the first iteration.
    //            for(unsigned int i=0; i<vIterationSet.size(); i++)
    //              if(vIterationSet[i]->bFound)
    //                vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
    //          }
    //          for(unsigned int i=0; i<vIterationSet.size(); i++)
    //            if(vIterationSet[i]->bFound)
    //              vIterationSet[i]->CalcJacobian();
    //          double dOverrideSigma = 0.0;
    //          // Hack: force the MEstimator to be pretty brutal
    //          // with outliers beyond the fifth iteration.
    //          if(iter > 5)
    //            dOverrideSigma = 1.0;
    //
    //          // Calculate and apply the pose update...
    //          Vector<6> v6Update =
    //              CalcPoseUpdate(vIterationSet, dOverrideSigma);
    //          mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
    //        };
    //      }
    //    };
    //
    //    // So, at this stage, we may or may not have done a coarse tracking stage.
    //    // Now do the fine tracking stage. This needs many more points!
    //
    //    int nFineRange = 10;  // Pixel search range for the fine stage.
    //    if(mbDidCoarse)       // Can use a tighter search if the coarse stage was already done.
    //      nFineRange = 5;
    //
    //    // What patches shall we use this time? The high-level ones are quite important,
    //    // so do all of these, with sub-pixel refinement.
    //    {
    //      int l = LEVELS - 1;
    //      for(unsigned int i=0; i<avPVS[l].size(); i++)
    //        avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
    //
    //
    //
    ////      SearchForPoints(avPVS[l], nFineRange, 8);
    //      for(unsigned int i=0; i<avPVS[l].size(); i++)
    //        vIterationSet.push_back(avPVS[l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
    //    };

    // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
    vNextToSearch.clear();
    for(int l=LEVELS - 1; l>=0; l--)
      for(unsigned int i=0; i<avPVS[l].size(); i++)
        vNextToSearch.push_back(avPVS[l][i]);

    std::cout<<"ok, added to opt set. having "<<vNextToSearch.size()<<" points"<<std::endl;
    //    // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit
    //    // ourselves to 1000, and choose these randomly.
    //    //Weiss{
    //    int gvnMaxPatchesPerFrame = pPars->MaxPatchesPerFrame;
    //    //static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
    //    //}
    //    int nFinePatchesToUse = gvnMaxPatchesPerFrame - vIterationSet.size();
    //    if(((int) vNextToSearch.size() > nFinePatchesToUse) && (nFinePatchesToUse>0))
    //    {
    //      random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
    //      vNextToSearch.resize(nFinePatchesToUse); // Chop!
    //    };
    //
    //    // If we did a coarse tracking stage: re-project and find derivs of fine points
    //    if(mbDidCoarse)
    //      for(unsigned int i=0; i<vNextToSearch.size(); i++)
    //        vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);

    // Find fine points in image:
    //    SearchForPoints(vNextToSearch, nFineRange, 0);

    //perturb the point locations in the image to simulate noise
    for(unsigned int i=0; i<vNextToSearch.size(); i++){
      vNextToSearch[i]->bFound = true;
      vNextToSearch[i]->bDidSubPix = true;
      vNextToSearch[i]->dSqrtInvNoise = (1.0 / 3); //TODO put some some level instead of 3

      Vector<2> perturbation;
      srand(i);
      perturbation[0] = rand()/(double)RAND_MAX * pixelnoise;
      perturbation[1] = rand()/(double)RAND_MAX * pixelnoise;


      vNextToSearch[i]->v2Found = vNextToSearch[i]->v2Image + perturbation;

      std::cout<<"point "<<i<<" perturbation "<<perturbation[0]<<" "<<perturbation[1]<<std::endl;
      std::cout<<"point "<<i<<" v2Image "<<vNextToSearch[i]->v2Image[0]<<" "<<vNextToSearch[i]->v2Image[1]<<std::endl;
      std::cout<<"point "<<i<<" v2Found "<<vNextToSearch[i]->v2Found[0]<<" "<<vNextToSearch[i]->v2Found[1]<<std::endl;

    }

    std::cout<<"ok, added perturbation and simulated observations"<<std::endl;

    // And attach them all to the end of the optimisation-set.
    for(unsigned int i=0; i<vNextToSearch.size(); i++)
      vIterationSet.push_back(vNextToSearch[i]);

    // Again, ten gauss-newton pose update iterations.
    Vector<6> v6LastUpdate;
    v6LastUpdate = Zeros;
    for(int iter = 0; iter<10; iter++)
    {
      std::cout<<"ok, starting iteration "<<iter; std::cout.flush();

      bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
      // reprojection at every iteration - it really isn't necessary!
      if(iter == 0 || iter == 4 || iter == 9)
        bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
      else                            // iterations is for M-Estimator convergence rather than
        bNonLinearIteration = false;  // linearisation effects.

      if(iter != 0)   // Either way: first iteration doesn't need projection update.
      {
        if(bNonLinearIteration)
        {
          for(unsigned int i=0; i<vIterationSet.size(); i++)
            if(vIterationSet[i]->bFound)
              vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
        }
        else
        {
          for(unsigned int i=0; i<vIterationSet.size(); i++)
            if(vIterationSet[i]->bFound)
              vIterationSet[i]->LinearUpdate(v6LastUpdate);
        };
      }

      if(bNonLinearIteration)
        for(unsigned int i=0; i<vIterationSet.size(); i++)
          if(vIterationSet[i]->bFound)
            vIterationSet[i]->CalcJacobian();

      // Again, an M-Estimator hack beyond the fifth iteration.
      double dOverrideSigma = 0.0;
      if(iter > 5)
        dOverrideSigma = 16.0;

      // Calculate and update pose; also store update vector for linear iteration updates.
      Vector<6> v6Update =
          CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
      mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
      v6LastUpdate = v6Update;
      std::cout<<" done."<<std::endl;
    };
  }
};

void doTest(){

  std::cout<<"Running doTest"<<std::endl;

  double Cam_fx = 0.795574;
  double Cam_fy = 1.25149;
  double Cam_cx = 0.50417;
  double Cam_cy = 0.51687;
  double Cam_s = 0.482014;
  double ImageSizeX = 640;
  double ImageSizeY = 480;
  std::vector<boost::shared_ptr<MapPoint> > vpPoints;
  SE3<> mse3CamFromWorld;

  ATANCamera mCamera("CAMERA", Cam_fx, Cam_fy, Cam_cx, Cam_cy, Cam_s, ImageSizeX, ImageSizeY);

  TooN::Matrix<3> rotation;
  rotation(0,0) =  0.991464;
  rotation(0,1) =  0.124893;
  rotation(0,2) =  0.124893;

  rotation(1,0) =  0.117282;
  rotation(1,1) =  -0.97972;
  rotation(1,2) =  -0.162461;

  rotation(2,0) =  -0.0569516;
  rotation(2,1) =  0.156685;
  rotation(2,2) =  -0.986005;

  TooN::Vector<3> translation = TooN::makeVector(0.789354, 0.0709251, 1.64471);

  TooN::SO3<> so3(rotation);
  mse3CamFromWorld = TooN::SE3<>(so3,translation);

  boost::shared_ptr<MapPoint> pt;

  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector(-0.520898, 0.059774, -0.00237556);
  vpPoints.push_back(pt);
  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector( -0.536628, 0.0557534, 0.0170862);
  vpPoints.push_back(pt);
  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector( -0.926411, -0.103911, -0.0103726);
  vpPoints.push_back(pt);
  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector( -0.913931, -0.127581, 0.0193194);
  vpPoints.push_back(pt);
  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector( -0.90015, -0.122935, -0.0160205);
  vpPoints.push_back(pt);
  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector( -0.71537, -0.107164, -0.00494531);
  vpPoints.push_back(pt);
  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector( -0.801244, 0.176297, -0.00502234);
  vpPoints.push_back(pt);
  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector( -0.814134, 0.169979, 0.0189816);
  vpPoints.push_back(pt);
  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector( -0.429025, 0.0571335, -0.000828381);
  vpPoints.push_back(pt);
  pt.reset(new MapPoint);
  pt->v3WorldPos = TooN::makeVector( -0.62596, 0.137993, 0.0405261);
  vpPoints.push_back(pt);

  IF theInterface;

  TrackerData::irImageSize = CVD::ImageRef(ImageSizeX, ImageSizeY);

  theInterface.calculateCovariance(vpPoints, mse3CamFromWorld, mCamera, 2);

  Matrix<6>& cov = theInterface.mmCInv;

  std::cout<<"inverse covariance"<<std::endl;
  for(int i = 0;i<6;++i){
    for(int j = 0;j<6;++j){
      std::cout<<cov(j,i)<<" ";
    }
    std::cout<<std::endl;

  }
}

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
  double pixelnoise;
  double *inPoints;               /* 3xN input points */
  size_t ncols;                   /* size of matrix */
  size_t nrows;
  double *inCamera;               /* 3x4 input camera */
  double *outCovarianceMatrix;              /* output cov matrix */

  /* check for proper number of arguments */
  if(nrhs!=3) {
    mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","Three inputs required.");
  }
  if(nlhs!=1) {
    mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","One output required.");
  }
  /* make sure the first input argument is scalar */
  if( !mxIsDouble(prhs[0]) ||
      mxIsComplex(prhs[0]) ||
      mxGetNumberOfElements(prhs[0])!=1 ) {
    mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notScalar","Input pixelnoise must be a scalar.");
  }

  /* check that number of rows in second input argument is 3 */
  if(mxGetM(prhs[1])!=3) {
    mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notRowVector","Input must be a 3xN Matrix vector.");
  }
  if(mxGetM(prhs[2])!=3 || mxGetN(prhs[2])!=4) {
    mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notRowVector","Camera Position must be a 3x4 Matrix vector.");
  }

  /* get the value of the scalar input  */
  pixelnoise = mxGetScalar(prhs[0]);

  /* create a pointer to the real data in the input matrix  */
  inPoints = mxGetPr(prhs[1]);
  inCamera = mxGetPr(prhs[2]);

  /* get dimensions of the input matrix */
  ncols = mxGetN(prhs[1]);
  nrows = mxGetM(prhs[1]);

  /* create the output matrix */
  plhs[0] = mxCreateDoubleMatrix(6,6,mxREAL);

  /* get a pointer to the real data in the output matrix */
  outCovarianceMatrix = mxGetPr(plhs[0]);


  double Cam_fx = 0.795574;
  double Cam_fy = 1.25149;
  double Cam_cx = 0.50417;
  double Cam_cy = 0.51687;
  double Cam_s = 0.482014;
  double ImageSizeX = 640;
  double ImageSizeY = 480;
  std::vector<boost::shared_ptr<MapPoint> > vpPoints;
  SE3<> mse3CamFromWorld;

  ATANCamera mCamera("CAMERA", Cam_fx, Cam_fy, Cam_cx, Cam_cy, Cam_s, ImageSizeX, ImageSizeY);

  std::cout<<"got "<<ncols<<" points in input Matrix"<<std::endl;

  boost::shared_ptr<MapPoint> pt(new MapPoint);
  for(size_t i = 0;i<ncols;++i){
    pt.reset(new MapPoint);
    for(int j = 0;j<3;++j){
      pt->v3WorldPos[j] = inPoints[i*3+j];
    }
    std::cout<<"point "<<i<<" "<<pt->v3WorldPos[0]<<" "<<pt->v3WorldPos[1]<<" "<<pt->v3WorldPos[2]<<std::endl;
    vpPoints.push_back(pt);
  }
  std::cout<<"now have "<<vpPoints.size()<<" points in map"<<std::endl;

  TooN::Matrix<3> rotation;
  for(int i = 0;i<3;++i){
    for(int j = 0;j<3;++j){
      rotation(j,i) = inCamera[i*3+j];
    }
  }
  TooN::Vector<3> translation;
  for(int i = 0;i<3;++i){
    translation[i] = inCamera[9+i];
  }
  TooN::SO3<> so3(rotation);
  mse3CamFromWorld = TooN::SE3<>(so3,translation);

  std::stringstream ss;
  ss<<"Camera matrix"<<std::endl;
  for(int i = 0;i<3;++i){
    for(int j = 0;j<3;++j){
      ss<<rotation(i,j)<<" ";
    }
    ss<<std::endl;
  }

  ss<<"translation "<<std::endl;

  for(int i = 0;i<3;++i){
    ss<<translation[i]<<" ";
  }
  std::cout<<ss.str()<<std::endl;

  IF theInterface;
  TrackerData::irImageSize = CVD::ImageRef(ImageSizeX, ImageSizeY);

  theInterface.calculateCovariance(vpPoints, mse3CamFromWorld, mCamera, pixelnoise);

  Matrix<6>& cov = theInterface.mmCInv;

  for(int i = 0;i<6;++i){
    for(int j = 0;j<6;++j){
      outCovarianceMatrix[i*6+j] = cov(i,j);
    }
  }

//  std::stringstream ss2;
//  ss2<<"inv covariance"<<std::endl;
//  for(int i = 0;i<6;++i){
//    for(int j = 0;j<6;++j){
//      ss2<<cov(j,i)<<" ";
//    }
//    ss2<<std::endl;
//  }
//  std::cout<<ss2.str()<<std::endl;
}

//has to live here, because we haven't compiled the Tracker sources
CVD::ImageRef TrackerData::irImageSize;

int main(int argc, char** argv){

  doTest();

}
