// Copyright 2008 Isis Innovation Limited
#include "ptam/KeyFrame.h"
#include "ptam/ShiTomasi.h"
#include "ptam/SmallBlurryImage.h"
#include <cvd/vision.h>
#include <cvd/fast_corner.h>
#include <agast/agast_corner_detect.h>
//{slynen reprojection
#include "ptam/MapPoint.h"
#include "ptam/TrackerData.h"
//}

using namespace CVD;
using namespace std;
//using namespace GVars3;

void KeyFrame::MakeKeyFrame_Lite(BasicImage<CVD::byte> &im)
{
	// Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
	// Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
	// e.g. does not perform FAST nonmax suppression. Things like that which are needed by the
	// mapmaker but not the tracker go in MakeKeyFrame_Rest();

	// adaptive thresholds
	static short thrs[4]={0,0,0,0};
	double buff;

	// First, copy out the image data to the pyramid's zero level.
	aLevels[0].im.resize(im.size());
	copy(im, aLevels[0].im);

	// Then, for each level...
	for(int i=0; i<LEVELS; i++)
	{
		Level &lev = aLevels[i];
		if(i!=0)
		{  // .. make a half-size image from the previous level..
			lev.im.resize(aLevels[i-1].im.size() / 2);
			halfSample(aLevels[i-1].im, lev.im);
		}

		// .. and detect and store FAST corner points.
		// I use a different threshold on each level; this is a bit of a hack
		// whose aim is to balance the different levels' relative feature densities.
		lev.vCorners.clear();
		lev.vCandidates.clear();
		lev.vMaxCorners.clear();

		//Weiss{
		void (*pFASTFunc)(const CVD::BasicImage<CVD::byte> &, std::vector<CVD::ImageRef> &,int)=NULL;
		
		const ptam::PtamParamsConfig& pPars = PtamParameters::varparams();
		pFASTFunc=&fast_corner_detect_9_nonmax;
		if (pPars.FASTMethod=="FAST9")
			pFASTFunc=&fast_corner_detect_9;
		else if (pPars.FASTMethod=="FAST10")
			pFASTFunc=&fast_corner_detect_10;
		else if (pPars.FASTMethod=="FAST9_nonmax")
			pFASTFunc=&fast_corner_detect_9_nonmax;
		else if (pPars.FASTMethod=="AGAST12d")
			pFASTFunc=&agast::agast7_12d::agast_corner_detect12d;
		else if (pPars.FASTMethod=="OAST16")
			pFASTFunc=&agast::oast9_16::oast_corner_detect16;

		if(i == 0)
			pFASTFunc(lev.im, lev.vCorners, pPars.Thres_lvl0+thrs[i]);
		if(i == 1)
			pFASTFunc(lev.im, lev.vCorners, pPars.Thres_lvl1+thrs[i]);
		if(i == 2)
			pFASTFunc(lev.im, lev.vCorners, pPars.Thres_lvl2+thrs[i]);
		if(i == 3)
			pFASTFunc(lev.im, lev.vCorners, pPars.Thres_lvl3+thrs[i]);

		if (pPars.AdaptiveThrs)
		{
			buff = lev.vCorners.size()-pPars.AdaptiveThrsMult*pPars.MaxPatchesPerFrame/pow(2.0,i);
			thrs[i] = thrs[i]+(buff>0)-(buff<0);
	//		printf("0: %d 1: %d 2: %d 3: %d N: %d\n",thrs[0],thrs[1],thrs[2],thrs[3],lev.vCorners.size());
		}
		else
			thrs[i]=0;
//		printf("N: %d",lev.vCorners.size());
		//}

		// Generate row look-up-table for the FAST corner points: this speeds up
		// finding close-by corner points later on.
		unsigned int v=0;
		lev.vCornerRowLUT.clear();
		for(int y=0; y<lev.im.size().y; y++)
		{
			while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
				v++;
			lev.vCornerRowLUT.push_back(v);
		}
	};
}

void KeyFrame::MakeKeyFrame_Rest()
{
  // Fills the rest of the keyframe structure needed by the mapmaker:
  // FAST nonmax suppression, generation of the list of candidates for further map points,
  // creation of the relocaliser's SmallBlurryImage.
  //Weiss{
  //static double gvdCandidateMinSTScore = 70;
  
  const FixParams& pPars = PtamParameters::fixparams();
  const VarParams& pVarPars = PtamParameters::varparams();
  static double gvdCandidateMinSTScore = pPars.CandidateMinSTScore;
  //static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);
  //}


  // For each level...
  int startlvl=0;
  if(pVarPars.NoLevelZeroMapPoints)
    startlvl=1;	// ignore level zero points for the map
  for(int l=startlvl; l<LEVELS; l++)
  {
    Level &lev = aLevels[l];
    // .. find those FAST corners which are maximal..
    fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
    // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
    // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
    // to make new map points out of.
    for(vector<ImageRef>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++)
    {
      if(!lev.im.in_image_with_border(*i, 10))
        continue;
      double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *i);
      if(dSTScore > gvdCandidateMinSTScore)
      {
        Candidate c;
        c.irLevelPos = *i;
        c.dSTScore = dSTScore;
        lev.vCandidates.push_back(c);
      }
    }
  };

  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  pSBI = new SmallBlurryImage(*this);
  // Relocaliser also wants the jacobians..
  pSBI->MakeJacs();

  //Weiss{
  static unsigned int idcounter=0;
  ID=idcounter;
  idcounter++;
  //}
}

//{slynen reprojection
// checks whether to add a MapPoint to the KeyMapPoint of the KeyFrame
// those points help selecting the Keyframes which are visible for reprojection
void KeyFrame::AddKeyMapPoint(MapPoint::Ptr mp){
  vpPoints.push_back(mp);
  //first point init
  if(apCurrentBestPoints[0]==NULL){
    for (unsigned int i = 0;i < iBestPointsCount; i++) apCurrentBestPoints[i] = mp;
    return;
  }
  //apCurrentBestPoints indices:center = 0;upperright = 1;upperleft = 2;lowerleft = 3;lowerright = 4
  CVD::ImageRef imgRef  = aLevels[0].im.size();

  //all point locations in level zero coords
  Vector<2> v2Imcenter = makeVector(imgRef.x / 2,imgRef.y / 2);
  Vector<2> v2PointPos = LevelZeroPos(mp->irCenter,0);
  Vector<2> v2Diff = v2PointPos - v2Imcenter;	//distance to center of level zero
  Vector<2> v2DiffBest; //best distance for this point

  v2DiffBest = LevelZeroPos(apCurrentBestPoints[0]->irCenter,apCurrentBestPoints[0]->nSourceLevel) - v2Imcenter;
  if((v2Diff*v2Diff) < (v2DiffBest*v2DiffBest))
    apCurrentBestPoints[0] = mp;	//the point is closer to the center that current best center point

  //now check which quadrant the point is in
  if(v2PointPos[0] > v2Imcenter[0] &&  v2PointPos[1] > v2Imcenter[1]){
    v2DiffBest = LevelZeroPos(apCurrentBestPoints[1]->irCenter,apCurrentBestPoints[1]->nSourceLevel) - v2Imcenter;
    if((v2Diff*v2Diff) > (v2DiffBest*v2DiffBest))
      apCurrentBestPoints[1] = mp; //further away than current best point
  }else if(v2PointPos[0]<v2Imcenter[0] &&  v2PointPos[1] > v2Imcenter[1]){
    v2DiffBest = LevelZeroPos(apCurrentBestPoints[2]->irCenter,apCurrentBestPoints[2]->nSourceLevel) - v2Imcenter;
    if((v2Diff*v2Diff) > (v2DiffBest*v2DiffBest))
      apCurrentBestPoints[2] = mp; //further away than current best point
  }else if(v2PointPos[0]<v2Imcenter[0] &&  v2PointPos[1] < v2Imcenter[1]){
    v2DiffBest = LevelZeroPos(apCurrentBestPoints[3]->irCenter,apCurrentBestPoints[3]->nSourceLevel) - v2Imcenter;
    if((v2Diff*v2Diff) > (v2DiffBest*v2DiffBest))
      apCurrentBestPoints[3] = mp; //further away than current best point
  }else if(v2PointPos[0]>v2Imcenter[0] &&  v2PointPos[1] < v2Imcenter[1]){
    v2DiffBest = LevelZeroPos(apCurrentBestPoints[4]->irCenter,apCurrentBestPoints[4]->nSourceLevel) - v2Imcenter;
    if((v2Diff*v2Diff) > (v2DiffBest*v2DiffBest))
      apCurrentBestPoints[4] = mp; //further away than current best point
  }
}
//}

// The keyframe struct is quite happy with default operator=, but Level needs its own
// to override CVD's reference-counting behaviour.
Level& Level::operator=(const Level &rhs)
{
  // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
  im.resize(rhs.im.size());
  copy(rhs.im, im);

  vCorners = rhs.vCorners;
  vMaxCorners = rhs.vMaxCorners;
  vCornerRowLUT = rhs.vCornerRowLUT;
  return *this;
}

// -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
Vector<3> gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller // Code which should be initialised on init goes here; this runs before main()
{
  LevelHelpersFiller()
  {
    for(int i=0; i<LEVELS; i++)
    {
      if(i==0)  gavLevelColors[i] = makeVector( 1.0, 0.0, 0.0);
      else if(i==1)  gavLevelColors[i] = makeVector( 1.0, 1.0, 0.0);
      else if(i==2)  gavLevelColors[i] = makeVector( 0.0, 1.0, 0.0);
      else if(i==3)  gavLevelColors[i] = makeVector( 0.0, 0.0, 0.7);
      else gavLevelColors[i] =  makeVector( 1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
    }
  }
};
static LevelHelpersFiller foo;







