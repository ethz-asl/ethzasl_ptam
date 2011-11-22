// Copyright 2008 Isis Innovation Limited
#include "ptam/System.h"
#include "ptam/OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ptam/ATANCamera.h"
#include "ptam/MapMaker.h"
#include "ptam/Tracker.h"
#include "ptam/ARDriver.h"
#include "ptam/MapViewer.h"

using namespace CVD;
using namespace std;
using namespace GVars3;


#ifndef NO_GUI
System::System()
: mVideoSource(), mPublisher(), mGLWindow(mVideoSource.Size(), "PTAM")
#else
System::System()
: mVideoSource(), mPublisher()
#endif
  {

	GUI.RegisterCommand("exit", GUICommandCallBack, this);
	GUI.RegisterCommand("quit", GUICommandCallBack, this);

	mimFrameBW.resize(mVideoSource.Size());
	mimFrameRGB.resize(mVideoSource.Size());
	// First, check if the camera is calibrated.
	// If not, we need to run the calibration widget.
	Vector<NUMTRACKERCAMPARAMETERS> vTest;

//Weiss{
	vTest = ATANCamera::mvDefaultParams;
	FixParams* pPars = ParamsAccess::fixParams;
	vTest = makeVector(pPars->Cam_fx, pPars->Cam_fy, pPars->Cam_cx, pPars->Cam_cy, pPars->Cam_s);
	//vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
//}
	mpCamera = new ATANCamera("Camera");

	if(vTest == ATANCamera::mvDefaultParams)
	{
		cout << endl;
		cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
		cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
		exit(1);
	}

	mpMap = new Map;
	mpMapMaker = new MapMaker(*mpMap, *mpCamera);
	mpTracker = new Tracker(mVideoSource.Size(), *mpCamera, *mpMap, *mpMapMaker);

#ifndef NO_GUI
	mpARDriver = new ARDriver(*mpCamera, mVideoSource.Size(), mGLWindow);
	mpMapViewer = new MapViewer(*mpMap, mGLWindow);

	GUI.ParseLine("GLWindow.AddMenu Menu Menu");
	GUI.ParseLine("Menu.ShowMenu Root");
	GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
	GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
	GUI.ParseLine("DrawAR=0");
	GUI.ParseLine("DrawMap=0");
	GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
	GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");
#endif

	mbDone = false;
  };

void System::Run()
{
	double timestamp = 0;
	std::string frameId;
	while(!mbDone && mVideoSource.running())
	{

		// We use two versions of each video frame:
		// One black and white (for processing by the tracker etc)
		// and one RGB, for drawing.

		// Grab new video frame...
		if(!mVideoSource.GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB)){
			usleep(1000);
			continue;
		}

		mVideoSource.getStampAndFrameId(timestamp, frameId);

#ifndef NO_GUI
		static bool bFirstFrame = true;
		if(bFirstFrame)
		{
			mpARDriver->Init();
			bFirstFrame = false;
		}

		mGLWindow.SetupViewport();
		mGLWindow.SetupVideoOrtho();
		mGLWindow.SetupVideoRasterPosAndZoom();


		if(!mpMap->IsGood())
			mpARDriver->Reset();

		static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
		static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN|SILENT);

		bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
		bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;



		mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap);


		if(bDrawMap)
			mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
		else if(bDrawAR)
			mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose());
#else

		mpTracker->TrackFrame(mimFrameBW, false);

#endif


		mPublisher.publishPose(mpTracker->GetCurrentPose(), mpTracker->GetCurrentCov(), mpTracker->getTrackingQuality(), mpMap->IsGood(), timestamp, frameId);
		mPublisher.publishPtamInfo(
				mpTracker->getTrackingQuality(),
				mpMap->IsGood(),
//				mpMapViewer->GetMessageForUser(),
				"",
				mpTracker->GetMessageForUser(),
				mpMap->vpKeyFrames.size(),
				timestamp,
				frameId
				);

		if(mRemoteInterface.newCommand()){
			mpTracker->command(mRemoteInterface.getCommand());
		}

		if(mpTracker->getTrailTrackingStarted()){
			mVisualization.publishTrails(mpTracker->getTrails());
		}

		mRemoteInterface.publishPreview(
				mimFrameBW,
				mpTracker->ComputeGrid(),
				mpTracker->getTrails(),
				mpTracker->getTrailTrackingComplete(),
				mpTracker->getTrailTrackingStarted()
				);

		//      mGLWindow.GetMousePoseUpdate();

		std::cout<<mpMapMaker->getMessageForUser();

#ifndef NO_GUI
		string sCaption;
		if(bDrawMap)
			sCaption = mpMapViewer->GetMessageForUser();
		else
			sCaption = mpTracker->GetMessageForUser();
		mGLWindow.DrawCaption(sCaption);
		mGLWindow.DrawMenus();
		mGLWindow.swap_buffers();
		mGLWindow.HandlePendingEvents();
#endif
	}
}

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
	if(sCommand=="quit" || sCommand == "exit")
		static_cast<System*>(ptr)->mbDone = true;
}








