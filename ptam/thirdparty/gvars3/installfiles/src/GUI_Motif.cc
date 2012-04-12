/*                       
	This file is part of the GVars3 Library.

	Copyright (C) 2005 The Authors

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 
    51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "gvars3/instances.h"
#include "gvars3/GUI_Motif.h"
#include "gvars3/GStringUtil.h"
#include <Xm/PushB.h>
#include <Xm/RowColumn.h>
#include <Xm/FileSB.h>
#include <Xm/Label.h>
#include <Xm/ScrollBar.h>
#include <Xm/ToggleB.h>
#include <Xm/Protocols.h>
#include <Xm/AtomMgr.h>
#include <pthread.h>
#include <unistd.h>
#include <sstream>

using namespace std;
namespace GVars3
{

GUI_Motif::GUI_Motif(class GUI *pGUI, GVars2 *pGV2)
{
	mpGUI=pGUI;
	mpGV2=pGV2;
	mpDisplay=NULL;
	mpGUI->RegisterCommand("GUI.InitXInterface", InitXInterfaceCB, this);
	mpGUI->RegisterCommand("GUI_Motif.InitXInterface", InitXInterfaceCB, this);
}


void GUI_Motif::InitXInterfaceCB(void* ptr, string sCommand, string sParams)
{
  ((GUI_Motif*)(ptr))->InitXInterface(sParams);
}

void GUI_Motif::AddWindowCB(void* ptr, string sCommand, string sParams)
{
  ((GUI_Motif*)(ptr))->AddWindow(sParams);
}



void GUI_Motif::AddWindow(string sParams)
{
  if(!mpDisplay)
    {
      cout << "! GUI_Motif::AddWindow: No display. Call InitXInterface first." << endl;
      return;
    }
  
  vector<string> vs = ChopAndUnquoteString(sParams);
  if(((vs.size()<1) || (vs.size()>3)) )
    {
      cout << "! GUI_Motif::AddWindow: need 1 - 3 params: Name, Caption=Name, Width=200 " << endl;
      return;
    };
  
  if(mmWindows.count(vs[0])>0)
  {
    cout << "? GUI_Motif::AddWindow: A window with id \"" << vs[0] << "\" already exists." << endl;
    return;
  }
  
  string sCaption;
  if(vs.size()>=2)
    sCaption=vs[1];
  else
    sCaption=vs[0];

  
  GUIWindowStruct gws;
  gws.PushButtonMap.clear();
  gws.MonitorMap.clear();
  gws.ToggleButtonMap.clear();
  gws.SliderMap.clear();
  gws.nWidth = 200;
  if(vs.size()>2)
    {
      int *pn = ParseAndAllocate<int>(vs[2]);
      if(pn)
	{
	  gws.nWidth=*pn;
	  delete pn;
	};      
    }
  gws.nHeight = 30;
  
  pthread_mutex_lock((pthread_mutex_t*)mpMutex);
  
  int argc=1;
  char* argv[] = {""};
  Arg al[10];
  int ac = 0;
  gws.wTopLevel=XtAppCreateShell(sCaption.c_str(), sCaption.c_str(), applicationShellWidgetClass, mpDisplay, al, ac);

  ac=0;
  XtSetArg(al[ac], XmNpacking, XmPACK_TIGHT);ac++;
  XtSetArg(al[ac], XmNorientation, XmVERTICAL);ac++;
  XtSetArg(al[ac], XmNadjustLast, False);ac++;
  XtSetArg(al[ac], XmNresizeHeight, True); ac++;
  XtSetArg(al[ac], XmNresizeWidth, False); ac++;
  
  gws.wRowCol=XmCreateRowColumn(gws.wTopLevel,"GUI_Motif_rowcol",al,ac);
  
  XtManageChild(gws.wRowCol);
  XtRealizeWidget(gws.wTopLevel);
  
  XEvent xevent;
  XtAppNextEvent(mxtac,&xevent);
  XtDispatchEvent(&xevent);
  DoMotifEvents();
  XResizeWindow(mpDisplay, XtWindow(gws.wTopLevel), gws.nWidth, gws.nHeight);
  XResizeWindow(mpDisplay, XtWindow(gws.wRowCol), gws.nWidth, gws.nHeight);
  DoMotifEvents();
  
  
  Atom delwinAtom1;
  delwinAtom1 = XmInternAtom (mpDisplay,"WM_DELETE_WINDOW", False);
  XmAddWMProtocolCallback (gws.wTopLevel, delwinAtom1, RemoveWindowCB, this);
  XtVaSetValues(gws.wTopLevel, XmNdeleteResponse, XmDO_NOTHING, NULL);
  
  mmWindows[vs[0]]=gws;

  mpGUI->RegisterCommand(vs[0] + ".AddPushButton", AddPushButtonCB, this);
  mpGUI->RegisterCommand(vs[0] + ".AddToggleButton", AddToggleButtonCB, this);
  mpGUI->RegisterCommand(vs[0] + ".AddMonitor", AddMonitorCB, this);
  mpGUI->RegisterCommand(vs[0] + ".AddSlider", AddSliderCB, this);
  mpGUI->RegisterCommand(vs[0] + ".Destroy", DestroyCB, this);
  
  pthread_mutex_unlock((pthread_mutex_t*)mpMutex);

  
};


void GUI_Motif::InitXInterface(string sParams)
{
	vector<string> vs = ChopAndUnquoteString(sParams);
	string sDisplay="";
	if(vs.size()>0)
		sDisplay=vs[0];
	if(vs.size()>1)
		msName = vs[1];
	else
		msName = "GUI";


	if(mpDisplay)
	{
		cout << "??GUI_Motif::InitXInterface: display already initialised.." << endl;
		return;
	}

	if(sDisplay=="")
		mpDisplay=XOpenDisplay(NULL);
	else
		mpDisplay=XOpenDisplay(sDisplay.c_str());

	if(!mpDisplay)
	{
		cout << "??GUI_Motif::InitXInterface: Could not open display \""<<sDisplay<< "\"." << endl;
		return;
	}


	// Ok, set up some widgets!
	int argc=1;
	char* argv[] = {""};
	Arg al[10];
	int ac = 0;

	XtToolkitInitialize();
	mxtac=XtCreateApplicationContext();
	XtDisplayInitialize(mxtac, mpDisplay, "GUI_Motif", "GUI_Motif", NULL, 0, &argc, argv);


	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	mpMutex = new(pthread_mutex_t);
	pthread_mutex_init(mpMutex,  &attr);
	mpGUI->RegisterCommand(msName+".AddWindow", AddWindowCB, this);

}

void GUI_Motif::start_thread()
{
  pthread_t t;
  pthread_create(&t, NULL, GUI_Motif_Thread_CB, this);
}

void GUI_Motif::RemoveWindow(Widget w)
{
	for(map<string,GUIWindowStruct>::iterator i = mmWindows.begin(); i!=mmWindows.end(); i++)
		if (i->second.wTopLevel == w)
		{
			DestroyWindow(i->first + ".Destroy");
			return;
		};
}


void GUI_Motif::RemoveWindowCB(Widget w, void* client_data, void* call_data)
{
  ((GUI_Motif *) client_data)->RemoveWindow(w);
} 

void GUI_Motif::DestroyCB(void* ptr, string sCommand, string sParams)
{
  ((GUI_Motif*)(ptr))->DestroyWindow(sCommand);
} 

void GUI_Motif::DestroyWindow(string sCommand)
{
  string sWindowName = sCommand;
  string sCommandSuffix(".Destroy");
  sWindowName.resize(sCommand.length() - sCommandSuffix.length());
  mpGUI->UnRegisterCommand(sWindowName + ".AddPushButton");
  mpGUI->UnRegisterCommand(sWindowName + ".AddToggleButton");
  mpGUI->UnRegisterCommand(sWindowName + ".AddMonitor");
  mpGUI->UnRegisterCommand(sWindowName + ".Destroy");
  XtUnrealizeWidget(mmWindows[sWindowName].wTopLevel);
  mmWindows.erase(sWindowName);
  DoMotifEvents();
  
};


void GUI_Motif::AddPushButtonCB(void* ptr, string sCommand, string sParams)
{
  ((GUI_Motif*)(ptr))->AddPushButton(sCommand, sParams);
}


void GUI_Motif::AddPushButton(string sCommand, string sParams)
{
  
  vector<string> vs = ChopAndUnquoteString(sParams);
  if(vs.size()!=2) 
    {
      cout << "! GUI_Motif::AddPushButton: Need 2 params (name, command)." << endl;
      return;
    };
  

  Widget *pwTopLevel;
  Widget *pwRowCol;

  string sWindowName = sCommand;
  string sCommandSuffix(".AddPushButton");
  sWindowName.resize(sCommand.length() - sCommandSuffix.length());
  Widget  w;
  
  Arg al[10];
  int ac = 0;
  char szName[1000];
  for(int i=0;i<1000;i++) szName[i]=0;
  vs[0].copy(szName,999);
  
  for(map<Widget,string>::iterator i = mmWindows[sWindowName].PushButtonMap.begin();
      i!=mmWindows[sWindowName].PushButtonMap.end();
      i++)
    if(i->second == vs[1])
      {
	cout << "? GUI_Motif::AddPushButton: Button with command \"" << vs[1] << "\" already there.. skipping. " << endl;
	return;
      }
  
  pthread_mutex_lock((pthread_mutex_t*)mpMutex);
  
  
  w = XmCreatePushButton(mmWindows[sWindowName].wRowCol, szName,al,ac);
  XtManageChild(w);
  mmWindows[sWindowName].PushButtonMap[w] = vs[1];
  XtAddCallback(w, XmNactivateCallback, ButtonHandlerCB, this);
  
  XWindowAttributes window_attributes_return;
  XGetWindowAttributes(mpDisplay, XtWindow(w), &window_attributes_return);
  int nButtonHeight = window_attributes_return.height;
  XGetWindowAttributes(mpDisplay, XtWindow(mmWindows[sWindowName].wTopLevel), &window_attributes_return);
  mmWindows[sWindowName].nWidth = (mmWindows[sWindowName].nWidth > window_attributes_return.width) ? mmWindows[sWindowName].nWidth : window_attributes_return.width;
  mmWindows[sWindowName].nHeight = (mmWindows[sWindowName].nHeight > window_attributes_return.height) ? mmWindows[sWindowName].nHeight : window_attributes_return.height;
  mmWindows[sWindowName].nHeight += (nButtonHeight + 3);
  XResizeWindow(mpDisplay, XtWindow(mmWindows[sWindowName].wTopLevel), 
		mmWindows[sWindowName].nWidth,
		mmWindows[sWindowName].nHeight);
  DoMotifEvents();
  pthread_mutex_unlock((pthread_mutex_t*)mpMutex);
  
}

void GUI_Motif::AddToggleButtonCB(void* ptr, string sCommand, string sParams)
{
  ((GUI_Motif*)(ptr))->AddToggleButton(sCommand, sParams);
}


void GUI_Motif::AddToggleButton(string sCommand, string sParams)
{
  vector<string> vs = ChopAndUnquoteString(sParams);
  if(!((vs.size() == 2) || (vs.size()==3)))
    {
      cout << "! GUI_Motif::AddToggleButton: Need 2 or 3 params (button name, gvar2_int name, {true,false}=false )." << endl;
      return;
    };
  string sWindowName = sCommand;
  string sCommandSuffix(".AddToggleButton");
  sWindowName.resize(sCommand.length() - sCommandSuffix.length());

  for(map<Widget,toggleMapStruct>::iterator i = mmWindows[sWindowName].ToggleButtonMap.begin();
      i!=mmWindows[sWindowName].ToggleButtonMap.end();
      i++)
    if(i->second.sName == vs[0])
      {
	cout << "? GUI_Motif::AddToggleButton: Button with name \"" << vs[0] << "\" already there.. skipping. " << endl;
	return;
      }
  
  toggleMapStruct tms;
  tms.sName = vs[0];
  int nDefault = 0;
  if(vs.size()==3)
    if(vs[2]=="true")
      nDefault = 1;
    else
      nDefault = 0;

  mpGV2->Register(tms.gvn,vs[1],nDefault,true);
  tms.nCache = *tms.gvn;
  
 
  pthread_mutex_lock((pthread_mutex_t*)mpMutex);
  
  Widget w;
  Arg al[10];
  int ac = 0;
  char szName[1000];
  for(int i=0;i<1000;i++) szName[i]=0;
  vs[0].copy(szName,999);

  w = XmCreateToggleButton(mmWindows[sWindowName].wRowCol, szName,al,ac);
  XtManageChild(w);
  mmWindows[sWindowName].ToggleButtonMap[w] = tms;
  XtAddCallback(w, XmNvalueChangedCallback, ButtonHandlerCB, this);
  if(tms.nCache) XmToggleButtonSetState(w,true,true);
  
  XWindowAttributes window_attributes_return;
  XGetWindowAttributes(mpDisplay, XtWindow(w), &window_attributes_return);
  int nButtonHeight = window_attributes_return.height;
  XGetWindowAttributes(mpDisplay, XtWindow(mmWindows[sWindowName].wTopLevel), &window_attributes_return);
  mmWindows[sWindowName].nWidth = (mmWindows[sWindowName].nWidth > window_attributes_return.width) ? mmWindows[sWindowName].nWidth : window_attributes_return.width;
  mmWindows[sWindowName].nHeight = (mmWindows[sWindowName].nHeight > window_attributes_return.height) ? mmWindows[sWindowName].nHeight : window_attributes_return.height;
  mmWindows[sWindowName].nHeight += (nButtonHeight + 3);
  XResizeWindow(mpDisplay, XtWindow(mmWindows[sWindowName].wTopLevel), 
		mmWindows[sWindowName].nWidth,
		mmWindows[sWindowName].nHeight);
  DoMotifEvents();
  pthread_mutex_unlock((pthread_mutex_t*)mpMutex);
  
}


void GUI_Motif::AddMonitorCB(void* ptr, string sCommand, string sParams)
{
  ((GUI_Motif*)(ptr))->AddMonitor(sCommand, sParams);
}


void GUI_Motif::AddMonitor(string sCommand, string sParams)
{
  if(!mpDisplay)
    {
      cout << "! GUI_Motif::AddMonitor: GUI_Motif not initialised. " << endl;
      return;
    }
  vector<string> vs = ChopAndUnquoteString(sParams);
  if((vs.size() <2) || (vs.size() > 3))
    {
      cout << "! GUI_Motif::AddMonitor: Need 2 or 3 params: label, gvar name, num 20ms cycles to skip = 0 " << endl;
      return;
    };
  string sWindowName = sCommand;
  string sCommandSuffix(".AddMonitor");
  sWindowName.resize(sCommand.length() - sCommandSuffix.length());

  for(map<Widget,monitorMapStruct>::iterator i = mmWindows[sWindowName].MonitorMap.begin();
      i!=mmWindows[sWindowName].MonitorMap.end();
      i++)
    if(i->second.sLabel == vs[0])
      {
	cout << "? GUI_Motif::AddMontiro: Montor with Label \"" << vs[0] << "\" already there.. skipping. " << endl;
	return;
      }
  
  
  pthread_mutex_lock((pthread_mutex_t*)mpMutex);
  
  Widget w;
  Arg al[10];
  int ac = 0;
  w = XmCreateLabel(mmWindows[sWindowName].wRowCol,"(wait)",al,ac);
  XtManageChild(w);
  monitorMapStruct &mms =  mmWindows[sWindowName].MonitorMap[w];
  mms.sLabel = vs[0];
  mms.sVarName = vs[1];
  if(vs.size()==3)
    {
      int* pn = ParseAndAllocate<int>(vs[2]);
      if (pn)
	{
	  mms.nDelaySetting = *pn;
	  delete pn;
	}
      else
	mms.nDelaySetting = 0;
    }
  else
    mms.nDelaySetting = 0;
  mms.nCurrentDelay=0;

  XWindowAttributes window_attributes_return;
  XGetWindowAttributes(mpDisplay, XtWindow(w), &window_attributes_return);
  int nButtonHeight = window_attributes_return.height;
  XGetWindowAttributes(mpDisplay, XtWindow(mmWindows[sWindowName].wTopLevel), &window_attributes_return);
  mmWindows[sWindowName].nWidth = (mmWindows[sWindowName].nWidth > window_attributes_return.width) ? mmWindows[sWindowName].nWidth : window_attributes_return.width;
  mmWindows[sWindowName].nHeight = (mmWindows[sWindowName].nHeight > window_attributes_return.height) ? mmWindows[sWindowName].nHeight : window_attributes_return.height;
  mmWindows[sWindowName].nHeight += (nButtonHeight + 3);
  XResizeWindow(mpDisplay, XtWindow(mmWindows[sWindowName].wTopLevel), 
		mmWindows[sWindowName].nWidth,
		mmWindows[sWindowName].nHeight);
  DoMotifEvents();
  pthread_mutex_unlock((pthread_mutex_t*)mpMutex);
  
}


void GUI_Motif::AddSliderCB(void* ptr, string sCommand, string sParams)
{
  ((GUI_Motif*)(ptr))->AddSlider(sCommand, sParams);
}


void GUI_Motif::AddSlider(string sCommand, string sParams)
{
  if(!mpDisplay)
    {
      cout << "! GUI_Motif::AddSlider: GUI_Motif not initialised. " << endl;
      return;
    }
  vector<string> vs = ChopAndUnquoteString(sParams);
  if((vs.size() != 3))
    {
      cout << "! GUI_Motif::AddSlider: 3 params: gvar name, min, max " << endl;
      return;
    };
  string sWindowName = sCommand;
  string sCommandSuffix(".AddSlider");
  sWindowName.resize(sCommand.length() - sCommandSuffix.length());
  for(map<Widget,sliderMapStruct>::iterator i = mmWindows[sWindowName].SliderMap.begin();
      i!=mmWindows[sWindowName].SliderMap.end();
      i++)
    if(i->second.sVarName == vs[0])
      {
	cout << "? GUI_Motif::AddSlider: Slider with gvar \"" << vs[0] << "\" already there.. skipping. " << endl;
	return;
      }

  sliderMapStruct sms;
  sms.sVarName = vs[0];  
  double *pd;
  pd = ParseAndAllocate<double>(vs[1]);
  if(!pd)
    {
      cout << "? GUI_Motif::AddSlider: Could not parse min range parameter \"" << vs[1] << "\"."<< endl;
      return;
    }
  sms.dMin = *pd;
  delete pd;
  pd=ParseAndAllocate<double>(vs[2]);
  if(!pd)
    {
      cout << "? GUI_Motif::AddSlider: Could not parse max range parameter \"" << vs[2] << "\"."<< endl;
      return;
    } 
  sms.dMax=*pd;
  delete pd;
  if(sms.dMax == sms.dMin)
    {
      cout << "? GUI_Motif::AddSlider: Cannot have max = min! " << endl;
      return;
    }

  
  
  pthread_mutex_lock((pthread_mutex_t*)mpMutex);
  
  Widget w;
  Arg al[10];
  int ac = 0;

  XtSetArg(al[ac], XmNorientation, XmHORIZONTAL);  ac++;  
  //  XtSetArg(al[ac], XmNsliderMark, XmETCHED_LINE);  ac++;  
  XtSetArg(al[ac], XmNwidth, mmWindows[sWindowName].nWidth - 5);  ac++;  
  XtSetArg(al[ac], XmNmaximum, 310);  ac++;  
  XtSetArg(al[ac], XmNsliderSize, 10);  ac++;  
  
  w = XmCreateScrollBar(mmWindows[sWindowName].wRowCol,"SliderName",al,ac);
  XtManageChild(w);
  XtAddCallback(w, XmNdragCallback, ButtonHandlerCB, this);
  XtAddCallback(w, XmNvalueChangedCallback, ButtonHandlerCB, this);

  mmWindows[sWindowName].SliderMap[w] = sms;
  
  //   if(vs.size()==3)
//     {
//       int* pn = ParseAndAllocate<int>(vs[2]);
//       if (pn)
// 	{
// 	  mms.nDelaySetting = *pn;
// 	  delete pn;
// 	}
//       else
// 	mms.nDelaySetting = 0;
//     }
//   else
//     mms.nDelaySetting = 0;
//   mms.nCurrentDelay=0;
  
  XWindowAttributes window_attributes_return;
  XGetWindowAttributes(mpDisplay, XtWindow(w), &window_attributes_return);
  int nButtonHeight = window_attributes_return.height;
  XGetWindowAttributes(mpDisplay, XtWindow(mmWindows[sWindowName].wTopLevel), &window_attributes_return);
  mmWindows[sWindowName].nWidth = (mmWindows[sWindowName].nWidth > window_attributes_return.width) ? mmWindows[sWindowName].nWidth : window_attributes_return.width;
  mmWindows[sWindowName].nHeight = (mmWindows[sWindowName].nHeight > window_attributes_return.height) ? mmWindows[sWindowName].nHeight : window_attributes_return.height;
  mmWindows[sWindowName].nHeight += (nButtonHeight + 3);
  XResizeWindow(mpDisplay, XtWindow(mmWindows[sWindowName].wTopLevel), 
		mmWindows[sWindowName].nWidth,
		mmWindows[sWindowName].nHeight);
  DoMotifEvents();
  pthread_mutex_unlock((pthread_mutex_t*)mpMutex);
  
}



// Callback function for MOTIF events, wrapper for button_F

void GUI_Motif::ButtonHandlerCB(Widget w, XtPointer ptrMe, XtPointer xtpCall) 
{
  ((GUI_Motif*)ptrMe) -> ButtonHandler(w, xtpCall);
}


// Handles the actual callback work
void GUI_Motif::ButtonHandler(Widget w, XtPointer xtpCall)
{
  
  for(map<string, GUIWindowStruct>::iterator i=mmWindows.begin();i!=mmWindows.end();i++)
    {
      if(i->second.PushButtonMap.count(w)>0)
	{
	  mpGUI->ParseLine(i->second.PushButtonMap[w]);
	}
      else if(i->second.ToggleButtonMap.count(w)>0)
	{
	  *i->second.ToggleButtonMap[w].gvn = i->second.ToggleButtonMap[w].nCache = ((XmToggleButtonCallbackStruct*) xtpCall)->set;
	}
      else if(i->second.SliderMap.count(w)>0)
	{
	  sliderMapStruct &sms =  i->second.SliderMap[w];
	  double dRawValue = (double) (((XmScrollBarCallbackStruct*)xtpCall)->value);
	  ostringstream ost;
	  ost.str()="";
	  ost << (dRawValue / 300.0) * (sms.dMax-sms.dMin) + sms.dMin;
	  mpGV2->SetVar(sms.sVarName, ost.str(), 1);
	  sms.sCachedValue = mpGV2->StringValue(sms.sVarName);
	};
      //
    };
  
}

void GUI_Motif::GUI_Motif_Thread()
{
	while(mpDisplay)
	{
		poll();
		usleep(20000);
	}
}

void GUI_Motif::poll()
{
	if(!mpDisplay)
		return;
  static char szString[1000];

  pthread_mutex_lock(mpMutex);
  for(map<string,GUIWindowStruct>::iterator w=mmWindows.begin();w!=mmWindows.end();w++)
    {
      for(map< Widget, toggleMapStruct >::iterator i = w->second.ToggleButtonMap.begin(); i!=w->second.ToggleButtonMap.end(); i++)
	{
	  if(*(i->second.gvn)!=(i->second.nCache))
	    {
	      i->second.nCache = *(i->second.gvn);
	      if(i->second.nCache)
		XmToggleButtonSetState(i->first,true,false);
	      else
		XmToggleButtonSetState(i->first,false,false);
	    };
	}
      
      for(map < Widget, monitorMapStruct>::iterator i = w->second.MonitorMap.begin(); i!=w->second.MonitorMap.end(); i++)
	{
	  if( (i->second.nCurrentDelay--) >0)
	    continue;
	  i->second.nCurrentDelay=i->second.nDelaySetting;	  
	  ostringstream os;
	  os.str()="";
	  os << i->second.sLabel << ":" << mpGV2->StringValue(i->second.sVarName);
	  if(os.str() == i->second.sCache)
	    continue;
	  i->second.sCache=os.str();
	  strncpy(szString,os.str().c_str(),999);
	  Arg al;
	  XmString xmstr;
	  xmstr = XmStringCreate(szString, XmFONTLIST_DEFAULT_TAG);
	  XtSetArg (al, XmNlabelString, xmstr);
	  XtSetValues(i->first, &al, 1);
	  XmStringFree(xmstr);
	};

      for(map < Widget, sliderMapStruct>::iterator i = w->second.SliderMap.begin(); i!=w->second.SliderMap.end(); i++)
	{
	  sliderMapStruct &sms = i->second;
	  string sNewValue = mpGV2->StringValue(sms.sVarName);
	  if(sNewValue == sms.sCachedValue)
	    continue;
	  sms.sCachedValue = sNewValue;
	  double *pdNewValue = ParseAndAllocate<double>(sNewValue);
	  if(!pdNewValue) continue;
	  double dFraction = ((*pdNewValue - sms.dMin)/ (sms.dMax-sms.dMin));
	  if(dFraction > 1.0)
	    sms.dMax = *pdNewValue;
	  if(dFraction < 0.0)
	    sms.dMin = *pdNewValue;
	  XmScrollBarSetValues(i->first, 
			       (int)(((*pdNewValue - sms.dMin)/ (sms.dMax-sms.dMin))*300.0),
			       0,0,0,False);	      
	  delete pdNewValue;
	};
      
    };
  DoMotifEvents();


} 
void* GUI_Motif::GUI_Motif_Thread_CB(void* ptr)
{
  ((GUI_Motif*) ptr)->GUI_Motif_Thread();
}

void GUI_Motif::DoMotifEvents()
{
  XEvent xevent;
  int num_events=0;
  while(XtAppPending(mxtac) && num_events < 100000){
    XtAppNextEvent(mxtac,&xevent);
    XtDispatchEvent(&xevent);
    num_events++;
  }
}

//Instantiations, link time virtual function definitions 
class GUI_Motif GUI_Motif_instance(&GUI, &GV2);

void GUIWidgets::process_in_crnt_thread()
{
	GUI_Motif_instance.poll();
}

void GUIWidgets::start_thread()
{
	GUI_Motif_instance.start_thread();
}

}
