// Copyright 2008 Isis Innovation Limited
// This is the main extry point for PTAM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "ptam/System.h"
#include <ptam/Params.h>
//#include "ptam/RosNode.h"

using namespace std;
using namespace GVars3;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ptam");
	ROS_INFO("starting ptam with node name %s", ros::this_node::getName().c_str());

	cout << "  Welcome to PTAM " << endl;
	cout << "  --------------- " << endl;
	cout << "  Parallel tracking and mapping for Small AR workspaces" << endl;
	cout << "  Copyright (C) Isis Innovation Limited 2008 " << endl;
	cout << endl;

	GUI.StartParserThread(); // Start parsing of the console input
	atexit(GUI.StopParserThread);

	try
	{
		PtamParameters mPtamParameters;
		System s;
		s.Run();
		// ugly, but should prevent from asking for a keyboard input
		abort();
	}
	catch(CVD::Exceptions::All e)
	{
		cout << endl;
		cout << "!! Failed to run system; got exception. " << endl;
		cout << "   Exception was: " << endl;
		cout << e.what << endl;
	}
}










