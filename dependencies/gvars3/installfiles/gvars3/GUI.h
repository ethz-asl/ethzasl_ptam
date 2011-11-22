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

#ifndef __GUI_H
#define __GUI_H

#include <gvars3/gvars3.h>
#include <vector>
#include <iostream>
#include <set>

namespace GVars3
{
	typedef void (*GUICallbackProc)(void* ptr, std::string sCommand, std::string sParams);

	typedef struct 
	{
	  GUICallbackProc cbp;
	  void* thisptr;
	} CallbackInfoStruct;

	typedef std::vector<CallbackInfoStruct> CallbackVector;


	
	class GUI_impl;

	class GUI
	{
		public:
			static GUI_impl& I();

			GUI();
			GUI(GVars2*);//Dummy gvars2

			void RegisterCommand(std::string sCommandName, GUICallbackProc callback, void* thisptr=NULL);
			void UnRegisterAllCommands(void* thisptr);
			void UnRegisterCommand(std::string sCommandName, void* thisptr);
			void UnRegisterCommand(std::string sCommandName);
			void ParseLine(std::string s, bool bSilentFailure = false);
			void ParseStream(std::istream& is);
			void LoadFile(std::string sFileName);

			bool CallCallbacks(std::string sCommand, std::string sParams);
			void SetupReadlineCompletion();

			/// Start a thread which parses user input from the console.
			/// Uses libreadline if configured, or just plain old iostream readline
			void StartParserThread();
			/// Stop the console parser thread, if running
			/// Top tip: This is static so that it can be used with atexit(void*)
			static void StopParserThread();

			/// parse command line arguments for GVar values. It expects the form --name value and will stop
			/// parsing when this form is not true anymore. possible cases are a single --, an argument, etc..
			/// if it finds an argument --exec it interprets the next argument as a file name to load via LoadFile
			/// @arg argc total number of arguments from main argc
			/// @arg argv pointer to array of strings from main argv
			/// @arg start first argument to look at to be compatible with other parsers
			/// @arg prefix the prefix to use. note that everything after the prefix will be interpreted as part of the variable name
			/// @arg execKeyword keyword to use to trigger execution of the file given as parameter with LoadFile
			/// @return
			int parseArguments( const int argc, char * argv[], int start = 1, const std::string prefix = "--", const std::string execKeyword = "exec" );
	};

}




#endif
