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
#include "gvars3/GUI.h"
#include "gvars3/GUI_non_readline.h"
#include <iostream>
#include <pthread.h>

using namespace std;

namespace GVars3
{

  bool spawn_non_readline_thread::running=0;
  bool spawn_non_readline_thread::quit=0;
  std::string spawn_non_readline_thread::quit_callback;
  
  spawn_non_readline_thread::spawn_non_readline_thread(const std::string& cb)
  {
    if(!running)
      {
	pthread_create(&cmd, 0, proc, 0);
	running = 1;
	quit=0;
	none=0;
	quit_callback = cb;
      }
    else
      {
	none =1;
	std::cerr << "Error: non_readline thread already running.\n";
      }
  }


  void* spawn_non_readline_thread::proc(void *)
  {
    char line[1000];
    GUI.SetupReadlineCompletion();


    while(1)
      {
	cout << "> ";  

    	cin.clear();
	cin.getline (line, 1000);

	if(quit || !line)
	  break;
		  
	if(*line)
	  GUI.ParseLine(line);
      }
    
    GUI.ParseLine(quit_callback);
    return 0;
  }

  spawn_non_readline_thread::~spawn_non_readline_thread()
  {
    //Terminate the readline and wait for it to finish
    if(!none)
      {
	cout << endl << endl << endl << "Press enter to quit..." << endl ;
  cout<<"... actually not: cancelling getline-thread ;)"<<endl<<endl;
	quit = 1;
	pthread_cancel(cmd);
	pthread_join(cmd,NULL);
	quit_callback = "";
	quit = 0;
	running = 0;
      }
  }
	
}
