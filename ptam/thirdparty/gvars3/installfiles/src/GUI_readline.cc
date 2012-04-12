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
#include "src/GUI_impl.h"
#include "gvars3/GUI_readline.h"

#include <pthread.h>
#include <poll.h>
#include <cstdlib>
#include <readline/readline.h>
#include <readline/history.h>

using namespace std;

namespace GVars3
{

	bool spawn_readline_thread::running=0;
	bool spawn_readline_thread::quit=0;
	std::string spawn_readline_thread::quit_callback;

	spawn_readline_thread::spawn_readline_thread(const std::string& cb)
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
			std::cerr << "Error: readline thread already running.\n";
		}
	}


	int rlhook()
	{
		return 1;
	}

	void* spawn_readline_thread::proc(void *)
	{
		char* line;
		GUI.SetupReadlineCompletion();

		rl_event_hook = rlhook;
		
		while(1)
		{
			line=readline("> ");

			if(quit || !line)
				break;

			if(*line)
			  {
			    GUI.ParseLine(line);
			    add_history(line);
			  };
		}
		
		GUI.ParseLine(quit_callback);

		return 0;
	}

	spawn_readline_thread::~spawn_readline_thread()
	{
		//Terminate the readline and wait for it to finish
		if(!none)
		{
			quit = 1;
			rl_done = 1;
			rl_stuff_char('\n');
			pthread_join(cmd, 0);
			quit_callback = "";
			quit = 0;
			running = 0;
		}
	}
	

	std::string readline_in_current_thread::quit_callback;

	readline_in_current_thread::~readline_in_current_thread()
	{
		rl_deprep_terminal();
	}

	readline_in_current_thread::readline_in_current_thread(const string& s)
	{
		quit_callback = s;
		GUI.SetupReadlineCompletion();
		rl_event_hook = rlhook;
		rl_set_keyboard_input_timeout(0);
		rl_callback_handler_install("> ", lineread);
	}

	void readline_in_current_thread::poll()
	{
		struct pollfd p;

		p.fd = 0;
		p.events=POLLIN | POLLHUP;

		if(::poll(&p, 1, 0) > 0)
			rl_callback_read_char();
	}

	void readline_in_current_thread::lineread(char* line)
	{
		if(line == NULL)
		{
			rl_deprep_terminal();
			GUI.ParseLine(quit_callback);
		}
		else
		{
			GUI.ParseLine(line);
			if(line != string(""))
				add_history(line);
		}
	}


char * GUI_impl::ReadlineCommandGeneratorCB(const char *szText, int nState)
{
  return mpReadlineCompleterGUI->ReadlineCommandGenerator(szText, nState);

}
char* GV3ReadlineCommandGenerator(const char *szText, int nState)
{
	static vector<string> tags;
	static vector<string>::iterator tag_i;
	
	if(!nState)
	{
		tags = GV3::tag_list();
		tag_i = tags.begin();
		
	};

	while(tag_i != tags.end())
	{
		size_t text_len = strlen(szText);
		bool bEqualsAtEnd = false;
		if(text_len>0)
		  if(szText[text_len-1]=='=')
		    bEqualsAtEnd = true;
		string sCurrent = *tag_i;
		string sComparison = *tag_i;
		if(bEqualsAtEnd)
		  sComparison = sComparison + "=";

		tag_i++;

		if(strncmp(szText, sComparison.c_str(), text_len) == 0)
		  {
			//If it's a prefect match, complete with the value
		        string sCompleted;
			if(text_len == sComparison.size())
			  sCompleted = szText + ((bEqualsAtEnd?"":"=") + GV3::get_var(sCurrent));
			else
			  sCompleted = sCurrent;
			
			char* r;
			
			if((r= (char*)malloc(sCompleted.size() + 1)) == NULL)
				return 0;

			strcpy(r, sCompleted.c_str());
			return r;
		}
	}

	return 0;
}



char * GUI_impl::ReadlineCommandGenerator(const char *szText, int nState)
{
  static map<string,CallbackVector>::iterator iRegistered;
  static int nTextLength;
  static int nOffset;
  const char *pcName;
  
  if(!nState)
  {
	iRegistered = mmCallBackMap.begin();
	nTextLength = strlen(szText);
	nOffset=-1;
  }
  
  while(iRegistered!=mmCallBackMap.end())
    {
      pcName = iRegistered->first.c_str();
      iRegistered++;
      if(strncmp(pcName,szText,nTextLength) == 0)
	{
	  char *t = (char*) malloc(strlen(pcName)+2);
	  if(!t) return NULL;
	  strcpy(t,pcName);
	  t[strlen(pcName)]=' ';
	  t[strlen(pcName)+1]=0;
	  return t;
	};
    };
  
  if(nOffset==-1)
    nOffset = nState;

  return GV3ReadlineCommandGenerator(szText, nState-nOffset);
  
};

}
