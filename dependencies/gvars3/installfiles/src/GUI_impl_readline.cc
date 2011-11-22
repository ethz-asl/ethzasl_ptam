#include "gvars3/GUI.h"
#include "src/GUI_impl.h"
#include "gvars3/GStringUtil.h"

#include <gvars3/GUI_readline.h>

#include <pthread.h>

#include <cctype>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <sys/wait.h>
#include <stdlib.h>
#include <algorithm>

#include <readline/readline.h>
#include <readline/history.h>

using namespace std;

namespace GVars3
{
 
  void GUI_impl::SetupReadlineCompletion()
  {
    mpReadlineCompleterGUI = this;
    rl_attempted_completion_function = ReadlineCompletionFunction;
    rl_basic_word_break_characters = " \t\n\"\\'`@$><;|&{("; 
  }
  
  char ** GUI_impl::ReadlineCompletionFunction (const char *text, int /*start*/, int /*end*/)
  {
    rl_completion_append_character=0;
    char **matches;
    matches = (char **)NULL;
    matches = rl_completion_matches (text,   ReadlineCommandGeneratorCB);
    return (matches);
  }
  
  void print_history(ostream &ost)
  {
    HIST_ENTRY **apHistEntries = history_list();
    if(apHistEntries)
      while((*apHistEntries)!=NULL)
	{
	  ost << (*apHistEntries)->line << endl;
	  apHistEntries++;
	}
  };


  void GUI_impl::StartParserThread()
  {
    if(mpParserThread)  // Only makes sense to have one instance of the parser thread.
      return;
    
    mpParserThread = new spawn_readline_thread("");
  }

void GUI_impl::StopParserThread()
  {
    if(!mpParserThread)
      return;

    delete( (spawn_readline_thread*) mpParserThread);
    mpParserThread = NULL;
  }


}
