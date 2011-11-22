#include "gvars3/GUI.h"
#include "src/GUI_impl.h"
#include "gvars3/GStringUtil.h"


#include <gvars3/GUI_non_readline.h>
#include <pthread.h>

#include <cctype>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <algorithm>

using namespace std;

namespace GVars3
{
 void GUI_impl::SetupReadlineCompletion()
  {
  }

  char ** GUI_impl::ReadlineCompletionFunction (const char *text, int start, int end)
  {
    return NULL;
  }

 void GUI_impl::StartParserThread()
  {
    if(mpParserThread)  // Only makes sense to have one instance of the parser thread.
      return;
    
    mpParserThread = new spawn_non_readline_thread("");
  }


  void print_history(ostream &ost)
  {
  }

  void GUI_impl::StopParserThread()
  {
    if(!mpParserThread)
      return;

    delete( (spawn_non_readline_thread*) mpParserThread);
    mpParserThread = NULL;
  }

 }
