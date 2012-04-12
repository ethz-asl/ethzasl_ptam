#include "gvars3/GUI.h"
#include "src/GUI_impl.h"
#include "gvars3/GStringUtil.h"

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
  }


  void print_history(ostream &ost)
  {
  }

  void GUI_impl::StopParserThread()
  {
  }

 }
