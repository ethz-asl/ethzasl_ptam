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

#include <gvars3/GUI.h>
#include <gvars3/instances.h>
using namespace GVars3;

#include <gvars3/GUI_readline.h>
#include <unistd.h>
#include <TooN/numerics.h>

using namespace std;

bool bDone;



void foo_callback(void* ptr, string sCommand, string sParams)
{
  cout << "Foo callback here! called with command " << endl <<
    "\"" << sCommand <<  "\""<< endl <<
    "and params" << endl <<
    "\"" << sParams << "\"." << endl;
}

void quit_callback(void* ptr, string sCommand, string sParams)
{
  bDone =true;
}



int main(void)
{
  bDone = false;
  GUI.LoadFile("autoexec.cfg");
  GUI.RegisterCommand("foo", foo_callback, NULL);
  GUI.RegisterCommand("quit", quit_callback, NULL);
  GUI.RegisterCommand("exit", quit_callback, NULL);
  GUI.SetupReadlineCompletion();
  
  GVars3::spawn_readline_thread thread("quit");
  
  GV2.GetInt("TestInt", 10);
  GV2.GetDouble("TestDouble", 10);
  GV2.GetString("TestString", "hello world");
  GV2.GetString("TestString2", "hello world");
  GVars3::gvar3<Vector<> > v;
  GV2.Register(v,"TestVecM1", "[   10 10 10]" );
  GVars3::gvar3<Vector<3> > v3;
  GV2.Register(v3,"TestVec3", "[   10 10 10]" );
  GV3::get<string>("SilentString", "pssst!" , SILENT);
  GV3::get("hidden__________hidden", 1, SILENT | HIDDEN);  

  gvar2_double a_double;
  gvar2_int an_int;
  GV2.Register(a_double, "test_double", 0., true);
  GV2.Register(an_int, "test_int", 0, true);
  
  while(!bDone)
    {
      usleep(10000);
    };
  cout << "end." << endl;
  
  return 0;
};
