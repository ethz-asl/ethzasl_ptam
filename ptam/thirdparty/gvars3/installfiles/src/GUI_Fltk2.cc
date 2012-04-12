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

#include <gvars3/GUI_Fltk2.h>
#include <gvars3/GUI_Widgets.h>
#include <gvars3/GStringUtil.h>
#include <gvars3/instances.h>

#include <vector>
#include <string.h>
#include <sstream>
#include <cstdlib>

#ifndef WIN32
// for usleep
#include <unistd.h>
#endif

#include <fltk/run.h>
#include <fltk/Window.h>
#include <fltk/Widget.h>
#include <fltk/Button.h>
#include <fltk/CheckButton.h>
#include <fltk/ToggleButton.h>
#include <fltk/Slider.h>
#include <fltk/InvisibleBox.h>
#include <fltk/ValueInput.h>
#include <fltk/Threads.h>

#define POLL_UPDATE 1

#ifdef WIN32 
#undef AddMonitor
#endif

using namespace std;
namespace GVars3
{

GUI_Fltk2::GUI_Fltk2(class GUI *pGUI, GVars2* pGV2)
{
	gui=pGUI;
	gv2=pGV2;
	init = 0;
	gui->RegisterCommand("GUI.InitXInterface", InitXInterfaceCB, this);
}

static void poll_callback(void* v)
{
	class GUI_Fltk2* t = (class GUI_Fltk2*) v;

	t->poll_windows();

	//Repeat the polling timeout
	fltk::repeat_timeout(0.02,  poll_callback, v);
}

void* GUI_Fltk2::do_stuff_CB(void* v)
{
	GUI_Fltk2* t = (GUI_Fltk2*)v;

	//Add a one shot timeout which makes widgets poll for changes
	fltk::add_timeout(0.02,  poll_callback,  v);

	for(;;)
	{
        fltk::lock();
		fltk::run();
		fltk::unlock();
		//If no windows are present, sleep and start again
#ifdef WIN32
        Sleep(10);
#else
        usleep(100000);
#endif
	}
}

#define UI (*(GUI_Fltk2*)(ptr))

void GUI_Fltk2::InitXInterfaceCB(void* ptr, string sCommand, string sParams)
{
	UI.InitXInterface(sParams);
}

void GUI_Fltk2::InitXInterface(string args)
{
	if(init)
	{
		cerr << "??GUI_Fltk2::InitXInterface: already initialised." << endl;
		return;
	}

	vector<string> vs = ChopAndUnquoteString(args);

	if(vs.size() > 0)
		name = vs[0];
	else
		name = "GUI";

	gui->RegisterCommand(name + ".AddWindow", AddWindowCB, this);

	init = 1;
}

void GUI_Fltk2::start_thread()
{
	pthread_create(&gui_thread, 0, do_stuff_CB, this);
}

void GUI_Fltk2::process_in_crnt_thread()
{
	fltk::lock();
	poll_windows();
	fltk::check();
	fltk::unlock();
}


////////////////////////////////////////////////////////////////////////////////
//
// Helper functions
//

string GUI_Fltk2::remove_suffix(string cmd, string suffix)
{
	cmd.resize(cmd.length() - suffix.length());
	return cmd;
}

bool GUI_Fltk2::check_window(string win_name, string err)
{
	if(windows.find(win_name) != windows.end())
		return 1;

	cerr << "!!!! GUI_Fltk2::" << err << "<<: window " << win_name << " does not exist!" << endl;
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
// Window
//

void GUI_Fltk2::AddWindowCB(void* ptr, string sCommand, string sParams)
{
	fltk::lock();
	UI.AddWindow(sParams);
	fltk::unlock();
}

class GUI_Fltk2_win:public fltk::Window
{
	public:
		GUI_Fltk2_win(int w, string name, string caption, class GUI* pgui)
		:fltk::Window(w, 10),win_name(name),labl(caption),gui(pgui)
		{
			label(caption.c_str());
			callback(my_callback);
		}

		void add(fltk::Widget* widg)
		{
            int lw = 0, lh = 0;

            if((widg->align() & fltk::ALIGN_LEFT) && !(widg->align() & fltk::ALIGN_INSIDE) )
                widg->measure_label(lw, lh);

            fltk::Window::add(widg);

            //Position the widget
			//Resize the window
			resize(w(), (GUI_Fltk2::widget_height + 2 * GUI_Fltk2::widget_padding_y)*children());
			widg->resize(GUI_Fltk2::widget_padding_x + lw, (GUI_Fltk2::widget_height + 2 * GUI_Fltk2::widget_padding_y)*(children()-1) + GUI_Fltk2::widget_padding_y,
						 w() - 2 * GUI_Fltk2::widget_padding_x - lw, GUI_Fltk2::widget_height);
                        layout();
		}

		void poll_update()
		{
			//Go through all the widgets, updating if necessary
			for(int c=0; c < children(); c++)
				child(c)->do_callback(child(c),POLL_UPDATE);
		}

	private:
		string  win_name, labl;
		class GUI* 	gui;

		static void my_callback(fltk::Widget* w)
		{
			//Called on close event
			GUI_Fltk2_win* win = (GUI_Fltk2_win*) w;
			win->gui->ParseLine(win->win_name+".Destroy");
		}
};

void GUI_Fltk2::poll_windows()
{
	for(map<string, window>::iterator i=windows.begin(); i != windows.end(); i++)
	{
		i->second.win->poll_update();
	}
}

void GUI_Fltk2::AddWindow(string sParams)
{
	if(!init)
	{
		cerr << "! GUI_Fltk2::AddWindow: Not initialised. Call InitXInterface first." << endl;
		return;
	}

	vector<string> vs = ChopAndUnquoteString(sParams);
	if(vs.size()<1 || vs.size()>3)
	{
		cerr << "! GUI_Motif::AddWindow: need 1 - 3 params: Name, Caption=Name, Width=200 " << endl;
		return;
	}

	if(windows.count(vs[0])>0)
	{
		cerr << "? GUI_Motif::AddWindow: A window with id \"" << vs[0] << "\" already exists." << endl;
		return;
	}

	string sCaption;
	int width = 200;

	if(vs.size()>=2)
		sCaption=vs[1];
	else
		sCaption=vs[0];

	if(vs.size() > 2)
		width = atoi(vs[2].c_str());

	window w;
	w.win = new GUI_Fltk2_win(width, vs[0], sCaption, gui);
	w.win->end();
	w.win->show();

	fltk::check();

	windows[vs[0]] = w;

	gui->RegisterCommand(vs[0] + ".Destroy", DestroyWindowCB, this);
	gui->RegisterCommand(vs[0] + ".AddPushButton", AddPushButtonCB, this);
	gui->RegisterCommand(vs[0] + ".AddToggleButton", AddToggleButtonCB, this);
	gui->RegisterCommand(vs[0] + ".AddSlider", AddSliderCB, this);
	gui->RegisterCommand(vs[0] + ".AddMonitor", AddMonitorCB, this);
	gui->RegisterCommand(vs[0] + ".AddSpin", AddSpinCB, this);
	gui->RegisterCommand(vs[0] + ".AddSmallToggleButton", AddSmallToggleCB, this);
}

void GUI_Fltk2::DestroyWindowCB(void* ptr, string cmd, string args)
{
	fltk::lock();
	UI.DestroyWindow(cmd);
	fltk::unlock();
}

void GUI_Fltk2::DestroyWindow(string cmd)
{
	string win_name = remove_suffix(cmd, ".Destroy");

	if(!check_window(win_name, "Destroy"))
		return;

	gui->UnRegisterCommand(win_name + ".Destroy");
	gui->UnRegisterCommand(win_name + ".AddPushButton");
	gui->UnRegisterCommand(win_name + ".AddToggleButton");
	gui->UnRegisterCommand(win_name + ".AddSlider");
	gui->UnRegisterCommand(win_name + ".AddMonitor");
	gui->UnRegisterCommand(win_name + ".AddSpin");
	gui->UnRegisterCommand(win_name + ".AddSmallToggleButton");

    windows[win_name].win->destroy();
	//delete windows[win_name].win;
	//windows[win_name].win->hide();
	windows.erase(win_name);
}

////////////////////////////////////////////////////////////////////////////////
//
// Pushbutton stuff
//

void GUI_Fltk2::AddPushButtonCB(void* ptr, string cmd, string args)
{
	fltk::lock();
	UI.AddPushButton(cmd, args);
	fltk::unlock();
}

class cmd_button2 :public fltk::Button
{
	public:
		cmd_button2(string name, string command, class GUI* pgui)
		:fltk::Button(0, 0, 1, 1),labl(name), cmd(command), gui(pgui)
		{
			label(labl.c_str());
			callback(my_callback);
		}

	private:
		//The button label just stores the pointer, so we need to store the string here
		string cmd, labl;
		class GUI*   gui;

		static void my_callback(fltk::Widget* w, long what_shall_I_do)
		{
			if(what_shall_I_do == POLL_UPDATE)
				return;

			cmd_button2* b = (cmd_button2 *) w;
			b->gui->ParseLine(b->cmd);
		}
};



void GUI_Fltk2::AddPushButton(string cmd, string args)
{
	string window_name = remove_suffix(cmd, ".AddPushButton");

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size()!=2)
	{
		cerr << "! GUI_Fltk2::AddPushButton: Need 2 params (name, command)." << endl;
		return;
	};

	if(!check_window(window_name, "AddPushButton"))
		return;

	window& w=windows[window_name];

	//Create button

	fltk::Button* b = new cmd_button2(vs[0], vs[1], gui);
	w.win->add(b);
}


////////////////////////////////////////////////////////////////////////////////
//
// Toggle button stuff
//

void GUI_Fltk2::AddToggleButtonCB(void* ptr, string cmd, string args)
{
	fltk::lock();
	UI.AddToggleButton(cmd, args);
	fltk::unlock();
}

class toggle_button2: public fltk::CheckButton
{
	public:
		toggle_button2(string name, string gvar, GVars2* gv2, string def)
		:fltk::CheckButton(0, 0, 1, 1),labl(name)
		{
			callback(my_callback);

			gv2->Register(my_int, gvar, def, true);
			value(*my_int);
			label(labl.c_str());

			when(fltk::WHEN_CHANGED);
		}

		void poll_update()
		{
			if(*my_int != value())
				value((bool)*my_int);
		}

	private:
		gvar2_int my_int;
		string labl;

		static void my_callback(fltk::Widget* w, long what_shall_I_do)
		{
			toggle_button2* b = (toggle_button2*)w;

			if(what_shall_I_do == POLL_UPDATE)
				b->poll_update();
			else
				*(b->my_int) = b->value();
		}
};


void GUI_Fltk2::AddToggleButton(string cmd, string args)
{
	string window_name = remove_suffix(cmd, ".AddToggleButton");

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 2 && vs.size() != 3)
	{
		cerr << "! GUI_Fltk2::AddToggleButton: Need 2-3 params (name, gvar2_int name, {default})." << endl;
		return;
	}

	if(!check_window(window_name, "AddToggleButton"))
		return;

	window& w = windows[window_name];

	if(vs.size() == 2)
		vs.push_back("true");

	fltk::Widget* b = new toggle_button2(vs[0], vs[1], gv2, vs[2]);
	w.win->add(b);
}

////////////////////////////////////////////////////////////////////////////////
//
// Slider stuff
//

void GUI_Fltk2::AddSliderCB(void* ptr, string cmd, string args)
{
	fltk::lock();
	UI.AddSlider(cmd, args);
	fltk::unlock();
}

typedef fltk::Slider slider_type;

class slider_bar2: public slider_type
{
	public:
		slider_bar2(string gvar_name, string title, GVars2 *pgv2, double min, double max)
		:slider_type(0, 0, 1, 1),gv2(pgv2),varname(gvar_name)
		{
			//type(slider_type::TICK_BOTH);
			copy_label(title.c_str());
			align(fltk::ALIGN_LEFT);
			range(min, max);
			callback(my_callback);
			when(fltk::WHEN_CHANGED);
			step(0);
			poll_update();
		}

		void poll_update()
		{
			string crnt=gv2->StringValue(varname, true);

			if(crnt != cached_value)
			{
				cached_value = crnt;
				double newval=0;
				serialize::from_string(crnt, newval);

				//Update range if necessary
				if(newval > maximum()) maximum(newval);
				if(newval < minimum()) minimum(newval);

				value(newval);
			}
		}

		void set_gvar_to_value()
		{
			ostringstream ost;
			ost << value();
			gv2->SetVar(varname, ost.str(), 1);
			cached_value = ost.str();
		}

	private:
		GVars2 *gv2;
		string varname, cached_value;

		static void my_callback(fltk::Widget* w, long what_shall_I_do)
		{
			slider_bar2* s = (slider_bar2*) w;

			if(what_shall_I_do == POLL_UPDATE)
				s->poll_update();
			else
				s->set_gvar_to_value();
		}
};

void GUI_Fltk2::AddSlider(string cmd, string args)
{
	string win_name = remove_suffix(cmd, ".AddSlider");
    string title;

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 3 && vs.size() !=4 )
	{
		cerr << "! GUI_Fltk2::AddSlider: Need 3-4 params (gvar_name min max [title]])." << endl;
		return;
	}

	if(!check_window(win_name, "AddSlider"))
		return;

	window& w = windows[win_name];

	double min, max;

	serialize::from_string(vs[1], min);
	serialize::from_string(vs[2], max);

    if( vs.size() == 4)
        title = vs[3];

	fltk::Widget* b = new slider_bar2(vs[0], title, gv2, min, max);

	w.win->add(b);
}


////////////////////////////////////////////////////////////////////////////////
//
// Monitor stuff
//

void GUI_Fltk2::AddMonitorCB(void* ptr, string cmd, string args)
{
	fltk::lock();
	UI.AddMonitor(cmd, args);
	fltk::unlock();
}

class monitor2: public fltk::InvisibleBox
{
	public:
		monitor2(string t, string gvar_name, GVars2* pgv)
		:fltk::InvisibleBox(0, 0, 1, 1),title(t), gv_name(gvar_name),gv(pgv)
		{
			callback(my_callback);
			align(fltk::ALIGN_INSIDE_LEFT);
			poll_update();
		}

		void poll_update()
		{
			string gvar_text = gv->StringValue(gv_name);

			if(gvar_text != cached_gv_text)
			{
				cached_gv_text = gvar_text;
				full_label = title + ": " + cached_gv_text;
				copy_label(full_label.c_str());
				redraw();
			}
		}

	private:
		string title, gv_name, full_label, cached_gv_text;
		GVars2* gv;

		static void my_callback(fltk::Widget* w, long what_shall_I_do)
		{
			if(what_shall_I_do == POLL_UPDATE)
				((monitor2*)w)->poll_update();
		}

};


void GUI_Fltk2::AddMonitor(std::string cmd, std::string args)
{
	string win_name = remove_suffix(cmd, ".AddMonitor");

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 2 && vs.size() != 2)
	{
		cerr << "! GUI_Fltk2::AddMonitor: Need 2 or 3 params (label, gvar_name, {poll update !!ignored!! )." << endl;
		return;
	}

	if(!check_window(win_name, "AddSlider"))
		return;

	window& w = windows[win_name];

	fltk::Widget* m = new monitor2(vs[0], vs[1], gv2);
	w.win->add(m);
}

////////////////////////////////////////////////////////////////////////////////
//
// Spincontrol stuff
//

void GUI_Fltk2::AddSpinCB(void* ptr, string cmd, string args)
{
	fltk::lock();
	UI.AddSpin(cmd, args);
	fltk::unlock();
}

class spin2: public fltk::ValueInput
{
	public:
		spin2(string gvar_name, string t,  GVars2 *pgv2, double min, double max)
		:fltk::ValueInput(0, 0, 1, 1),gv2(pgv2),varname(gvar_name), title(t)
		{
			copy_label(title.c_str());
			align(fltk::ALIGN_LEFT);
			range(min, max);
			callback(my_callback);
			when(fltk::WHEN_CHANGED);
			step(0);
			poll_update();
		}

		void poll_update()
		{
			string crnt=gv2->StringValue(varname, true);


			if(crnt != cached_value)
			{
				cached_value = crnt;
				double newval=0;
				serialize::from_string(crnt, newval);

				//Update range if necessary
				if(newval > maximum()) maximum(newval);
				if(newval < minimum()) minimum(newval);

				value(newval);
			}
		}

		void set_gvar_to_value()
		{
			ostringstream ost;
			ost << value();
			gv2->SetVar(varname, ost.str(), 1);
			cached_value = ost.str();
		}

	private:
		GVars2 *gv2;
		string varname, cached_value, title;

		static void my_callback(fltk::Widget* w, long what_shall_I_do)
		{
			spin2* s = (spin2*) w;

			if(what_shall_I_do == POLL_UPDATE)
				s->poll_update();
			else
				s->set_gvar_to_value();
		}
};

void GUI_Fltk2::AddSpin(string cmd, string args)
{
	string win_name = remove_suffix(cmd, ".AddSpin");
	string title = "";

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 3 && vs.size() != 4)
	{
		cerr << "! GUI_Fltk2::AddSpin: Need 3-4 params (gvar_name min max [title]])." << endl;
		return;
	}

	if(!check_window(win_name, "AddSpin"))
		return;

	window& w = windows[win_name];

	double min, max;

	serialize::from_string(vs[1], min);
	serialize::from_string(vs[2], max);

    if( vs.size() == 4)
        title = vs[3];

	fltk::Widget* b = new spin2(vs[0], title, gv2, min, max);

	w.win->add(b);
}

////////////////////////////////////////////////////////////////////////////////
//
// real toogle button stuff
//

void GUI_Fltk2::AddSmallToggleCB(void* ptr, string cmd, string args)
{
	fltk::lock();
	UI.AddSmallToggle(cmd, args);
	fltk::unlock();
}

class small_toggle2: public fltk::ToggleButton
{
	public:
		small_toggle2(string gvar_name, string t,  GVars2 *pgv2, string def)
		:fltk::ToggleButton(0, 0, 1, 1),title(t)
		{
			copy_label(title.c_str());

			callback(my_callback);
			when(fltk::WHEN_CHANGED);

			pgv2->Register(my_int, gvar_name, def, true);
			value(*my_int);
		}

		void poll_update()
		{
			if(*my_int != value())
				value((bool)*my_int);
		}

	private:
		gvar2_int my_int;
		string title;

		static void my_callback(fltk::Widget* w, long what_shall_I_do)
		{
			small_toggle2* b = (small_toggle2*)w;

			if(what_shall_I_do == POLL_UPDATE)
				b->poll_update();
			else
				*(b->my_int) = b->value();
		}
};

void GUI_Fltk2::AddSmallToggle(string cmd, string args)
{
	string win_name = remove_suffix(cmd, ".AddSmallToggleButton");
	string title = "";

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 2 && vs.size() != 3)
	{
		cerr << "! GUI_Fltk2::AddSmallToggleButton: Need 2-3 params (name, gvar2_int name, {default}).." << endl;
		return;
	}

	if(!check_window(win_name, "AddSmallToggleButton"))
		return;

	window& w = windows[win_name];

	if(vs.size() == 2)
		vs.push_back("true");

	fltk::Widget* b = new small_toggle2( vs[1], vs[0], gv2, vs[2]);
	w.win->add(b);
}

//Instantiations
class GUI_Fltk2 GUI_Fltk2_instance(&GUI, &GV2);

void GUIWidgets::process_in_crnt_thread()
{
	GUI_Fltk2_instance.process_in_crnt_thread();
}

void GUIWidgets::start_thread()
{
	GUI_Fltk2_instance.start_thread();
}



}
