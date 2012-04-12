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
#include "gvars3/GUI_Fltk.h"
#include "gvars3/GStringUtil.h"
#include <vector>
#include <string.h>
#include <cstdlib>
#include <sstream>
#include <unistd.h>
#include <map>
#include <set>
#include <string>

#include <error.h>

#include <FL/Fl.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Widget.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Check_Button.H>
#include <Fl/Fl_Value_Slider.H>
#include <Fl/Fl_Value_Input.H>
#include <Fl/Fl_Box.H>


#define POLL_UPDATE 1

using namespace std;
namespace GVars3
{

GUI_Fltk::GUI_Fltk()
{
	init = 0;
	GUI.RegisterCommand("GUI.InitXInterface", InitXInterfaceCB, this);
}


void poll_callback(void* v)
{
	GUI_Fltk* t = (GUI_Fltk*) v;

	t->poll_windows();
	Fl::check();

	//Repeat the polling timeout
	Fl::repeat_timeout(0.02,  poll_callback, v);
}

void* GUI_Fltk::do_stuff_CB(void* v)
{
	GUI_Fltk* t = (GUI_Fltk*)v;

	//Add a one shot timeout which makes widgets poll for changes
	Fl::add_timeout(0.02,  poll_callback,  v);
	
	for(;;)
	{
		//Fl::lock();	
		Fl::run();
		Fl::check();
		//Fl::unlock();
		
		//If no windows are present, sleep and start again
		usleep(100000);
		
	}
}


#define UI (*(GUI_Fltk*)(ptr))

void GUI_Fltk::InitXInterfaceCB(void* ptr, string sCommand, string sParams)
{
	UI.InitXInterface(sParams);
}

void GUI_Fltk::InitXInterface(string args)
{
	if(init)
	{
		cerr << "??GUI_Fltk::InitXInterface: already initialised." << endl;
		return;
	}
		
	vector<string> vs = ChopAndUnquoteString(args);

	if(vs.size() > 0)
		name = vs[0];
	else
		name = "GUI";

	GUI.RegisterCommand(name + ".AddWindow", AddWindowCB, this);

	init = 1;
}

void GUI_Fltk::start_thread()
{
	pthread_create(&gui_thread, 0, do_stuff_CB, this);
}

void GUI_Fltk::process_in_crnt_thread()
{
	//Fl::lock();
	poll_windows();
	Fl::check();
	//Fl::unlock();
}


////////////////////////////////////////////////////////////////////////////////
//
// Helper functions
//

string GUI_Fltk::remove_suffix(string cmd, string suffix)
{
	cmd.resize(cmd.length() - suffix.length());
	return cmd;
}

bool GUI_Fltk::check_window(string win_name, string err)
{
	if(windows.find(win_name) != windows.end())
		return 1;

	cerr << "!!!! GUI_Fltk::" << err << "<<: window " << win_name << " does not exist!" << endl;
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
// Window
//

void GUI_Fltk::AddWindowCB(void* ptr, string sCommand, string sParams)
{
	//Fl::lock();
	UI.AddWindow(sParams);
	//Fl::unlock();
}

class GUI_Fltk_win:public Fl_Window
{
	public:
		GUI_Fltk_win(int w, string name, string caption)
		:Fl_Window(w, 0),win_name(name),labl(caption)
		{
			label(caption.c_str());
			callback(my_callback);
		}

		void add(Fl_Widget* widg)
		{
            int lw = 0, lh = 0;

            if((widg->align() & FL_ALIGN_LEFT) && !(widg->align() & FL_ALIGN_INSIDE) )
                widg->measure_label(lw, lh);




			//Position the widget
			widg->resize(GUI_Fltk::widget_padding_x + lw, h() + GUI_Fltk::widget_padding_y, 
						 w() - 2 * GUI_Fltk::widget_padding_x - lw , GUI_Fltk::widget_height);

			//Resize the window
			size(w(), h() + GUI_Fltk::widget_height + 2 * GUI_Fltk::widget_padding_y);

			Fl_Window::add(widg);
		}

		void poll_update()
		{
			//Go through all the widgets, updating if necessary
			for(int c=0; c < children(); c++)
				child(c)->do_callback(child(c),POLL_UPDATE);
		}

	private:
		string  win_name, labl;
		
		static void my_callback(Fl_Widget* w)
		{
			//Called on close event
			GUI_Fltk_win* win = (GUI_Fltk_win*) w;
			GUI.ParseLine(win->win_name+".Destroy");
		}
};

void GUI_Fltk::poll_windows()
{
	for(map<string, window>::iterator i=windows.begin(); i != windows.end(); i++)
	{
		if(i->second.showme)
		{
			i->second.win->show();
			i->second.showme = false;
		}
		i->second.win->poll_update();
	}
}



void GUI_Fltk::AddWindow(string sParams)
{
	if(!init)
	{
		cerr << "! GUI_Fltk::AddWindow: Not initialised. Call InitXInterface first." << endl;
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
	w.win = new GUI_Fltk_win(width, vs[0], sCaption);
	w.showme = true;
	w.win->end();
	//w.win->show();

	windows[vs[0]] = w;

	GUI.RegisterCommand(vs[0] + ".Destroy", DestroyWindowCB, this);
	GUI.RegisterCommand(vs[0] + ".AddPushButton", AddPushButtonCB, this);
	GUI.RegisterCommand(vs[0] + ".AddToggleButton", AddToggleButtonCB, this);
	GUI.RegisterCommand(vs[0] + ".AddSlider", AddSliderCB, this);
	GUI.RegisterCommand(vs[0] + ".AddMonitor", AddMonitorCB, this);
	GUI.RegisterCommand(vs[0] + ".AddSpin", AddSpinCB, this);
	GUI.RegisterCommand(vs[0] + ".AddLabel", AddLabelCB, this);
	//gui->RegisterCommand(vs[0] + ".AddSmallToggleButton", AddSmallToggleCB, this);
}

void GUI_Fltk::DestroyWindowCB(void* ptr, string cmd, string args)
{
	//Fl::lock();
	UI.DestroyWindow(cmd);
	//Fl::unlock();
}

void GUI_Fltk::DestroyWindow(string cmd)
{
	string win_name = remove_suffix(cmd, ".Destroy");

	if(!check_window(win_name, "Destroy"))
		return;
	
	GUI.UnRegisterCommand(win_name + ".Destroy");
	GUI.UnRegisterCommand(win_name + ".AddPushButton");
	GUI.UnRegisterCommand(win_name + ".AddToggleButton");
	GUI.UnRegisterCommand(win_name + ".AddSlider");
	GUI.UnRegisterCommand(win_name + ".AddMonitor");
	GUI.UnRegisterCommand(win_name + ".AddSpin");
	GUI.UnRegisterCommand(win_name + ".AddLabel");
	//gui->UnRegisterCommand(win_name + ".AddSmallToggleButton");


	delete windows[win_name].win;
	//windows[win_name].win->hide();
	windows.erase(win_name);
}




////////////////////////////////////////////////////////////////////////////////
//
// Pushbutton stuff
//

void GUI_Fltk::AddPushButtonCB(void* ptr, string cmd, string args)
{
	//Fl::lock();
	UI.AddPushButton(cmd, args);
	//Fl::unlock();
}

class cmd_button:public Fl_Button
{
	public:
		cmd_button(string name, string command)
		:Fl_Button(0, 0, 1, 1),labl(name), cmd(command)
		{
			label(labl.c_str());
			callback(my_callback);
		}

	private:
		//The button label just stores the pointer, so we need to store the string here	
		string cmd, labl;
		static void my_callback(Fl_Widget* w, long what_shall_I_do)
		{
			if(what_shall_I_do == POLL_UPDATE)
				return;
	
			cmd_button* b = (cmd_button*) w;
			GUI.ParseLine(b->cmd);
		}
};



void GUI_Fltk::AddPushButton(string cmd, string args)
{
	string window_name = remove_suffix(cmd, ".AddPushButton");

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size()!=2) 
	{
		cerr << "! GUI_Fltk::AddPushButton: Need 2 params (name, command)." << endl;
		return;
	};

	if(!check_window(window_name, "AddPushButton"))
		return;

	window& w=windows[window_name];

	//Create button

	Fl_Button* b = new cmd_button(vs[0], vs[1]);
	w.win->add(b);
}


////////////////////////////////////////////////////////////////////////////////
//
// Toggle button stuff
//

void GUI_Fltk::AddToggleButtonCB(void* ptr, string cmd, string args)
{
	//Fl::lock();
	UI.AddToggleButton(cmd, args);
	//Fl::unlock();
}

class toggle_button: public Fl_Check_Button
{
	public:
		toggle_button(string name, string gvar, string def)
		:Fl_Check_Button(0, 0, 1, 1),labl(name),my_int(gvar, def, true)
		{
			callback(my_callback);
			value(*my_int);
			label(labl.c_str());

			when(FL_WHEN_CHANGED);
		}

		void poll_update()
		{
			if(*my_int != value())
				value((bool)*my_int);
		}

	private:
		gvar3<int> my_int;
		string labl;

		static void my_callback(Fl_Widget* w, long what_shall_I_do)
		{
			toggle_button* b = (toggle_button*)w;

			if(what_shall_I_do == POLL_UPDATE)
				b->poll_update();
			else
				*(b->my_int) = b->value();
		}
};


void GUI_Fltk::AddToggleButton(string cmd, string args)
{
	string window_name = remove_suffix(cmd, ".AddToggleButton");

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 2 && vs.size() != 3) 
	{
		cerr << "! GUI_Fltk::AddToggleButton: Need 2-3 params (name, gvar2_int name, {default})." << endl;
		return;
	}

	if(!check_window(window_name, "AddToggleButton"))
		return;

	window& w = windows[window_name];

	if(vs.size() == 2)
		vs.push_back("true");

	Fl_Widget* b = new toggle_button(vs[0], vs[1], vs[2]);
	w.win->add(b);
}

////////////////////////////////////////////////////////////////////////////////
//
// Slider stuff
//

void GUI_Fltk::AddSliderCB(void* ptr, string cmd, string args)
{
	//Fl::lock();
	UI.AddSlider(cmd, args);
	//Fl::unlock();
}

typedef Fl_Slider slider_type;

class slider_bar: public slider_type
{
	public:
    slider_bar(string gvar_name, double min, double max, bool show_val_)
	:slider_type(0, 0, 1, 1),varname(gvar_name), show_val(show_val_)
		{
			type(FL_HORIZONTAL);
			bounds(min, max);
			align(FL_ALIGN_CENTER);
			callback(my_callback);
			when(FL_WHEN_CHANGED);
			step(0);
			poll_update();
		}

    double compute_slider_frac()
    {
	fl_font(labelfont(), labelsize());
	double lw = fl_width(label());
	double s_size = lw + 5;
	return s_size / w();	
    }

    void resize(int x_, int y_, int w_, int h_)
    {
	slider_type::resize(x_,y_,w_,h_);
	if (show_val)
	    slider_size(compute_slider_frac());
    }


		void poll_update()
		{		    
			string crnt=GV3::get_var(varname);
			
			if(crnt != cached_value)
			{
				cached_value = crnt;
				double newval=0;
				serialize::from_string(crnt, newval);

				//Update range if necessary
				if(newval > maximum()) maximum(newval);
				if(newval < minimum()) minimum(newval);

				if (show_val) {
				    label(cached_value.c_str());
				    double new_frac = compute_slider_frac();
				    if (new_frac > slider_size())
					slider_size(new_frac);
				}
				value(newval);
			}
		}

		string set_gvar_to_value()
		{	
			ostringstream ost;
			ost.precision(3);
			ost << value();
			GV3::set_var(varname, ost.str(), 1);
			return ost.str();
		}

	private:
		string varname, cached_value;
    bool show_val;

		static void my_callback(Fl_Widget* w, long what_shall_I_do)
		{
			slider_bar* s = (slider_bar*) w;

			if(what_shall_I_do == POLL_UPDATE)
				s->poll_update();
			else {
				string val = s->set_gvar_to_value();
				s->poll_update();
			}
		}
};

void GUI_Fltk::AddSlider(string cmd, string args)
{
	string win_name = remove_suffix(cmd, ".AddSlider");

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 3 && vs.size() !=4)
	{
		cerr << "! GUI_Fltk::AddSlider: Need 3 or 4 params (gvar_name min max [showval])." << endl;
		return;
	}

	if(!check_window(win_name, "AddSlider"))
		return;
	
	window& w = windows[win_name];
	
	double min, max;
	
	serialize::from_string(vs[1], min);
	serialize::from_string(vs[2], max);

	Fl_Widget* b = new slider_bar(vs[0], min, max, vs.size()==4);

	w.win->add(b);
}


////////////////////////////////////////////////////////////////////////////////
//
// Monitor stuff
// 

void GUI_Fltk::AddMonitorCB(void* ptr, string cmd, string args)
{
	//Fl::lock();
	UI.AddMonitor(cmd, args);
	//Fl::unlock();
}

class monitor: public Fl_Box
{
	public:
		monitor(string t, string gvar_name)
		:Fl_Box(0, 0, 1, 1),title(t), gv_name(gvar_name) 
		{
			callback(my_callback);
			align(FL_ALIGN_TOP|FL_ALIGN_INSIDE|FL_ALIGN_LEFT);
			poll_update();
		}

		void poll_update()
		{
			string gvar_text = "\"" + GV3::get_var(gv_name) + "\"";

			if(gvar_text != cached_gv_text)
			{
				cached_gv_text = gvar_text;
				full_label = title + ": " + cached_gv_text;
				label(full_label.c_str());
			}
		}

	private:
		string title, gv_name, full_label, cached_gv_text;

		static void my_callback(Fl_Widget* w, long what_shall_I_do)
		{
			if(what_shall_I_do == POLL_UPDATE)
				((monitor*)w)->poll_update();
		}

};


void GUI_Fltk::AddMonitor(string cmd, string args)
{
	string win_name = remove_suffix(cmd, ".AddMonitor");

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 2 && vs.size() != 2)
	{
		cerr << "! GUI_Fltk::AddMonitor: Need 2 or 3 params (label, gvar_name, {poll update !!ignored!! )." << endl;
		return;
	}

	if(!check_window(win_name, "AddSlider"))
		return;

	window& w = windows[win_name];
	
	Fl_Widget* m = new monitor(vs[0], vs[1]);
	w.win->add(m);
}

//GUI_Fltk  Gui_Fltk(&GUI);





////////////////////////////////////////////////////////////////////////////////
//
// Spincontrol stuff
//

void GUI_Fltk::AddSpinCB(void* ptr, string cmd, string args)
{
	//Fl::lock();
	UI.AddSpin(cmd, args);
	//Fl::unlock();
}

class spin2: public Fl_Value_Input
{
	public:
		spin2(string gvar_name, string t,  double min, double max)
		:Fl_Value_Input(0, 0, 1, 1),varname(gvar_name), title(t)
		{
			label(title.c_str());
			align(FL_ALIGN_LEFT);
			bounds(min, max);
			callback(my_callback);
			when(FL_WHEN_CHANGED);
			increment(0, 1);
			poll_update();
		}

		void poll_update()
		{
			string crnt=GV3::get_var(varname);


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
			GV3::set_var(varname, ost.str(), 1);
			cached_value = ost.str();
		}

	private:
		string varname, cached_value, title;

		static void my_callback(Fl_Widget* w, long what_shall_I_do)
		{
			spin2* s = (spin2*) w;

			if(what_shall_I_do == POLL_UPDATE)
				s->poll_update();
			else
				s->set_gvar_to_value();
		}
};

void GUI_Fltk::AddSpin(string cmd, string args)
{
	string win_name = remove_suffix(cmd, ".AddSpin");
	string title = "";

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 3 && vs.size() != 4)
	{
		cerr << "! GUI_Fltk::AddSpin: Need 3-4 params (gvar_name min max [title]])." << endl;
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

	Fl_Widget* b = new spin2(vs[0], title, min, max);

	w.win->add(b);
}


void GUI_Fltk::AddLabelCB(void* ptr, string cmd, string args)
{
	//Fl::lock();
	UI.AddLabel(cmd, args);
	//Fl::unlock();
}


class label: public Fl_Box
{
	public:
		label(string t)
		:Fl_Box(0, 0, 1, 1), title(t)
		{
			align(FL_ALIGN_CENTER);
                        Fl_Box::label(title.c_str());
		}
        private:
            string title;
};

void GUI_Fltk::AddLabel(string cmd, string args)
{
    string win_name = remove_suffix(cmd, ".AddLabel");

	vector<string> vs = ChopAndUnquoteString(args);
	if(vs.size() != 1)
	{
		cout << "! GUI_Fltk::AddLabel: Need 1 params (label)." << endl;
		return;
	}

	if(!check_window(win_name, "AddLabel"))
		return;

	window& w = windows[win_name];

	Fl_Widget* m = new label(vs[0]);
	w.win->add(m);
}





//Instantiations
class GUI_Fltk GUI_Fltk_instance;

void GUIWidgets::process_in_crnt_thread()
{
	GUI_Fltk_instance.process_in_crnt_thread();
}

void GUIWidgets::start_thread()
{
	GUI_Fltk_instance.start_thread();
}


}
