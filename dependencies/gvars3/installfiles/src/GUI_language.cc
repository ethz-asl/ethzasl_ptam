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
#include "gvars3/GStringUtil.h"
#include <vector>
#include <iostream>
#include <set>
#include <pthread.h>

using namespace std;

namespace GVars3
{

	template<class C> class ThreadLocal
	{
		private:
			pthread_key_t key;

			static void deleter(void* v)
			{
				delete static_cast<C*>(v);
			}
		
		public:
			ThreadLocal()
			{
				pthread_key_create(&key, deleter);
				pthread_setspecific(key, new C);
			}

			~ThreadLocal()
			{
				deleter(pthread_getspecific(key));
				pthread_setspecific(key, 0);
				pthread_key_delete(key);
			}


			C& operator()()
			{
				return *static_cast<C*>(pthread_getspecific(key));
			}
	};

	template<class A, class B> class MutexMap
	{
		private:
			map<A, B> _map;
			pthread_mutex_t mutex;

		public:
			MutexMap()
			{
				pthread_mutex_init(&mutex, 0);
			}

			~MutexMap()
			{
				pthread_mutex_destroy(&mutex);
			}

			B get(const A& a)
			{
				B b;
				pthread_mutex_lock(&mutex);
				b = _map[a];
				pthread_mutex_unlock(&mutex);
				return b;
			}

			void set(const A&a, const B& b)
			{
				pthread_mutex_lock(&mutex);
				_map[a] = b;
				pthread_mutex_unlock(&mutex);
			}
	};


	class GUI_language
	{
		
		public:
			GUI_language()
			{
				GUI.RegisterCommand(".", collect_lineCB, this);
				GUI.RegisterCommand("function", functionCB, this);
				GUI.RegisterCommand("endfunction", endfunctionCB, this);
				GUI.RegisterCommand("if_equal", gui_if_equalCB, this);
				GUI.RegisterCommand("else", gui_if_elseCB, this);
				GUI.RegisterCommand("endif", gui_endifCB, this);
			}


			~GUI_language()
			{
			}


		private:
			pthread_mutex_t  functionlist_mutex;

			ThreadLocal<string> current_function, if_gvar, if_string;
			ThreadLocal<vector<string> > collection, ifbit, elsebit;
			MutexMap<string, vector<string> > functions;

			static GUI_language& C(void* v)
			{
				return *static_cast<GUI_language*>(v);
			}

			
			#define CallBack(X) static void X##CB(void* t, string a, string b){C(t).X(a, b);} 

			CallBack(collect_line);
			void collect_line(string, string l)
			{
				collection().push_back(l);
			}


			
			CallBack(function);
			void function(string name, string args)
			{

				vector<string> vs = ChopAndUnquoteString(args);
				if(vs.size() != 1)
				{
					cerr << "Error: " << name << " takes 1 argument: " << name << " name\n";
					return;
				}

				current_function()=vs[0];
				collection().clear();
			}

			CallBack(endfunction)
			void endfunction(string name, string args)
			{
				if(current_function() == "")
				{
					cerr << "Error: " << name << ": no current function.\n";
					return;
				}

				vector<string> vs = ChopAndUnquoteString(args);
				if(vs.size() != 0)
					cerr << "Warning: " << name << " takes 0 arguments.\n";

				functions.set(current_function(), collection());

				GUI.RegisterCommand(current_function(), runfuncCB, this);

				current_function().clear();
				collection().clear();
			}
			
			CallBack(runfunc)
			void runfunc(string name, string /*args*/)
			{
				vector<string> v = functions.get(name);
				for(unsigned int i=0; i < v.size(); i++)
					GUI.ParseLine(v[i]);
			}


			CallBack(gui_if_equal)
			void gui_if_equal(string name, string args)
			{
				vector<string> vs = ChopAndUnquoteString(args);
				if(vs.size() != 2)
				{
					cerr << "Error: " << name << " takes 2 arguments: " << name << " gvar string\n";
					return;
				}

				collection().clear();
				if_gvar() = vs[0];
				if_string() = vs[1];
			}


			CallBack(gui_if_else)
			void gui_if_else(string /*name*/, string /*args*/)
			{
				ifbit() = collection();
				if(ifbit().empty())
					ifbit().push_back("");
				collection().clear();
			}
			
			CallBack(gui_endif)
			void gui_endif(string /*name*/, string /*args*/)
			{
				if(ifbit().empty())
					ifbit() = collection();
				else 
					elsebit() = collection();

				collection().clear();
				
				//Save a copy, since it canget trashed
				vector<string> ib = ifbit(), eb = elsebit();
				string gv = if_gvar(), st = if_string();

				ifbit().clear();
				elsebit().clear();
				if_gvar().clear();
				if_string().clear();

				if(GV3::get_var(gv) == st)
					for(unsigned int i=0; i < ib.size(); i++)
						GUI.ParseLine(ib[i]);
				else
					for(unsigned int i=0; i < eb.size(); i++)
						GUI.ParseLine(eb[i]);
			}



	};
		
	GUI_language GUI_language_instance;

}
