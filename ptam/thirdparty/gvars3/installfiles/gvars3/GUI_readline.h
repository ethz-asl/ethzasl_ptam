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

#ifndef __GUI_READLINE_H
#define __GUI_READLINE_H

#include <string>

namespace GVars3
{

	class spawn_readline_thread
	{
		public:
			spawn_readline_thread(const std::string&);
			~spawn_readline_thread();

		private:
			static bool running;
			static bool quit;
			static std::string quit_callback;
			pthread_t cmd;
			bool 	  none;
			static  void* proc(void*);
	};


	class readline_in_current_thread
	{
		public:
			void poll();
			readline_in_current_thread(const std::string&);
			~readline_in_current_thread();
		private:
			static void lineread(char*);	
			static std::string quit_callback;


	};
};
#endif
