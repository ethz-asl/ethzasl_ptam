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

#ifndef GV3_INC_TYPE_NAME_H
#define GV3_INC_TYPE_NAME_H

#include <string>
#include <gvars3/config.h>

#ifdef GVARS3_HAVE_TOON
	#include <TooN/TooN.h>
#endif

namespace GVars3
{

	template <class T> std::string type_name() 
	{
#ifdef WIN32
      static std::string funcname = std::string(__FUNCSIG__);
      static std::string::size_type begin = funcname.find("type_name<")+10;
      static std::string name = funcname.substr(begin, funcname.rfind(">") - begin);
#else
      std::string funcname = std::string(__PRETTY_FUNCTION__);
      std::string bname = funcname.substr(funcname.rfind(" = ")+3);
	  std::string name = bname.substr(0, bname.length()-1);
#endif
	  return name;
	}
	
	template<> inline std::string type_name<std::string>()
	{
		return "string";
	}

	#ifdef GVARS3_HAVE_TOON
		template<> inline std::string type_name<TooN::Vector<> >()
		{
			return "TooN::Vector<>";
		}
	#endif
	
	template <class T> std::string type_name(const T& t)
	{
	  return type_name<T>();
	}

}



#endif

