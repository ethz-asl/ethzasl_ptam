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

#ifndef GV3_INC_GSTRINGUTIL_H
#define GV3_INC_GSTRINGUTIL_H

#include <vector>
#include <string>
#include <sstream>
#include <gvars3/serialize.h>

namespace GVars3
{

	std::string UncommentString(std::string s);
	std::vector<std::string> ChopAndUnquoteString(std::string s);
	template <class T> T* ParseAndAllocate(std::string s)
	{
		std::istringstream is(s);	
		T* n = new T(serialize::from_stream<T>(is));
		return n;
	}

}



#endif
