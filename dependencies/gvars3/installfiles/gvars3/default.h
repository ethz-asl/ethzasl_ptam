/*                       
	This file is part of the GVars3 Library.

	Copyright (C) 2009 The Authors

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

#ifndef GV3_INC_DEFUALT_H
#define GV3_INC_DEFUALT_H
#include <gvars3/config.h>

#ifdef GVARS3_HAVE_TOON
#include <TooN/TooN.h>
#endif

namespace GVars3
{

template<class C> struct DefaultValue
{
	static inline const C val()
	{
		return C();
	}
};

#ifdef GVARS3_HAVE_TOON
template<> struct DefaultValue<TooN::Vector<-1> >
{
	inline static const TooN::Vector<-1> val()
	{
		return TooN::makeVector(0);
	}
};
#endif
}

#endif
