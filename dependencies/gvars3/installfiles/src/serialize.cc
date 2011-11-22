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

#include "gvars3/serialize.h"
#include <vector>

using namespace std;


namespace GVars3
{
namespace serialize
{
	std::string to_string(const std::string& s)
	{
		ostringstream os;
		os << "\"";
		
		for(size_t i=0; i < s.size(); i++)
		{
			if(s[i] == '\\')
				os << "\\\\";
			else if(s[i] == '\n')
				os << "\\n";
			else
				os << s[i];
		}

		os << "\"";
		return os.str();
	}

	string FromStream<string>::from(istream& in)
	{	
		string s;

		bool quoted=0;
		int c;

		//Eat whitespace
		in >> ws;

		if((c=in.get()) == EOF)
			return s;

		if(c == '"')
			quoted=1;
		
		//This variable holds an escape sequence in progress.
		//empty means no escaping.
		string escape;

		for(;;)
		{
			c = in.get();
			if(c == EOF || (quoted && escape.empty() && c == '"'))
				break;
			else if(escape.empty() && c == '\\')
				escape = "\\";
			else if(!escape.empty())
				escape += c;
			else
				s += c;

			//Check escapes
			if(escape == "\\\\")
			{
				s+="\\"; 
				escape.clear();
			}
			else if(escape == "\\n")
			{
				s+="\n";
				escape.clear();
			}
		}
		
		//Append any trailing parts of an escape sequence
		s += escape;

		return s;
	}

	int check_stream(std::istream& i)
	{
		if(i.good())
			return 0;

		if(i.bad())
			return 1;

		if(i.fail())
		{
			return -i.tellg();
		}
		return 0;
	}

}


}
