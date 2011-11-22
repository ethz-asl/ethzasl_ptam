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

#ifndef GV3_INV_GVARS3_H
#define GV3_INV_GVARS3_H
#include <map>
#include <set>
#include <string>
#include <list>
#include <vector>
#include <iostream>

#include <gvars3/config.h>
#include <gvars3/default.h>
#include <gvars3/type_name.h>
#include <gvars3/serialize.h>

namespace GVars3
{
void parse_warning(int e, std::string type, std::string name, std::string from);

struct type_mismatch{};
struct gvar_was_not_defined{};


class GV3;


class BaseMap
{
	public:
		virtual std::string get_as_string(const std::string& name)=0;
		virtual int set_from_string(const std::string& name, const std::string& val)=0;
		virtual std::string name()=0;
		virtual std::vector<std::string> list_tags()=0;
		virtual ~BaseMap(){};
};

template<class T> class gvar2
{
  friend class GV3;
 public:
  gvar2() {data=NULL;}      // Debugging comfort feature, makes unregistered vars more obvious.
  T& operator*()
    {
      return *data;
    }
  
  const T & operator*() const 
  {
     return *data;
  }
  
  T* operator->()
    {
      return data;
    }
  const T * operator->() const 
  {
    return data;
  }

  bool IsRegistered() const
  {
    return data!=NULL;
  }
   
 protected:
  T* data;
};

// Bit-masks for gvar registration:
// SILENT makes gvars not complain if it has to use the default;
// HIDDEN makes vars not appear in gvarlist unless used with -a

enum { SILENT = 1<<0, HIDDEN = 1<<1, FATAL_IF_NOT_DEFINED = 1<<2};
       

typedef gvar2<double> gvar2_double;
typedef gvar2<int> gvar2_int;
typedef gvar2<std::string> gvar2_string;

template<class T> class gvar3: public gvar2<T>
{
	friend class GV3;
	public:
	inline gvar3(const std::string& name, const T& default_val = T(), int flags=0);
	inline gvar3(const std::string& name, const std::string& default_val, int flags);
	inline gvar3(){};
};

template<> class gvar3<std::string>: public gvar2<std::string>
{
	friend class GV3;
	public:
	inline gvar3(const std::string& name, const std::string& default_val = "", int flags=0);
	inline gvar3(){};
};
class GV3
{
	private:

		template<class T> class TypedMap: public BaseMap
		{
			private:
				friend class GV3;

				//This gives us singletons
				static TypedMap& instance()
				{
					static TypedMap* inst=0;

					if(!inst)
					{
						inst = new TypedMap();
						//Register ourselves with GV3
						GV3::add_typemap(inst);
					}

					return *inst;
				}

				//Get a data member	
				T* get(const std::string& n)
				{
					typename std::map<std::string, T>::iterator i;

					i = data.find(n);

					if(i == data.end())
						return NULL;
					else
						return &(i->second);
				}
				
				//Replace a member using erase then reinsert
				T& safe_replace(const std::string& n, const T& t)
				{
					typename std::map<std::string, T>::iterator i, j;
					//Keep track of the neighboring point
					//to pass as a hint to insert.
					i = data.find(n);
					j=i;

					if(i != data.end())
					{
						j++;
						data.erase(i);
					}

					data.insert(j, make_pair(n, t));

					return data.insert(j, make_pair(n, t))->second;
				}

				//Create a data member
				T* create(const std::string& n)
				{
					return data.insert(make_pair(n, DefaultValue<T>::val()))->second;
				}
			
				virtual int set_from_string(const std::string& name, const std::string& val)
				{
					std::istringstream is(val);
					T tmp = serialize::from_stream<T>(is);
					int e = serialize::check_stream(is);

					if(e == 0)
						safe_replace(name, tmp);
					return e;
				}

				virtual std::string get_as_string(const std::string& name)
				{	
					typename std::map<std::string, T>::iterator i = data.find(name);

					if(i == data.end())
						i = data.insert(make_pair(name, DefaultValue<T>::val())).first;

					return serialize::to_string(i->second);
				}

				virtual std::string name()
				{
					return type_name<T>();
				}

				virtual std::vector<std::string> list_tags()
				{
					std::vector<std::string> l;
					for(typename std::map<std::string,T>::iterator i=data.begin(); i != data.end(); i++)
						l.push_back(i->first);
					return l;
				}

				std::map<std::string, T>		data;
		};

		template<class T> friend class TypedMap;

		template<class T> static T* attempt_get(const std::string& name)
		{
			T* d = TypedMap<T>::instance().get(name);
			
			if(!d)	 //Data not present in map of the correct type
			{
				//Does it exist with a different type?
				if(registered_type_and_trait.count(name))
				{		//Yes: programmer error.
					std::cerr << "GV3:Error: type mismatch while getting " << type_name<T>() << " " << name << ": already registered "
							"as type " << registered_type_and_trait[name].first->name() << ". Fix your code.\n";

					throw type_mismatch();
				}
				else
					return NULL;
			}

			return d;
		}

		template<class T> static T& safe_replace(const std::string& name, const T& t)
		{
			return TypedMap<T>::instance().safe_replace(name, t);
		}

		static void add_typemap(BaseMap* m);

		static std::map<std::string, std::string>		unmatched_tags;
		static std::map<std::string, std::pair<BaseMap*, int> >	registered_type_and_trait;
		static std::list<BaseMap*>				maps;

		
		template<class T> static T* get_by_val(const std::string& name, const T& default_val, int flags);
		template<class T> static T* get_by_str(const std::string& name, const std::string& default_val, int flags);
		template<class T> static T* register_new_gvar(const std::string &name, const T& default_val, int flags);

	public:
		//Get references by name
		template<class T> static T& get(const std::string& name, const T& default_val=DefaultValue<T>::val(), int flags=0);
		template<class T> static T& get(const std::string& name, std::string default_val, int flags=0);
		
		//Register GVars
		template<class T> static void Register(gvar2<T>& gv, const std::string& name, const T& default_val=DefaultValue<T>::val(), int flags=0);
		template<class T> static void Register(gvar2<T>& gv, const std::string& name, const std::string& default_val, int flags=0);
		static inline void Register(gvar2<std::string>& gv, const std::string& name, const std::string& default_val="", int flags=0);

		//Get and set by string only
		static std::string get_var(std::string name);
		static bool set_var(std::string name, std::string val, bool silent=false);

		//Some helper functions
		static void print_var_list(std::ostream& o, std::string pattern="", bool show_all=true);
		static std::vector<std::string> tag_list();
};



template<class T> gvar3<T>::gvar3(const std::string& name, const T& default_val, int flags)
{
	GV3::Register(*this, name, default_val, flags);
}

template<class T> gvar3<T>::gvar3(const std::string& name, const std::string& default_val, int flags)
{
	GV3::Register(*this, name, default_val, flags);
}
gvar3<std::string>::gvar3(const std::string& name, const std::string& default_val, int flags)
{
	GV3::Register(*this, name, default_val, flags);
}


#include <gvars3/gv3_implementation.hh>

//Compatibility with old GVARS
class GVars2
{
	public:
		template<class T> void Register(gvar2<T>& gv, const std::string& name, const T& default_val=T(), int flags=false)
		{ 
			GV3::Register(gv, name, default_val, flags);
		}

		template<class T> void Register(gvar2<T>& gv, const std::string& name, const std::string& default_val, int flags=false)
		{
			GV3::Register(gv, name, default_val, flags);
		}

		inline void Register(gvar2<std::string>& gv, const std::string& name, const std::string& default_val="", int flags=false)
		{
			GV3::Register(gv, name, default_val, flags);
		}

		template<class T> T& Get(const std::string& name, const T& default_val=T(),  int flags=false)
		{
			return GV3::get<T>(name, default_val, flags);
		}

		template<class T> T& Get(const std::string& name, const std::string& default_val="", int flags=false)
		{
			return GV3::get<T>(name, default_val, flags);
		}

		inline std::string& Get(const std::string& name, const std::string& default_val="", int flags=false)
		{
			return GV3::get<std::string>(name, default_val, flags);
		}

		void SetVar(std::string sVar, std::string sValue, bool silent=false);
		void SetVar(std::string s);


		int& GetInt(const std::string& name, int default_val=0, int flags=0);
		double& GetDouble(const std::string& name, double default_val=0.0, int flags=0); 
		std::string& GetString(const std::string& name, const std::string& default_val="", int flags=0); 

		int& GetInt(const std::string& name, const std::string& default_val, int flags=0);
		double& GetDouble(const std::string& name, const std::string& default_val, int flags=0); 

		std::string StringValue(const std::string &name, bool no_quotes=false);
		void PrintVarList(std::ostream& os=std::cout);
		void PrintVar(std::string s, std::ostream& os, bool bEndl = true);

		//char* ReadlineCommandGenerator(const char *szText, int nState);

	private:
};

}
#endif
