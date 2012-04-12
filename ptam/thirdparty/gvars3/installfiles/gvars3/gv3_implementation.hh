
template<class T> inline void robust_set_var(std::string& name, const T& value)
{
	GV3::safe_replace<T>(name,value);
};

// TODO: Make the specialisation for matrices as well. 
template<class T> T* GV3::register_new_gvar(const std::string& name, const T& default_val, int flags)
{
	std::map<std::string, std::string>::iterator i;

	i = unmatched_tags.find(name);

	T* d;

	registered_type_and_trait[name] = std::pair<BaseMap*, int>(&TypedMap<T>::instance(), flags);

	//Look to see if ``name'' has not already been set	
	if(i == unmatched_tags.end())
	{
		if(flags & FATAL_IF_NOT_DEFINED)
		{
			std::cerr << "!!GV3::Register: " << type_name<T>() << " " << name << " must be defined. Exception. " << std::endl;
			throw gvar_was_not_defined();
		};

		if(!(flags & SILENT))
			std::cerr << "? GV3::Register: " << type_name<T>() << " " << name << " undefined. Defaults to " << serialize::to_string(default_val) << std::endl;

		d = &safe_replace(name, default_val);
	}
	else
	{
		std::istringstream is(i->second);
		T value = serialize::from_stream<T>(is);
		int e = serialize::check_stream(is);

		parse_warning(e, type_name<T>(), name, i->second);
		if(e > 0 && flags & FATAL_IF_NOT_DEFINED)
		{
			std::cerr << "!!GV3::Register: " << type_name<T>() << " " << name << " must be defined. Exception. " << std::endl;
			throw gvar_was_not_defined();
		}

		d = &safe_replace(name, value);

		unmatched_tags.erase(i);
	}

	return d;
}


template<class T> T* GV3::get_by_val(const std::string& name, const T& default_val, int flags)
{
  T* d = attempt_get<T>(name);
  if(!d)
    d = register_new_gvar(name, default_val, flags);
  return d;
}

template<class T> T* GV3::get_by_str(const std::string& name, const std::string& default_val, int flags)
{
  T* d = attempt_get<T>(name);
  if(d!=NULL) return d;
  
  std::istringstream is(default_val);
  T def = serialize::from_stream<T>(is);
  int e = serialize::check_stream(is);
  
  parse_warning(e, type_name<T>(), name, default_val);
  
  return register_new_gvar(name, def, flags);
}

template<>inline std::string& GV3::get<std::string>(const std::string& name, std::string default_val, int flags)
{
	return *get_by_val(name, default_val, flags);
}

template<class T> T& GV3::get(const std::string& name, std::string default_val, int flags)
{
	return *get_by_str<T>(name, default_val, flags);
}

template<class T> T& GV3::get(const std::string& name, const T& default_val, int flags)
{
	return *get_by_val(name, default_val, flags);
}



template<class T> void GV3::Register(gvar2<T>& gv, const std::string& name, const T& default_val, int flags)
{
	gv.data = get_by_val(name, default_val, flags);
}

template<class T> void GV3::Register(gvar2<T>& gv, const std::string& name, const std::string& default_val, int flags)
{
	gv.data = get_by_str<T>(name, default_val, flags);
}

inline void GV3::Register(gvar2<std::string>& gv, const std::string& name, const std::string& default_val, int flags)
{
	gv.data = get_by_val(name, default_val, flags);
}





