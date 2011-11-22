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

#include "gvars3/GUI.h"
#include "src/GUI_impl.h"
#include "gvars3/GStringUtil.h"

#include <cctype>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <algorithm>



using namespace std;
namespace GVars3
{

  GUI::GUI(GVars2* /* v2 */){
    }

	GUI_impl& GUI::I()
	{
		return GUI_impl_singleton<>::instance();
	}

	GUI::GUI()
	{
	}

	void GUI::RegisterCommand(string cmd, GUICallbackProc c, void* p)
	{
		I().RegisterCommand(cmd, c, p);
	}


	void GUI::UnRegisterAllCommands(void* p)
	{
		I().UnRegisterAllCommands(p);
	}

	void GUI::UnRegisterCommand(std::string sCommandName, void* thisptr)
	{
		I().UnRegisterCommand(sCommandName, thisptr);
	}

	void GUI::UnRegisterCommand(std::string sCommandName)
	{
		I().UnRegisterCommand(sCommandName);
	}

	void GUI::ParseLine(std::string s, bool bSilentFailure)
	{
		I().ParseLine(s, bSilentFailure);
	}

	void GUI::ParseStream(std::istream& is)
	{
		I().ParseStream(is);
	}

	void GUI::LoadFile(std::string sFileName)
	{
		I().LoadFile(sFileName);
	}


	bool GUI::CallCallbacks(std::string sCommand, std::string sParams)
	{
		return I().CallCallbacks(sCommand, sParams);
	}

	void GUI::SetupReadlineCompletion()
	{
		I().SetupReadlineCompletion();
	}

	void GUI::StartParserThread()
	{
		I().StartParserThread();
	}

	void GUI::StopParserThread()
	{
		I();
		GUI_impl::StopParserThread();
	}

	int GUI::parseArguments( const int argc, char * argv[], int start, const std::string prefix, const std::string execKeyword)
	{
		return I().parseArguments(argc, argv, start, prefix, execKeyword);
	}






  bool setvar(string s)
  {
    //Execution failed. Maybe its an assignment.
    string::size_type n;
    n=s.find("=");

    if(n != string::npos)
      {    
	string var = s.substr(0, n);
	string val = s.substr(n+1);

	//Strip whitespace from around var;
	string::size_type s=0, e = var.length()-1; 
	for(; isspace(var[s]) && s < var.length(); s++);
	if(s==var.length()) // All whitespace before the `='?
	  return false; 
	for(; isspace(var[e]); e--);
	if(e >= s)
	  {
	    var = var.substr(s, e-s+1);
			
	    //Strip whitespace from around val;			
	    s = 0, e = val.length() - 1;
	    for(; isspace(val[s]) && s < val.length(); s++);
	    if( s < val.length())
	      {
		for(; isspace(val[e]); e--);
		val = val.substr(s, e-s+1);
	      }
	    else val = "";

	    GV3::set_var(var, val);
	    return true;
	  }
      }

    return false;
  }



  GUI_impl *GUI_impl::mpReadlineCompleterGUI=NULL;


  void GUI_impl::UnRegisterCommand(string sCommandName)
  {
    mmCallBackMap.erase(sCommandName);
  };

  void GUI_impl::UnRegisterAllCommands(void* thisptr)
  {
    for(map<string, CallbackVector>::iterator i=mmCallBackMap.begin(); i!=mmCallBackMap.end(); i++)
      UnRegisterCommand(i->first, thisptr);
  };
  
  void GUI_impl::UnRegisterCommand(string sCommandName, void* thisptr)
  {
    CallbackVector &cbv = mmCallBackMap[sCommandName];
    for(int i = static_cast<int>(cbv.size()) - 1; i>=0; i--)
      if(cbv[i].thisptr == thisptr)
	cbv.erase(cbv.begin() + i);
  };

  void GUI_impl::RegisterCommand(string sCommandName, GUICallbackProc callback, void* thisptr)
  {
    if(builtins.count(sCommandName))
      {
	cerr << "!!GUI_impl::RegisterCommand: Tried to register reserved keyword " << sCommandName << "." << endl;
	return;
      }

    CallbackInfoStruct s;
    s.cbp = callback;
    s.thisptr = thisptr;

    bool bAlreadyThere = false;
    CallbackVector *cbv = &mmCallBackMap[sCommandName];


    for(CallbackVector::iterator i = cbv->begin(); i<cbv->end();i++)
      if( (i->cbp ==s.cbp) && (i->thisptr == s.thisptr))
	bAlreadyThere=true;

    if(!bAlreadyThere)
      cbv->push_back(s);
  };


  // Returns true if something was called;
  // otherwise returns false
  bool GUI_impl::CallCallbacks(string sCommand, string sParams)
  {
    if(mmCallBackMap.count(sCommand)==0)
      return false;

    //Make a copy of this callback vector, since the command might call Unregister.
    CallbackVector cbv = mmCallBackMap[sCommand];
    if(cbv.size() == 0)
      return false;

    for(CallbackVector::iterator i = cbv.begin(); i<cbv.end();i++)
      i->cbp(i->thisptr, sCommand, sParams);

    return true;
  };


  void GUI_impl::ParseStream(istream& is)
  {
    string buffer;
    while (getline(is, buffer)) {
      // Lines ending with '\' are taken as continuing on the next line.
      while(!buffer.empty() && buffer[buffer.length() - 1] == '\\') {
	string buffer2;
	if (! getline(is, buffer2))
	  break;
	buffer = buffer.substr(0, buffer.length() - 1) + buffer2;
      }
      ParseLine(buffer);
    }  
  }

  void GUI_impl::LoadFile(string sFileName)
  {
    ifstream ifs;

    vector<string> v = ChopAndUnquoteString(sFileName);

    if(v.size()<1) 
      return;
		
    ifs.open(v[0].c_str());

    if(!ifs.good())
      {
	cerr << "! GUI_impl::Loadfile: Failed to load script file \"" << sFileName << "\"."<< endl;
	return;
      }

    ParseStream(ifs);
    ifs.close();
  }





  string::size_type FindCloseBrace(const string& s, string::size_type start, char op, char cl)
  {
    string::size_type open=1;
    string::size_type i;

    for(i=start+1; i < s.size() ; i++)
      {
	if(s[i] == op) 
	  open++;
	else if(s[i] == cl) 
	  open--;

	if(open == 0)
	  break;
      }
	
    if(i == s.size())
      i = s.npos;
    return i;
  }


  void GUI_impl::ParseLine(string s, bool bSilentFailure )
  {
    s = UncommentString(s);	
    if(s == "")
      return;





    // New for 2004! brace expansion! any line with {gvarname} in it 
    // will have the brace bit replaced with the contents of that gvar,
    // one line per word in the gvar value...
    // e.g. 
    // > TrackerList = Tracker FireTracker
    // > {TrackerList}.ReloadModel 
    // would be equivalent to writing 
    // > Tracker.ReloadModel 
    // > FireTracker.ReloadModel
    // Can use double braces so that brace expansion only occurs later;
    // e.g. GUI_Motif.AddPushButton DoStuffToTrackerList {{TrackerList}}.do_stuff
    // will create a single button which then exectues {TrackerList}.do_stuff
    // (Without double braces, the it would try to make two buttons!)

    {   // Brace expansion wrapper follows:
      string::size_type nOpenBracePos = s.find("{");
      //int nCloseBracePos = s.rfind("}");
      string::size_type nCloseBracePos = FindCloseBrace(s, nOpenBracePos, '{', '}');
      if( (nOpenBracePos  !=s.npos) && 
	  (nCloseBracePos !=s.npos) &&
	  (nCloseBracePos > nOpenBracePos))
	{   // Brace Pair found!!

	  string sBegin = s.substr(0,nOpenBracePos);
	  string sVarName = s.substr(nOpenBracePos+1,nCloseBracePos-nOpenBracePos-1);
	  string sEnd = s.substr(nCloseBracePos+1);
	
	  string::size_type nLength = sVarName.size();   // Check if it's a double brace: {{foo}} in which case remove one pair, but don't expand.
	  bool bIsDoubleQuoted = false;
	
	  if(nLength>2)
	    if((sVarName[0]=='{') && (sVarName[nLength-1]=='}'))
	      bIsDoubleQuoted = true;
	
	  if(!bIsDoubleQuoted)
	    {
	      //vector<string> vsExpandedList = ChopAndUnquoteString(mpGV2->StringValue(sVarName, true));
	      vector<string> vsExpandedList = ChopAndUnquoteString(GV3::get_var(sVarName));

	      //ER: look at the end of the string for '$'s to also perform the substitution
	      vector<string> chopped;
	      string end;
	      string::size_type dollarpos=s.npos, lastpos=0;
	      while((dollarpos = sEnd.find("$", dollarpos+1)) != s.npos)
		{
		  chopped.push_back(sEnd.substr(lastpos, dollarpos - lastpos));
		  lastpos = dollarpos+1;
		}
	      end = sEnd.substr(lastpos);

	      for(vector<string>::iterator i = vsExpandedList.begin(); i!=vsExpandedList.end();i++)
		{
		  string line = sBegin + *i;
		  for(vector<string>::iterator s=chopped.begin(); s != chopped.end(); s++)
		    line += *s + *i;
		  line += end;

		  ParseLine(line, bSilentFailure);
		}
	      return;
	    };
	
	  // if it was double quoted, just parse it as normal, but replace s with the new, single-braced thingy.
	  s = sBegin + sVarName + sEnd;
	}
    }
  
    // Newer for 2004: Round brace expansion 
    // Expands gvar in round braces to the value
    // e.g.  A = 2
    //       B = (A)
    // assigns 2 to B
    // And as before, use double brace to protect.

    {   // Round expansion wrapper follows:

      string::size_type nOpenBracePos = s.find("(");
      //int nCloseBracePos = s.rfind(")");
      string::size_type nCloseBracePos = FindCloseBrace(s, nOpenBracePos, '(', ')');

      //		cerr << "((( " << nOpenBracePos << "  " << nCloseBracePos << endl;
		

      if((nOpenBracePos  !=s.npos) && (nCloseBracePos !=s.npos) && (nCloseBracePos > nOpenBracePos))
	{   // Brace Pair found!!
	  //cerr << "Found (\n";
	  //cout << "Found brace pair. " << endl;
	  string sBegin = s.substr(0,nOpenBracePos);
	  string sVarName = s.substr(nOpenBracePos+1,nCloseBracePos-nOpenBracePos-1);
	  string sEnd = s.substr(nCloseBracePos+1);

	  //cerr << "varname = --" << sVarName << "--\n";

	  string::size_type nLength = sVarName.size();   // Check if it's a double brace: {{foo}} in which case remove one pair, but don't expand.
	  bool bIsDoubleQuoted = false;

	  if(nLength>2)
	    if((sVarName[0]=='(') && (sVarName[nLength-1]==')'))
	      {
		s = sBegin + sVarName + sEnd;  // Just remove the first set of braces.
		bIsDoubleQuoted = true;
	      };

	  if(!bIsDoubleQuoted)
	    {
	      //cerr << "varname = --" << sVarName << "--\n";
	      //cerr << "***************" << GV3::get_var(sVarName) << endl;
	      //string sExpanded = mpGV2->StringValue(sVarName, true);
	      string sExpanded = GV3::get_var(sVarName);
	      s = sBegin + sExpanded + sEnd;
	      //cout << "DEBUG : xx" << s << "xx" << endl;
	    };
	}
    }



    // Old ParseLine code follows, here no braces can be left (unless in arg.)
    istringstream ist(s);
		
    string sCommand;
    string sParams;
	
    //Get the first token (the command name)
    ist >> sCommand;
    if(sCommand == "")
      return;

    //Get everything else (the arguments)...
	
    //Remove any whitespace
    ist >> ws;
    getline(ist, sParams);
	
    //Attempt to execute command
    if (CallCallbacks(sCommand, sParams))
      return;
	
    if(setvar(s))
      return;

    if(!bSilentFailure) 
      cerr << "? GUI_impl::ParseLine: Unknown command \"" << sCommand << "\" or invalid assignment." << endl;
  }



  ////////////////////////////////////////////////////////////////////////////////
  //
  // Builtin commands

  void builtin_shell(void* /* ptr */, string /* sCommand */, string sParams)
  {
    vector<string> v = ChopAndUnquoteString(sParams);
    
    if(v.size()==0) 
      {
	cerr <<"? GUI_impl internal shell command: No prog/args given."<< endl; 
	return;
      }

    system(sParams.c_str());
  }

  void builtin_ls(void* ptr, string /*sCommand*/, string sParams)
  {
    GUI_impl* p = (GUI_impl*)ptr;
    p->ParseLine("shell ls " + sParams);	
  }

  void builtin_ll(void* ptr, string /*sCommand*/, string sParams)
  {
    GUI_impl* p = (GUI_impl*)ptr;
    p->ParseLine("shell ls -l " + sParams);	
  }

  void builtin_try(void* ptr, string /*sCommand*/, string sParams)
  {
    GUI_impl* p = (GUI_impl*)ptr;
    p->ParseLine(sParams, true);
  }

  void builtin_exec(void* ptr, string /*sCommand*/, string sParams)
  {
    GUI_impl* p = (GUI_impl*)ptr;
    p->LoadFile(sParams);
  }

  void builtin_echo(void* /*ptr*/, string /*sCommand*/, string sParams)
  {
    cout << sParams << endl;
  }

  void builtin_qmark(void* /*ptr*/, string /*sCommand*/, string /*sParams*/)
  {
    cout << "  Perhaps you'd like to type \"commandlist\" or \"gvarlist\"." << endl;
  }

  void builtin_if(void* ptr, string /*sCommand*/, string sParams)
  {
    GUI_impl* p = (GUI_impl*)ptr;
    vector<string> v = ChopAndUnquoteString(sParams);
    if(v.size()<2)
      {
	cerr <<"? GUI_impl internal if command: not enough params (syntax: if gvar command)"<< endl; 
	return;
      }

    string sValue = GV3::get_var(v[0]);
    if(sValue == "(not in GVar list)")
      return;
  
    // Have to do this to fix whitespace issues on non-registered gvars
    vector<string> vv = ChopAndUnquoteString(sValue); 
  
    if(vv.size()<1) 
      return;
    else if(vv[0]!="0")
      {
	string s;
	s = "";
	for(size_t i=1;i<v.size();i++)
	  s = s + " " +  v[i];
	p->ParseLine(s);
      }
    return;
  }

  void builtin_ifnot(void* ptr, string /*sCommand*/, string sParams)
  {
    GUI_impl* p = (GUI_impl*)ptr;
    vector<string> v = ChopAndUnquoteString(sParams);
    if(v.size()<2)
      {
	cerr <<"? GUI_impl internal ifnot command: not enough params (syntax: if gvar command)"<< endl; 
	return;
      }

    string sValue = GV3::get_var(v[0]);
    if(sValue == "(not in GVar list)")
      return;
  
    // Have to do this to fix whitespace issues on non-registered gvars
    vector<string> vv = ChopAndUnquoteString(sValue); 
  
    if(vv.size()<1) 
      return;
    else if(vv[0]=="0")
      {
	string s;
	s = "";
	for(size_t i=1;i<v.size();i++)
	  s = s + " " +  v[i];
	p->ParseLine(s);
      }
    return;
  }

  void builtin_ifeq(void* ptr, string /*sCommand*/, string sParams)
  {
    GUI_impl* p = (GUI_impl*)ptr;
    vector<string> v = ChopAndUnquoteString(sParams);
    if(v.size()<3)
      {
	cerr <<"? GUI_impl internal ifeq command: not enough params (syntax: if gvar command)"<< endl; 
	return;
      }

    string sValue = GV3::get_var(v[0]);
    string sIfValue = v[1];
    if(sValue == "(not in GVar list)")
      return;
  
    // Have to do this to fix whitespace issues on non-registered gvars
    vector<string> vv = ChopAndUnquoteString(sValue); 
  
    if(vv.size()<1) 
      return;
    else if(vv[0]==sIfValue)
      {
	string s;
	s = "";
	for(size_t i=2;i<v.size();i++)
	  s = s + " " +  v[i];
	p->ParseLine(s);
      }
    return;
  }

  void builtin_toggle(void* /*ptr*/, string /*sCommand*/, string sParams)
  {
    vector<string> v = ChopAndUnquoteString(sParams);
    if(v.size()!=1) 
      {
	cout <<"? GUI_impl internal toggle command: invalid num of params (syntax: toggle gvar)"<< endl; 
	return;
      };
    int nValue = GV3::get<int>(v[0]);
    if(nValue) 
      GV3::set_var(v[0],"0");
    else
      GV3::set_var(v[0],"1");
    return;
  
  };

  void builtin_set(void* /*ptr*/, string /*sCommand*/, string sParams)
  {
    setvar(sParams);
  }


  void builtin_gvarlist(void* /*ptr*/, string sCommand, string sParams)
  {
    bool error = false;
    bool print_all = false;
    string pattern = "";
  
    vector<string> v = ChopAndUnquoteString(sParams);
    if(v.size() > 0  && v[0][0] == '-')
      if(v[0].size() == 2)
	{
	  switch(v[0][1])
	    {
	    case 'a':
	      print_all = true;
	      break;
	    default:
	      error = true;
	    }
	  if(!error)
	    v.erase(v.begin());
	}
      else
	error = true;
  
    if(v.size()==1)
      pattern = v[0];
    else if(v.size() > 1)
      error = true;
  
    if(error)
      cout << "? GUI_impl internal " << sCommand << ": syntax is " << sCommand << " [-a] [pattern] " << endl;
    else
      GV3::print_var_list(cout, pattern, print_all);
  }


  void builtin_printvar(void* /*ptr*/, string /*sCommand*/, string sParams)
  {
    cout << sParams << "=" << GV3::get_var(sParams) << endl;;
  }


  void builtin_commandlist(void* ptr, string /*sCommand*/, string /*sParams*/)
  {
    GUI_impl* p = (GUI_impl*)ptr;

    cout << "  Builtin commands:" << endl;

    for(set<string>::iterator i=p->builtins.begin(); i!=p->builtins.end(); i++)
      cout << "    " << *i << endl;


    cout << "  Registered commands:" << endl;

    for(map<string,CallbackVector>::iterator i=p->mmCallBackMap.begin(); i!=p->mmCallBackMap.end(); i++)
      if(p->builtins.count(i->first) == 0)
	cout << "    " << i->first << endl;
  }

  void builtin_queue(void* ptr, string /*sCommand*/, string sParams)
  {
    vector<string> vs = ChopAndUnquoteString(sParams);
    if(vs.size() < 2)
      {
	cout << "? GUI_impl Internal queue command syntax: queue queue-name line-to-enqueue" << endl;
	return;
      }
    string &sQueueName = vs[0];
    sParams.erase(sParams.find(sQueueName), sQueueName.length());
	
    GUI_impl* pGUI = (GUI_impl*)ptr;
    pGUI->mmQueues[sQueueName].push_back(sParams);
  }

  void builtin_runqueue(void* ptr, string sCommand, string sParams)
  {
    GUI_impl* pGUI = (GUI_impl*)ptr;
    vector<string> vs = ChopAndUnquoteString(sParams);
    if(vs.size() != 1)
      {
	cout << "? GUI_impl Internal " << sCommand << " command syntax: runqueue queue-name " << endl;
	size_t nQueues = pGUI->mmQueues.size();
	    
	cout << "  Currently there are " << nQueues << " queues registered." << endl;
	if(nQueues > 0)
	  {
	    cout << "  They are: ";
	    for(map<string,vector<string> >::iterator it=pGUI->mmQueues.begin(); 
		it!=pGUI->mmQueues.end(); 
		it++)
	      cout << ((it==pGUI->mmQueues.begin())?"":", ") << it->first;
	    cout << endl;
	  }
	return;
      }
    string &sQueueName = vs[0];
    vector<string> &vQueue = pGUI->mmQueues[sQueueName];
    for(size_t i=0; i<vQueue.size(); i++)
      pGUI->ParseLine(vQueue[i]);
    if(sCommand=="runqueue")
      vQueue.clear();   // do not clear the queue if the command was runqueue_noclear!
  }

  int GUI_impl::parseArguments( const int argc, char * argv[], int start, const string prefix, const string execKeyword ){
    while(start < argc){
      string opt = argv[start];
      if(opt.size() > prefix.size() && opt.find(prefix) == 0){
	string name = opt.substr(2);
	if(start < argc-1){
	  string value = argv[start+1];
	  if(name == execKeyword){
	    LoadFile(value);
	  } else {
	    GV3::set_var(name, value);
	  }
	  start +=2;
	}
	else break;
      }
      else break;
    }
    return start;
  }

  void GUI_impl::RegisterBuiltin(string sCommandName, GUICallbackProc callback)
  {
    RegisterCommand(sCommandName, callback, this);
    builtins.insert(sCommandName);
  }

  GUI_impl::GUI_impl()
  {
    do_builtins();
  }

  GUI_impl::GUI_impl(GVars2*)
  {
    do_builtins();
  }

  void print_history(ostream &ost);
  void builtin_history(void* /*ptr*/, string /*sCommand*/, string /*sParams*/)
  {
    cout << "History: " << endl;
    print_history(cout);
  };

  void builtin_save_history(void* /*ptr*/, string /*sCommand*/, string sParams)
  {
    vector<string> v = ChopAndUnquoteString(sParams);
    if(v.size()!=1) 
      cout << "? GUI_impl internal savehistory command: need one param (filename)." << endl;
    else
      {
	ofstream ofs;
	ofs.open(v[0].c_str());
	if(!ofs.good())
	  cout << "? GUI_impl internal savehistory command: cannot open " << v[0] << " for write." << endl;
	else
	  {
	    print_history(ofs);
	    ofs.close();
	    cout << "  Saved to " << v[0] << endl;
	  }
      };
  };


  void GUI_impl::do_builtins()
  {
    RegisterBuiltin("shell", builtin_shell);
    RegisterBuiltin("ls", builtin_ls);
    RegisterBuiltin("ll", builtin_ll);
    RegisterBuiltin("try", builtin_try);
    RegisterBuiltin("exec", builtin_exec);
    RegisterBuiltin("echo", builtin_echo);
    RegisterBuiltin("if", builtin_if);
    RegisterBuiltin("ifnot", builtin_ifnot);
    RegisterBuiltin("ifeq", builtin_ifeq);
    RegisterBuiltin("toggle", builtin_toggle);
    RegisterBuiltin("set", builtin_set);
    RegisterBuiltin("history", builtin_history);
    RegisterBuiltin("savehistory", builtin_save_history);
    RegisterBuiltin("?", builtin_qmark);
    RegisterBuiltin("gvarlist", builtin_gvarlist);
    RegisterBuiltin("printvar", builtin_printvar);
    RegisterBuiltin("commandlist", builtin_commandlist);
    RegisterBuiltin("queue",  builtin_queue);
    RegisterBuiltin("runqueue", builtin_runqueue);
    RegisterBuiltin("runqueue_noclear", builtin_runqueue);
  };

  

  ///////////////////////////////////////
  ///////////////////////////////////////
  ////////     Parser/Readline stuff:
  ///////////////////////////////////////
  ///////////////////////////////////////

  void* GUI_impl::mpParserThread = NULL;
  

}



