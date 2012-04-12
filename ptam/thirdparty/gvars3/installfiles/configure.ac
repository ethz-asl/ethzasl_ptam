AC_INIT(gvars3, 0.6)
AC_PROG_AWK
AC_LANG(C++)

AC_CHECK_HEADERS(iostream)

optionlist="gui readline toon"
options=""

#Host detection
AC_CANONICAL_HOST
if test "$host_vendor" = apple
then
	CPPFLAGS="$CPPFLAGS -I /sw/include -I/opt/local/include -I/usr/X11R6/include"
	LIBS="$LIBS -L/sw/lib -L/opt/local/lib"
	AC_MSG_NOTICE([Adding /sw to the build path.])
	AC_MSG_NOTICE([Adding /opt/local to the build path.])
	AC_SUBST(osx_hacks,yes)
fi

dnl APPEND(var, value)
dnl This appends vale to a shell variable var
define(APPEND, [$1="$$1 $2"])




dnl TEST_AND_SET_CXXFLAG(flag, [program], [run])
dnl
dnl This attempts to compile a and run program with a certain compiler flag.
dnl If no program is given, then the minimal C++ program is compiled, and 
dnl this tests just the validity of the compiler flag. 
dnl
dnl Taken from the libCVD configure.in
define([TEST_AND_SET_CXXFLAG],[
	if test "$3" == ""
	then
		AC_MSG_CHECKING([if compiler flag $1 works])	
	else
		AC_MSG_CHECKING([$3])	
	fi
	save_CXXFLAGS="$CXXFLAGS"
	APPEND(CXXFLAGS, [$1])

	m4_if([$2],[],[pushdef(prog, [int main(){}])], [pushdef(prog, [$2])])

	m4_if([$4],[run],
		[AC_RUN_IFELSE([prog], [cvd_conf_test=1],[cvd_conf_test=0], [cvd_conf_test=0])],
		[AC_COMPILE_IFELSE([prog], [cvd_conf_test=1],[cvd_conf_test=0])]
	)
	
	
	popdef([prog])

	if test $cvd_conf_test = 1
	then
		AC_MSG_RESULT(yes)
		ts_success=yes
	else
		AC_MSG_RESULT(no)
		CXXFLAGS="$save_CXXFLAGS"
		ts_success=no
	fi
])





TEST_AND_SET_CXXFLAG(-Wall)
TEST_AND_SET_CXXFLAG(-Wextra)






echo
echo --------------------------
echo Checking for options
echo --------------------------

AC_ARG_WITH(TooN, [  --with-TooN                                       ], [opt_toon_dir=$withval])

if test "$opt_toon_dir" != no
then
	#Check for TooN
	AC_MSG_CHECKING([for TooN])

	a=
	AC_COMPILE_IFELSE([#include <TooN/TooN.h>
int main(){TooN::Vector<2> v;return 0;}], [a=1]) 

	if test x$a = x1
	then
		AC_MSG_RESULT(yes)
		AC_DEFINE(GVARS3_HAVE_TOON)
		AC_SUBST(have_toon,yes)
		options="$options toon"
	else
		AC_MSG_RESULT(no)
	fi

	AC_MSG_CHECKING([for TooN-2])

	a=
	AC_COMPILE_IFELSE([#include <TooN/TooN.h>
int main(){TooN::Vector<2, float> v;return 0;}], [a=1]) 

	if test x$a = x1
	then
		AC_MSG_RESULT(yes)
		AC_DEFINE(GVARS3_HAVE_TOON2)
	else
		AC_MSG_RESULT(no)
	fi
fi

TEST_AND_SET_CXXFLAG(-pthread)

HLIBS="$LIBS"

################################################################################
################################################################################
################################################################################

AC_ARG_WITH(readline,   [  --without-readline      Disable use of libreadline], [with_readline="$withval"], [with_readline=yes])
if test "$with_readline" == "yes"
then
foo_LIBS="$LIBS"
for curse in "" ncurses curses
do
	LIBS="$foo_LIBS"
	if test x$curse != x
	then 
		AC_MSG_CHECKING($curse as a requirement for readline)
		AC_MSG_RESULT()
		AC_CHECK_LIB($curse, tputs)
	fi
		
	AC_CHECK_LIB(readline,rl_done,[],[])

	if test x$ac_cv_lib_readline_rl_done == xyes
	then
		AC_SUBST(have_readline,yes)
		options="$options readline"
		break
	else
		unset ac_cv_lib_readline_rl_done
	fi
done
fi
################################################################################
#
# Basic dialogue box functionality
#

echo
echo ------------------------------------------------
echo "Checking for widget libraries (provides GUI_...)"
echo ------------------------------------------------



#dnl check_GUI(fltk,FL/Fl.h,fltk)
AC_ARG_ENABLE(widgets,[  --disable-widgets])
enable_widgets=${enable_widgets:-yes}

AC_ARG_ENABLE(fltk2,[  --disable-fltk2])
AC_ARG_ENABLE(fltk,[  --disable-fltk])
AC_ARG_ENABLE(motif,[  --disable-motif])

enable_fltk2=${enable_fltk2:-$enable_widgets}
enable_fltk=${enable_fltk:-$enable_widgets}
enable_motif=${enable_motif:-$enable_widgets}

gui=none

if test $enable_fltk2 == yes && test $gui == none
then
	a=
	AC_PATH_PROG(a,fltk2-config)
	if test x$a != x
	then
		FLTK2_CPPFLAGS=`$a --cxxflags`
		FLTK2_LIBS=`$a --ldflags`
		CPPFLAGS="$FLTK2_CPPFLAGS $CPPFLAGS"
		LIBS="$FLTK2_LIBS $LIBS"
		AC_SUBST(guiobj, GUI_Fltk2)
		options="$options gui"
		gui=fltk2
	fi
fi

if test $gui == none && test $enable_fltk == yes
then
	AC_CHECK_HEADERS(FL/Fl.H, have_fltk_headers=yes)
	AC_CHECK_LIB(fltk, main, have_fltk_lib=yes)

	if test x$have_fltk_headers$have_fltk_lib == xyesyes
	then
		LIBS="$LIBS -lfltk"
		options="$options gui"
		gui=fltk
		AC_SUBST(guiobj, GUI_Fltk)
	fi
fi


if test $gui = none && test $enable_motif == yes
then
	AC_PATH_X
	foo_LIBS="$LIBS"
	foo_CPPFLAGS="$CPPFLAGS"
	test -n "$x_libraries" && LIBS="-L$x_libraries $LIBS"
	test -n "$x_includes" && CPPFLAGS="$CPPFLAGS -I$x_includes"

	AC_CHECK_HEADERS(Xm/Xm.h, have_motif_headers=yes)
	AC_CHECK_LIB(Xm, main, have_motif_lib=yes)
	if test x$have_motif_headers$have_motif_lib == xyesyes
	then
		LIBS="$LIBS -lXm -lXt -lX11"
		options="$options gui"
		gui=motif
		AC_SUBST(guiobj, GUI_Motif)
	else
		LIBS="$foo_LIBS"
		CPPFLAGS="$foo_CPPFLAGS"
	fi
fi

if test $gui == none
then
	AC_MSG_WARN(No GUI functionality enabled)
	AC_SUBST(guiobj, GUI_none)
fi

################################################################################
#
# The rest
#

AC_ARG_WITH(headless,   [  --without-headless      Do not build additional headless library])
if test "$with_headless" != "no"
then
	AC_SUBST(make_headless,yes)
fi


major=`echo $PACKAGE_VERSION | awk -F'\\\\.' '{print $1}'`
minor=`echo $PACKAGE_VERSION | awk -F'\\\\.' '{print $2}'`

AC_SUBST(major, $major)
AC_SUBST(minor, $minor)
AC_SUBST(HLIBS, $HLIBS)

changequote(,)
echo Options:
echo $options
echo
echo Missing options:
echo $options $optionlist |  $AWK 'BEGIN{RS=" "}{a[$1]++}END{for(i in a)if(a[i]==1)printf(i" "); print""}'
echo GUI support:
echo $gui
echo
changequote([,])


AC_CONFIG_HEADERS(gvars3/config.h)
AC_OUTPUT(Makefile)
