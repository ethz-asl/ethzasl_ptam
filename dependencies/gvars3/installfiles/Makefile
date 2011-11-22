# Makefile.  Generated from Makefile.in by configure.

SHELL = /bin/sh
srcdir = .
top_srcdir = .

prefix = /home/slynen/Documents/projects/ros_ptam/dependencies/gvars3
exec_prefix = ${prefix}
bindir = ${exec_prefix}/bin
sbindir = ${exec_prefix}/sbin
libexecdir = ${exec_prefix}/libexec
datadir = ${datarootdir}
datarootdir = ${prefix}/share
sysconfdir = ${prefix}/etc
sharedstatedir = ${prefix}/com
localstatedir = ${prefix}/var
libdir = ${exec_prefix}/lib
infodir = ${datarootdir}/info
mandir = ${datarootdir}/man
includedir = ${prefix}/include
oldincludedir = /usr/include
arch= x86_64


CC = @CC@
CXX=/usr/bin/c++
CPPFLAGS = -I. -I/home/slynen/Documents/projects/ros_ptam/dependencies/TooN/include
CFLAGS = $(CPPFLAGS) @CFLAGS@
CXXFLAGS = $(CPPFLAGS) -g -O2 -Wall -Wextra -pthread
OFLAGS=@OFLAGS@
LDFLAGS = 
LIBS = -lreadline  -lm 
HLIBS =  -lm 
INSTALL = @INSTALL@
AR=ar

DESTDIR =

pkgdatadir = $(datadir)/@PACKAGE@
pkglibdir = $(libdir)/@PACKAGE@
pkgincludedir = $(includedir)/@PACKAGE@

top_builddir = .

ACLOCAL = @ACLOCAL@
AUTOCONF = @AUTOCONF@
AUTOMAKE = @AUTOMAKE@
AUTOHEADER = @AUTOHEADER@

INSTALL = @INSTALL@
INSTALL_PROGRAM = @INSTALL_PROGRAM@ $(AM_INSTALL_PROGRAM_FLAGS)
INSTALL_DATA = @INSTALL_DATA@
INSTALL_SCRIPT = @INSTALL_SCRIPT@
transform = s,x,x,

NORMAL_INSTALL = :
PRE_INSTALL = :
POST_INSTALL = :
NORMAL_UNINSTALL = :
PRE_UNINSTALL = :
POST_UNINSTALL = :
MAKEINFO = @MAKEINFO@
PACKAGE = @PACKAGE@
VERSION = @VERSION@

SUBDIRS =
ACLOCAL_M4 = $(top_srcdir)/aclocal.m4
mkinstalldirs = $(SHELL) $(top_srcdir)/mkinstalldirs
CONFIG_CLEAN_FILES =
DIST_COMMON =  README ChangeLog Makefile.am \
Makefile.in aclocal.m4 configure configure.ac install-sh missing \
mkinstalldirs


DISTFILES = $(DIST_COMMON) $(SOURCES) $(HEADERS) $(TEXINFOS) $(EXTRA_DIST)

TAR = gtar
GZIP_ENV = --best

HEADLESSOBJS=\
	src/gvars3.o			\
	src/GUI.o				\
	src/GStringUtil.o		\
	src/gvars2.o			\
	src/GUI_language.o		\
	src/serialize.o			\
	src/GUI_none.o			\
	src/GUI_impl_headless.o\
	src/inst.o				\

OBJS=\
	src/gvars3.o			\
	src/GUI.o				\
	src/GStringUtil.o		\
	src/gvars2.o			\
	src/GUI_language.o		\
	src/serialize.o			\
	src/GUI_none.o          \
	src/GUI_non_readline.o	\
	src/inst.o\

ifeq (yes,yes)
	OBJS+=src/GUI_readline.o
	OBJS+=src/GUI_impl_readline.o
	CPPFLAGS+=-DGUI_HAVE_READLINE
else
	OBJS+=src/GUI_impl_noreadline.o
endif

	#src/@readlineobj@.o     \
#G++ seems to run constructors in the reverse order
#so inst needs to be last.





ifeq (x86_64,x86_64)
    CPPFLAGS+=-fPIC
endif


ifeq (,yes)
	hsoname=libGVars3_headless.dylib
	hsoname1=libGVars3_headless-0.dylib
	hsoname2=libGVars3_headless-0.6.dylib
	 
	soname=libGVars3.dylib
	soname1=libGVars3-0.dylib
	soname2=libGVars3-0.6.dylib
	shared=-dynamiclib
else
	hsoname=libGVars3_headless.so
	hsoname1=libGVars3_headless.so.0
	hsoname2=libGVars3_headless.so.0.6

	soname=libGVars3.so
	soname1=libGVars3.so.0
	soname2=libGVars3.so.0.6
	shared=-shared
endif

archive=libGVars3.a
harchive=libGVars3_headless.a

sos= $(soname) $(soname1) $(soname2)
hsos= $(hsoname) $(hsoname1) $(hsoname2)

install_shared=install-shared-normal

ifeq (yes,yes)
	HEADLESS=$(harchive) $(hsoname)
	Harchive=$(harchive)
	install_shared+=install-shared-headless
endif

all: $(archive) $(soname) $(HEADLESS)

$(hsoname): $(OBJS)
	rm -f $(hsos)
	$(CXX) $(shared) -o $(hsoname2) $(HEADLESSOBJS) $(HLIBS)
	ln -s $(hsoname2) $(hsoname1)
	ln -s $(hsoname1) $(hsoname)

$(soname): $(OBJS)
	rm -f $(sos)
	$(CXX) $(shared) -o $(soname2) $(OBJS) $(LIBS)
	ln -s $(soname2) $(soname1)
	ln -s $(soname1) $(soname)

.PHONY: install-static install-headers install-shared-normal install-shared-headless

install:install-static $(install_shared) install-headers

install-static: $(archive) $(Harchive)
	mkdir -p $(DESTDIR)/$(libdir)
	cp $(archive) $(Harchive) $(DESTDIR)/$(libdir)

install-shared-normal: $(soname) 
	mkdir -p $(DESTDIR)/$(libdir)
	cp $(soname2) $(DESTDIR)/$(libdir)
	ln -fs  $(DESTDIR)/$(libdir)/$(soname2) $(DESTDIR)/$(libdir)/$(soname1)
	ln -fs  $(DESTDIR)/$(libdir)/$(soname1) $(DESTDIR)/$(libdir)/$(soname)
	
install-shared-headless: $(hsoname) 
	mkdir -p $(DESTDIR)/$(libdir)
	cp $(hsoname2) $(DESTDIR)/$(libdir)
	ln -fs  $(DESTDIR)/$(libdir)/$(hsoname2) $(DESTDIR)/$(libdir)/$(hsoname1)
	ln -fs  $(DESTDIR)/$(libdir)/$(hsoname1) $(DESTDIR)/$(libdir)/$(hsoname)
	
install-headers:
	mkdir -p $(DESTDIR)/$(includedir)
	cp -r gvars3 $(DESTDIR)/$(includedir)
	
$(harchive): $(HEADLESSOBJS)
	$(AR) crvs $(harchive) $(HEADLESSOBJS)

$(archive): $(OBJS)
	$(AR) crvs $(archive) $(OBJS)

clean:
	rm -f *.a src/*.o *.so *.so.? test *.o *.so.?.? main

main:  main.cc
	$(CXX) $(CXXFLAGS) -o main -lGVars3 main.cc
