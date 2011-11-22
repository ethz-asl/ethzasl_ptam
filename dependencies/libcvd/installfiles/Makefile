# Makefile.  Generated from Makefile.in by configure.

SHELL = /bin/sh
srcdir = .
top_srcdir = .

prefix = /home/slynen/Desktop/ptam_git/dependencies/libcvd
exec_prefix = ${prefix}
bindir = ${exec_prefix}/bin
sbindir = ${exec_prefix}/sbin
libexecdir = ${exec_prefix}/libexec
datadir = ${datarootdir}
sysconfdir = ${prefix}/etc
sharedstatedir = ${prefix}/com
localstatedir = ${prefix}/var
libdir = ${exec_prefix}/lib
infodir = ${datarootdir}/info
mandir = ${datarootdir}/man
includedir = ${prefix}/include
oldincludedir = /usr/include
arch= x86_64
datarootdir=${datarootdir}


CC = /usr/bin/gcc
CXX=/usr/bin/c++
LD=/usr/bin/c++
RANLIB=ranlib
CPPFLAGS = -I/home/slynen/Desktop/ptam_git/dependencies/TooN/include -I/home/slynen/Desktop/ptam_git/dependencies/libcvd/include
CFLAGS = -g -O2
CXXFLAGS = -D_REENTRANT -Wall -Wextra -pipe -ggdb -fPIC -mmmx -msse -msse -msse2 -msse3 -pthread
OFLAGS= -O3
LDFLAGS = 
LOADLIBES = -ltiff -ljpeg -lpng -llapack -lGLU -lGL -lrt  -L -lX11 -lXext -pthread
INSTALL = /usr/bin/install -c
AR=ar

pkgdatadir = $(datadir)/@PACKAGE@
pkglibdir = $(libdir)/@PACKAGE@
pkgincludedir = $(includedir)/@PACKAGE@

top_builddir = .

ACLOCAL = @ACLOCAL@
AUTOCONF = @AUTOCONF@
AUTOMAKE = @AUTOMAKE@
AUTOHEADER = @AUTOHEADER@

INSTALL = /usr/bin/install -c
INSTALL_PROGRAM = ${INSTALL} $(AM_INSTALL_PROGRAM_FLAGS)
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_SCRIPT = ${INSTALL}
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

DEBUGEXTRA=-D_GLIBCXX_DEBUG -DCVD_IMAGE_DEBUG

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

################################################################################
#
# CVD objects
#

CVD_OBJS=   \
			cvd_src/diskbuffer2.o                           \
			cvd_src/deinterlacebuffer.o                     \
			cvd_src/exceptions.o                            \
			cvd_src/image_io.o                              \
			cvd_src/bayer.o                                 \
			cvd_src/colourspace_convert.o                   \
			cvd_src/half_sample.o                           \
			cvd_src/draw.o                                  \
			cvd_src/yuv422.o                                \
			cvd_src/yuv420.o                                \
			pnm_src/pnm_grok.o                              \
			pnm_src/bmp.o                                   \
			pnm_src/fits.o                                  \
			pnm_src/fitswrite.o                             \
			pnm_src/save_postscript.o                       \
			pnm_src/text_write.o                            \
			pnm_src/text.o                                  \
			cvd_src/fast_corner.o                           \
			cvd_src/convolution.o                           \
			cvd_src/nonmax_suppression.o                    \
			cvd_src/timeddiskbuffer.o                       \
			cvd_src/videosource.o                           \
			cvd_src/connected_components.o                  \
			cvd_src/i686/yuv411_to_stuff_MMX_64.o cvd_src/noarch/convert_rgb_to_y.o cvd_src/i686/convolve_gaussian.o cvd_src/i686/gradient.o cvd_src/noarch/yuv422_wrapper.o cvd_src/i686/median_3x3.o cvd_src/i686/utility_float.o cvd_src/i686/utility_byte_differences.o cvd_src/i686/utility_double_int.o cvd_src/SSE2/two_thirds_sample.o cvd_src/fast/fast_7_detect.o cvd_src/fast/fast_7_score.o cvd_src/slower_corner_7.o cvd_src/fast/fast_8_detect.o cvd_src/fast/fast_8_score.o cvd_src/slower_corner_8.o cvd_src/fast/fast_9_detect.o cvd_src/fast/fast_9_score.o cvd_src/fast_corner_9_nonmax.o cvd_src/fast/fast_10_detect.o cvd_src/fast/fast_10_score.o cvd_src/fast/fast_11_detect.o cvd_src/fast/fast_11_score.o cvd_src/slower_corner_11.o cvd_src/fast/fast_12_detect.o cvd_src/fast/fast_12_score.o cvd_src/faster_corner_9.o cvd_src/faster_corner_10.o cvd_src/faster_corner_12.o cvd_src/videosource_nodvbuffer.o cvd_src/Linux/videosource_nov4l1buffer.o cvd_src/Linux/videosource_nov4lbuffer.o cvd_src/OSX/videosource_noqtbuffer.o cvd_src/posix/timer.o cvd_src/synchronized.o cvd_src/eventobject.o cvd_src/thread.o cvd_src/thread/runnable_batch.o cvd_src/noarch/posix_memalign.o cvd_src/videodisplay.o cvd_src/glwindow.o cvd_src/gltext.o pnm_src/jpeg.o pnm_src/tiff.o pnm_src/tiffwrite.o pnm_src/png.o cvd_src/videosource_novideofilebuffer.o cvd_src/globlist.o cvd_src/tensor_voting.o cvd_src/brezenham.o cvd_src/draw_toon.o 

################################################################################
#
# Shared objects
#

ifeq (,yes)
	soname=libcvd.dylib
	soname1=libcvd-0.dylib
	soname2=libcvd-0.8.dylib

	dsoname=libcvd_debug.dylib
	dsoname1=libcvd_debug-0.dylib
	dsoname2=libcvd_debug-0.8.dylib

	shared=-dynamiclib
else
	soname=libcvd.so
	soname1=libcvd.so.0
	soname2=libcvd.so.0.8

	dsoname=libcvd_debug.so
	dsoname1=libcvd_debug.so.0
	dsoname2=libcvd_debug.so.0.8

	set_soname=-Wl,-soname,libcvd.so.0
	set_dsoname=-Wl,-soname,libcvd_debug.so.0

	shared=-shared
endif

OBJS=$(CVD_OBJS)

DEBUG_OBJSA=$(OBJS:cvd_src%=debug/cvd_src%)
DEBUG_OBJS=$(DEBUG_OBJSA:pnm_src%=debug/pnm_src%)

sos= $(soname) $(soname1) $(soname2) 
dsos= $(dsoname) $(dsoname1) $(dsoname2) 


all: libcvd.a $(soname) progs $(DTARGETS) libcvd_debug.a $(dsoname)

$(soname): $(OBJS)
	rm -f $(sos)
	$(LD) $(shared) -o $(soname2) $(OBJS) $(LDFLAGS) $(LOADLIBES) $(sed_soname)
	ln -s $(soname2) $(soname1)
	ln -s $(soname1) $(soname)

$(dsoname): $(DEBUG_OBJS)
	rm -f $(dsos)
	$(LD) $(shared) -o $(dsoname2) $(DEBUG_OBJS) $(LDFLAGS) $(LOADLIBES) $(set_dsoname)
	ln -s $(dsoname2) $(dsoname1)
	ln -s $(dsoname1) $(dsoname)

libcvd.a: $(OBJS)
	$(AR) crvs libcvd.a $(OBJS)
	$(RANLIB) libcvd.a

libcvd_debug.a: $(DEBUG_OBJS)
	$(AR) crvs libcvd_debug.a $(DEBUG_OBJS)
	$(RANLIB) libcvd_debug.a

#Automatically deduce the debug directories
DEBUG_DIRS=$(shell find ???_src -type d -not -name 'CVS' | xargs -n1 -Ix  echo  debug/x )

#This the debug subdir all debug directories, so specifying a dependency on this
#will ensure that all subdirs get created
debug:$(DEBUG_DIRS)

#Multiple targets are allowed on the left, so all directories can be created in batch
#with a single rule
$(DEBUG_DIRS):
	mkdir -p $(DEBUG_DIRS)

# Before we compile the test programs we need to augment the
# search paths with the path to libcvd, and also add libcvd to the
# libraries to use
CPPFLAGS = -I. -I/home/slynen/Desktop/ptam_git/dependencies/TooN/include -I/home/slynen/Desktop/ptam_git/dependencies/libcvd/include
ifneq (,yes)
    LDFLAGS = -Wl,-R. 
endif
LDFLAGS =  -L.
LOADLIBES = -ltiff -ljpeg -lpng -llapack -lGLU -lGL -lrt  -L -lX11 -lXext -pthread

PROGS=progs/se3_exp progs/se3_ln progs/se3_pre_mul progs/se3_post_mul progs/se3_inv progs/calibrate progs/img_play progs/cvd_display_image progs/img_play_bw progs/img_play_deinterlace progs/video_play_source  
TESTPROGS=test/diskbuffer2 test/test_images test/font 

$(PROGS):$(soname)
$(TESTPROGS):$(soname)

testprogs: $(TESTPROGS)
progs: $(PROGS)


docs:
	doxygen Doxyfile

test-fast:test/fast_test
	test/fast_test test/fast_test_image/noise.pgm


#If the mandir is older than the lib, then remake. A bit of a hack
man: libcvd.a
	doxygen Doxyfile
	touch man


install: install-nodebug install-libs-debug

install-nodebug: install-progs install-headers install-libs 

install-progs:progs
	mkdir -p $(DESTDIR)$(bindir)
	[ "$(PROGS)" = "" ] || cp $(PROGS) $(DESTDIR)$(bindir)

install-headers:
	mkdir -p $(DESTDIR)$(includedir)
	cp -r cvd $(DESTDIR)$(includedir)

install-libs-so:$(soname)
	mkdir -p $(DESTDIR)$(libdir)
	cp $(soname2) $(DESTDIR)$(libdir)
	rm -f  $(libdir)/$(soname1) $(DESTDIR)$(libdir)/$(soname)
	ln -s  $(libdir)/$(soname2) $(DESTDIR)$(libdir)/$(soname1)
	ln -s  $(libdir)/$(soname1) $(DESTDIR)$(libdir)/$(soname)

install-libs-a:libcvd.a
	mkdir -p $(DESTDIR)$(libdir)
	cp libcvd.a $(DESTDIR)$(libdir)

install-libs:install-libs-so install-libs-a

install-libs-debug:$(dsoname) libcvd_debug.a
	mkdir -p $(DESTDIR)$(libdir)
	cp $(dsoname2) $(DESTDIR)$(libdir)
	rm -f  $(libdir)/$(dsoname1) $(DESTDIR)$(libdir)/$(dsoname)
	ln -s  $(libdir)/$(dsoname2) $(DESTDIR)$(libdir)/$(dsoname1)
	ln -s  $(libdir)/$(dsoname1) $(DESTDIR)$(libdir)/$(dsoname)
	cp libcvd_debug.a $(DESTDIR)$(libdir)




clean:
	/bin/rm -f tmp libcvd.a $(sos) libcvd_debug.a $(dsos)
	find . -name '*.o' | xargs rm -f
	find progs/ -perm +0100 -type f | xargs rm -f
	rm -rf html man

distclean: clean
	/bin/rm -f Makefile config.h config.status config.cache config.log cvd/config.h .deps config.dep_tmp
	/bin/rm -rf debug

.deps:
	rm -f .sourcefiles
	find . -name "*.cxx" -o -name "*.cc" -o -name "*.cpp" -o -name "*.c" -o -name "*.C"  | grep -v python > .sourcefiles
	rm -f .deps
	bash generate_dependencies.bash "-I. $(CPPFLAGS) $(CXXFLAGS)" > .deps

depend: .deps

.PHONY: ChangeLog 

ChangeLog:
	cvs log | gawk -f make/log_to_changelog.awk > ChangeLog

.PRECIOUS: %.o

%.o : %.cc
	$(CXX) $(OFLAGS) -I. $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(OFLAGS) -I. $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

%.o: %.cxx
	$(CXX) $(OFLAGS) -I. $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

%.o: %.C
	$(CXX) $(OFLAGS) -I. $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

%.o: %.s
	as -g -o $@ $<



debug/%.o : %.cc debug
	$(CXX) -I. $(CPPFLAGS) $(CXXFLAGS) $(DEBUGEXTRA) -c $< -o $@

debug/%.o: %.cpp debug
	$(CXX) -I. $(CPPFLAGS) $(CXXFLAGS) $(DEBUGEXTRA) -c $< -o $@

debug/%.o: %.cxx debug
	$(CXX) -I. $(CPPFLAGS) $(CXXFLAGS) $(DEBUGEXTRA) -c $< -o $@

debug/%.o: %.C debug
	$(CXX) -I. $(CPPFLAGS) $(CXXFLAGS) $(DEBUGEXTRA) -c $< -o $@

debug/%.o: %.s debug
	as -g -o $@ $<

%: %.o
	$(CXX) -o $@ $< $(LDFLAGS) -lcvd  $(LOADLIBES)

include .deps
