# Makefile.  Generated from Makefile.in by configure.

SHELL = /bin/sh
srcdir = .
top_srcdir = .

prefix = /usr/local
exec_prefix = ${prefix}
bindir = ${exec_prefix}/bin
sbindir = ${exec_prefix}/sbin
libexecdir = ${exec_prefix}/libexec
datadir = ${prefix}/share
sysconfdir = ${prefix}/etc
sharedstatedir = ${prefix}/com
localstatedir = ${prefix}/var
libdir = ${exec_prefix}/lib
infodir = ${prefix}/share/info
mandir = ${prefix}/share/man
includedir = ${prefix}/include
oldincludedir = /usr/include
arch= x86_64


CC = gcc
CXX=g++
LD=g++
RANLIB=ranlib
CPPFLAGS =  -INONE/include
CFLAGS = -g -O2
CXXFLAGS = -g  -Wall -Wextra -ggdb -fPIC -mmmx -msse -msse -msse2 -msse3 -pthread
OFLAGS= -O3
LDFLAGS = 
LOADLIBES = -ltiff -ljpeg -lpng -llapack -lGL -ldc1394  -L -lX11 -lXext -pthread
INSTALL = /usr/bin/install -c
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

CVD_OBJS=   cvd_src/cvd_timer.o                             \
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
			pnm_src/save_postscript.o                       \
			cvd_src/fast_corner.o                           \
			cvd_src/convolution.o                           \
			cvd_src/nonmax_suppression.o                    \
			cvd_src/timeddiskbuffer.o                       \
			cvd_src/videosource.o                           \
			cvd_src/i686/yuv411_to_stuff_MMX_64.o cvd_src/noarch/convert_rgb_to_y.o cvd_src/i686/convolve_gaussian.o cvd_src/i686/gradient.o cvd_src/noarch/yuv422_wrapper.o cvd_src/i686/median_3x3.o cvd_src/i686/utility_float.o cvd_src/i686/utility_byte_differences.o cvd_src/i686/utility_double_int.o cvd_src/fast/fast_7_detect.o cvd_src/fast/fast_7_score.o cvd_src/slower_corner_7.o cvd_src/fast/fast_8_detect.o cvd_src/fast/fast_8_score.o cvd_src/slower_corner_8.o cvd_src/fast/fast_9_detect.o cvd_src/fast/fast_9_score.o cvd_src/fast_corner_9_nonmax.o cvd_src/fast/fast_10_detect.o cvd_src/fast/fast_10_score.o cvd_src/fast/fast_11_detect.o cvd_src/fast/fast_11_score.o cvd_src/slower_corner_11.o cvd_src/fast/fast_12_detect.o cvd_src/fast/fast_12_score.o cvd_src/faster_corner_9.o cvd_src/faster_corner_10.o cvd_src/faster_corner_12.o cvd_src/Linux/dvbuffer3_dc1394v2.o cvd_src/synchronized.o cvd_src/eventobject.o cvd_src/thread.o cvd_src/thread/runnable_batch.o cvd_src/noarch/posix_memalign.o cvd_src/videodisplay.o cvd_src/glwindow.o cvd_src/gltext.o pnm_src/jpeg.o pnm_src/tiff.o pnm_src/tiffwrite.o pnm_src/png.o cvd_src/globlist.o 

################################################################################
#
# Shared objects
#

ifeq (,yes)
	soname=libcvd.dylib
	soname1=libcvd-0.dylib
	soname2=libcvd-0.7.dylib

	dsoname=libcvd_debug.dylib
	dsoname1=libcvd_debug-0.dylib
	dsoname2=libcvd_debug-0.7.dylib

	shared=-dynamiclib
else
	soname=libcvd.so
	soname1=libcvd.so.0
	soname2=libcvd.so.0.7

	dsoname=libcvd_debug.so
	dsoname1=libcvd_debug.so.0
	dsoname2=libcvd_debug.so.0.7

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
	$(LD) $(shared) -o $(soname2) $(OBJS) $(LDFLAGS) $(LOADLIBES)
	ln -s $(soname2) $(soname1)
	ln -s $(soname1) $(soname)

$(dsoname): $(DEBUG_OBJS)
	rm -f $(dsos)
	$(LD) $(shared) -o $(dsoname2) $(DEBUG_OBJS) $(LDFLAGS) $(LOADLIBES)
	ln -s $(dsoname2) $(dsoname1)
	ln -s $(dsoname1) $(dsoname)

libcvd.a: $(OBJS)
	$(AR) crvs libcvd.a $(OBJS)
	$(RANLIB) libcvd.a

libcvd_debug.a: $(DEBUG_OBJS)
	$(AR) crvs libcvd_debug.a $(DEBUG_OBJS)
	$(RANLIB) libcvd_debug.a

debug:
	mkdir -p debug/cvd_src debug/cvd_src/IRIX debug/cvd_src/Linux debug/cvd_src/OSX debug/cvd_src/fast debug/cvd_src/i686 debug/cvd_src/noarch debug/pnm_src debug/cvd_src/thread debug/cvd_src/nothread


# Before we compile the test programs we need to augment the
# search paths with the path to libcvd, and also add libcvd to the
# libraries to use
CPPFLAGS = -I.  -INONE/include
ifneq (,yes)
    LDFLAGS = -Wl,-R. 
endif
LDFLAGS = -L. 
LOADLIBES = -ltiff -ljpeg -lpng -llapack -lGL -ldc1394  -L -lX11 -lXext -pthread

PROGS=progs/img_play progs/cvd_display_image progs/img_play_bw progs/img_play_deinterlace  progs/video_play_source
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


install: install-progs install-headers install-libs install-libs-debug

install-progs:progs
	mkdir -p $(bindir)
	[ "$(PROGS)" = "" ] || cp $(PROGS) $(bindir)

install-headers:
	mkdir -p $(includedir)
	cp -r cvd $(includedir)

install-libs:$(soname) libcvd.a
	mkdir -p $(libdir)
	cp $(soname2) $(libdir)
	rm -f  $(libdir)/$(soname1) $(libdir)/$(soname)
	ln -s  $(libdir)/$(soname2) $(libdir)/$(soname1)
	ln -s  $(libdir)/$(soname1) $(libdir)/$(soname)
	cp libcvd.a $(libdir)

install-libs-debug:$(dsoname) libcvd_debug.a
	mkdir -p $(libdir)
	cp $(dsoname2) $(libdir)
	rm -f  $(libdir)/$(dsoname1) $(libdir)/$(dsoname)
	ln -s  $(libdir)/$(dsoname2) $(libdir)/$(dsoname1)
	ln -s  $(libdir)/$(dsoname1) $(libdir)/$(dsoname)
	cp libcvd_debug.a $(libdir)




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
	find . -name "*.cxx" -o -name "*.cc" -o -name "*.cpp" -o -name "*.c" -o -name "*.C" > .sourcefiles
	rm -f .deps
	bash generate_dependencies.bash "-I. $(CPPFLAGS)" > .deps

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

debug/%.o: %.s
	as -g -o $@ $<

%: %.o
	$(CXX) -o $@ $< $(LDFLAGS) -lcvd  $(LOADLIBES)

include .deps
