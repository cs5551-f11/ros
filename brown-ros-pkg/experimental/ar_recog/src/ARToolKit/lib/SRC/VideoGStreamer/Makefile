#
# For instalation. Change this to your settings.
#
INC_DIR = ../../../include
LIB_DIR = ../..
#
#  compiler
#
CC=cc
CFLAG= -O -pthread -I/usr/include/gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include -I/usr/include/libxml2 -I/usr/X11R6/include -I$(INC_DIR)
#
# For making the library
#
AR= ar
ARFLAGS= rs
#
#   products
#
LIB= ${LIB_DIR}/libARvideo.a
INCLUDE= ${INC_DIR}/AR/video.h
#
#   compilation control
#
LIBOBJS= ${LIB}(video.o)

all:		${LIBOBJS}

${LIBOBJS}:	${INCLUDE}

.c.a:
	${CC} -c ${CFLAG} $<
	${AR} ${ARFLAGS} $@ $*.o
	rm -f $*.o

clean:
	rm -f *.o
	rm -f ${LIB}

allclean:
	rm -f *.o
	rm -f ${LIB}
	rm -f Makefile
