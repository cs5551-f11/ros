#
# For instalation. Change this to your settings.
#
INC_DIR = ../../../include
LIB_DIR = ../..
#
#  compiler
#
CC= cc
CFLAG= -O -pthread -I/usr/include/gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include -I/usr/include/libxml2 -I/usr/X11R6/include -I$(INC_DIR)
#
# For making the library
#
AR= ar
ARFLAGS= rs
RANLIB= 
#
#   products
#
LIB= ${LIB_DIR}/libAR.a
INCLUDE1= ${INC_DIR}/AR/config.h \
          ${INC_DIR}/AR/matrix.h
INCLUDE2= ${INC_DIR}/AR/param.h
INCLUDE3= ${INC_DIR}/AR/ar.h
#
#   compilation control
#
LIBOBJS1= ${LIB}(mAlloc.o)      ${LIB}(mFree.o)  \
	  ${LIB}(mAllocDup.o)   ${LIB}(mDup.o)   \
	  ${LIB}(mAllocTrans.o) ${LIB}(mTrans.o) \
          ${LIB}(mAllocMul.o)   ${LIB}(mMul.o)   \
	  ${LIB}(mAllocInv.o)   ${LIB}(mInv.o)   ${LIB}(mSelfInv.o) \
	  ${LIB}(mAllocUnit.o)  ${LIB}(mUnit.o)  \
	  ${LIB}(mDisp.o)       ${LIB}(mDet.o)   \
	  ${LIB}(mPCA.o)   \
	  ${LIB}(vAlloc.o)      ${LIB}(vDisp.o)  \
	  ${LIB}(vFree.o)       ${LIB}(vHouse.o) \
	  ${LIB}(vInnerP.o)     ${LIB}(vTridiag.o)

LIBOBJS2= ${LIB}(paramGet.o)        \
          ${LIB}(paramDecomp.o)     \
          ${LIB}(paramDistortion.o) \
          ${LIB}(paramChangeSize.o) \
          ${LIB}(paramFile.o)       \
          ${LIB}(paramDisp.o)

LIBOBJS3= ${LIB}(arDetectMarker.o) \
          ${LIB}(arGetTransMat.o) \
          ${LIB}(arGetTransMat2.o) \
          ${LIB}(arGetTransMat3.o) \
          ${LIB}(arGetTransMatCont.o) \
          ${LIB}(arLabeling.o) \
          ${LIB}(arDetectMarker2.o) \
          ${LIB}(arGetMarkerInfo.o) \
          ${LIB}(arGetCode.o) \
          ${LIB}(arUtil.o)


all:		${LIBOBJS1} ${LIBOBJS2} ${LIBOBJS3}

${LIBOBJS1}:	${INCLUDE1}
${LIBOBJS2}:	${INCLUDE1} ${INCLUDE2}
${LIBOBJS3}:	${INCLUDE1} ${INCLUDE2} ${INCLUDE3}

.c.a:
	${CC} -c ${CFLAG} $<
	${AR} ${ARFLAGS} $@ $*.o
	${RANLIB}
	rm -f $*.o

clean:
	rm -f *.o
	rm -f ${LIB}

allclean:
	rm -f *.o
	rm -f ${LIB}
	rm -f Makefile
