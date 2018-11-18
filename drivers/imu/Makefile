# Project Name
PROJECT := imud


# Project Version 
VERSION := 0.1

# Binary Files
BINFILES :=  imud imu_listen_and_print

# Library files
SHAREDLIBS := ssdmu

.PHONY: default clean

all: default

include /usr/share/make-common/common.1.mk

CFLAGS += -O0 -Wno-conversion
CFLAGS += --std=gnu99

default: $(LIBFILES) $(BINFILES)

## BUILDING LIBRARIES: call with  $(call LINKLIB, name_of_lib, list of object files)
$(call LINKLIB, ssdmu, ssdmu.o)

## BUILDING BINARIES: call with $(call LINKBIN, name_of_binary, object files, shared libs, static libs)
$(call LINKBIN, imud, imud.o ssdmu.o, rt amino somatic ach ntcan protobuf-c stdc++ blas lapack)
$(call LINKBIN, imu_listen_and_print, imu_listen_and_print.o, rt amino somatic ach protobuf-c stdc++ blas lapack)

clean:
	rm -fr *.o $(BINFILES) $(LIBFILES) *.o .dep debian *.deb *.lzma
