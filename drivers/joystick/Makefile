# Project Name
PROJECT := achjoystick

# Project Version
VERSION := 0.1

# Binary Files
BINFILES :=  joyd #snachd

# Library files
SHAREDLIBS :=

.PHONY: default clean

all: default

include /usr/share/make-common/common.1.mk

CFLAGS += -O0
CFLAGS += --std=gnu99

default: $(LIBFILES) $(BINFILES)

## BUILDING LIBRARIES: call with  $(call LINKLIB, name_of_lib, list of object files)

## BUILDING BINARIES: call with $(call LINKBIN, name_of_binary, object files, shared libs, static libs)
$(call LINKBIN, snachd, snachd.o, spnav X11 rt amino somatic ach protobuf-c stdc++ )
$(call LINKBIN, jachd, jachd.o js.o, rt amino somatic ach protobuf-c stdc++ )
$(call LINKBIN, jach_listen_and_print, jach_listen_and_print.o, rt amino somatic ach protobuf-c stdc++ )
$(call LINKBIN, joyd, joyd.o js.o, rt amino sns ach)

clean:
	rm -fr *.o $(BINFILES) $(LIBFILES) *.o .dep debian *.deb *.lzma
