# Project Name
PROJECT := filter

# Project Version 
VERSION := 20100318

# Binary Files
#BINFILES := ftread_test ati_ftd

# Library files
SHAREDLIBS := filter 

all: default

include /usr/share/make-common/common.1.mk

#CFLAGS += -O0 -Wno-conversion
CFLAGS += --std=gnu99 -fPIC
FFLAGS += -I/usr/include -fPIC

default: $(LIBFILES) $(BINFILES)

$(call LINKLIB, filter, filter.o kalman.o)

.PHONY: default clean doc

doc:
	doxygen

clean:
	rm -fr *.o $(BINFILES) $(LIBFILES) *.o .dep debian *.deb *.lzma


