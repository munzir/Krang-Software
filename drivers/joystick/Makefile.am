ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

include_HEADERS = include/gamepad.h

bin_PROGRAMS = joyd

bin_SCRIPTS = scripts/joyd-logitech-f310

joyd_SOURCES = joyd.c js.c
joyd_LDADD = -lsns -lach -lblas -llapack
