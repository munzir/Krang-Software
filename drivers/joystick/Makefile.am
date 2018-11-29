ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

include_HEADERS = include/jachd.h include/js.h

bin_PROGRAMS = jachd

bin_SCRIPTS = scripts/joyd-logitech-f310

jachd_SOURCES = jachd.c js.c
jachd_LDADD = -lach -lblas -llapack -lrt -lsomatic -lamino
