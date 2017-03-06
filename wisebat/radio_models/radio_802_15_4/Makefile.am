ACLOCAL_AMFLAGS=-I m4 

lib_LTLIBRARIES = libradio_802_15_4_868_bpsk.la libradio_802_15_4_902_bpsk.la libradio_802_15_4_2400_oqpsk.la 

libradio_802_15_4_868_bpsk_la_CFLAGS = $(CFLAGS) $(GLIB_FLAGS) $(GSL_FLAGS) -DBPSK_868 -I$(top_srcdir)/src -Wall

libradio_802_15_4_868_bpsk_la_SOURCES = 802_15_4.c

libradio_802_15_4_868_bpsk_la_LDFLAGS = -module

libradio_802_15_4_902_bpsk_la_CFLAGS = $(CFLAGS) $(GLIB_FLAGS) $(GSL_FLAGS) -DBPSK_902 -I$(top_srcdir)/src -Wall

libradio_802_15_4_902_bpsk_la_SOURCES = 802_15_4.c

libradio_802_15_4_902_bpsk_la_LDFLAGS = -module

libradio_802_15_4_2400_oqpsk_la_CFLAGS = $(CFLAGS) $(GLIB_FLAGS) $(GSL_FLAGS) -DOQPSK_2400 -I$(top_srcdir)/src -Wall

libradio_802_15_4_2400_oqpsk_la_SOURCES = 802_15_4.c

libradio_802_15_4_2400_oqpsk_la_LDFLAGS = -module
