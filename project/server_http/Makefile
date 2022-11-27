CONTIKI_PROJECT=http-server
all: $(CONTIKI_PROJECT) 
TARGET = iotlab-m3

CONTIKI=../..

#linker optimizations
SMALL=1

APPS += er-coap
APPS += rest-engine

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
PROJECT_SOURCEFILES += httpd-simple.c
CFLAGS += -DUIP_CONF_TCP=1
CFLAGS += -DWEBSERVER=1

ifeq ($(PREFIX),)
 PREFIX = aaaa::1/64
endif

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include

ifeq ($(TARGET), minimal-net)
CFLAGS += -DHARD_CODED_ADDRESS=\"fdfd::10\"
${info INFO: er-example compiling with large buffers}
CFLAGS += -DUIP_CONF_BUFFER_SIZE=1300
CFLAGS += -DREST_MAX_CHUNK_SIZE=1024
CFLAGS += -DCOAP_MAX_HEADER_SIZE=176
CONTIKI_WITH_RPL=0
endif

$(CONTIKI)/tools/tunslip6:      $(CONTIKI)/tools/tunslip6.c
	(cd $(CONTIKI)/tools && $(MAKE) tunslip6)

connect-router: $(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 $(PREFIX)

connect-router-cooja:   $(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 -a 127.0.0.1 $(PREFIX)
