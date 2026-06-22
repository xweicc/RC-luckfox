
ifeq ($(APP_PARAM), )
    APP_PARAM:=../Makefile.param
    include $(APP_PARAM)
endif

export LC_ALL=C
SHELL:=/bin/bash

CURRENT_DIR := $(shell pwd)

PKG_NAME := rproxyc
PKG_BIN ?= out

CC := $(RK_APP_CROSS)-gcc
STRIP := $(RK_APP_CROSS)-strip

CFLAGS += -I./include -I./source -Wall -O2
LDFLAGS +=

SRCS := source/rproxyc.c source/timer.c

PKG_TARGET := rproxyc-build

all: $(PKG_TARGET)
	@echo "build $(PKG_NAME) done"

rproxyc-build:
	@mkdir -p $(PKG_BIN)/bin
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(PKG_BIN)/bin/rproxyc $(SRCS)
	$(call MAROC_COPY_PKG_TO_APP_OUTPUT, $(RK_APP_OUTPUT), $(PKG_BIN))

clean:
	@rm -rf $(PKG_BIN)
	@rm -f source/*.o

distclean: clean

info:
	@echo -e "$(C_YELLOW)-------------------------------------------------------------------------$(C_NORMAL)"
	@echo -e "PKG_NAME=$(PKG_NAME)"
	@echo -e "$(C_YELLOW)-------------------------------------------------------------------------$(C_NORMAL)"
