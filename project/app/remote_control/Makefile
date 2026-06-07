

ifeq ($(APP_PARAM), )
    APP_PARAM:=../Makefile.param
    include $(APP_PARAM)
endif

export LC_ALL=C
SHELL:=/bin/bash

CURRENT_DIR := $(shell pwd)

PKG_NAME := remote_control
PKG_BIN ?= out

CC := $(RK_APP_CROSS)-gcc
STRIP := $(RK_APP_CROSS)-strip

CFLAGS += -Wall -O2
LDFLAGS += -lpthread

SRCS := source/remote_control.c

PKG_TARGET := remote_control-build

all: $(PKG_TARGET)
	@echo "build $(PKG_NAME) done"

remote_control-build:
	@mkdir -p $(PKG_BIN)/bin
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(PKG_BIN)/bin/remote_control $(SRCS)
	$(call MAROC_COPY_PKG_TO_APP_OUTPUT, $(RK_APP_OUTPUT), $(PKG_BIN))

clean:
	@rm -rf $(PKG_BIN)
	@rm -f source/*.o

distclean: clean

info:
	@echo -e "$(C_YELLOW)-------------------------------------------------------------------------$(C_NORMAL)"
	@echo -e "PKG_NAME=$(PKG_NAME)"
	@echo -e "$(C_YELLOW)-------------------------------------------------------------------------$(C_NORMAL)"
