

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
LDFLAGS += -lpthread -lm

SRCS := source/remote_control.c source/ml307c.c source/audio.c source/log.c

# ALSA library (libasound)
RK_BUILDROOT_SYSROOT := $(shell ls -d $(CURDIR)/../../../sysdrv/source/buildroot/buildroot-*/output/host/arm-buildroot-linux-uclibcgnueabihf/sysroot 2>/dev/null | head -1)
ifneq ($(RK_BUILDROOT_SYSROOT),)
    CFLAGS += -I$(RK_BUILDROOT_SYSROOT)/usr/include
endif
RK_ROOTFS_LIB := $(RK_PROJECT_OUTPUT)/rootfs_uclibc_$(RK_CHIP)/usr/lib
LDFLAGS += -L$(RK_ROOTFS_LIB) -lasound

PKG_TARGET := remote_control-build

remote_control-build:
	@mkdir -p $(PKG_BIN)/bin
	@echo "========================================"
	@echo "Building remote_control (ALSA)"
	@echo "========================================"
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
