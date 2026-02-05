CC ?= cc
AR ?= ar

BUILD_DIR := build/make
LIB_NAME := libhc-compress.a

INCLUDES := -Iinclude
CPPFLAGS := -DUNIT_TEST
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Wpedantic
LDFLAGS ?=
LDLIBS ?= -lm

LIB_SRCS := \
	src/ccsds123_core.c \
	src/ccsds123_utils.c \
	src/ccsds123_io.c \
	src/hc_compress_api.c

APP_SRCS := \
	src/ccsds123_cli.c

LIB_OBJS := $(LIB_SRCS:src/%.c=$(BUILD_DIR)/src/%.o)
APP_OBJS := $(APP_SRCS:src/%.c=$(BUILD_DIR)/src/%.o)

.PHONY: all clean

all: hc-compress

$(BUILD_DIR)/$(LIB_NAME): $(LIB_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

hc-compress: $(APP_OBJS) $(BUILD_DIR)/$(LIB_NAME)
	$(CC) $(LDFLAGS) -o $@ $(APP_OBJS) $(BUILD_DIR)/$(LIB_NAME) $(LDLIBS)

$(BUILD_DIR)/src/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) hc-compress
