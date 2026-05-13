CROSS_COMPILE ?= arm-rockchip830-linux-uclibcgnueabihf-
CC             = $(CROSS_COMPILE)gcc
STRIP          = $(CROSS_COMPILE)strip

SRCDIR      = main
VL53SRCDIR  = components/STM32duino_VL53L8CH/src
MLXSRCDIR   = components/Adafruit_MLX90640
BUILDDIR    = build
TARGET      = sensor_hub

USE_TCP       ?= 0
USE_CAM       ?= 0
USE_CAM_SYNC  ?= 0
UVC_AUTOSTART ?= 0

FEATURE_FLAGS  = -DUSE_TCP_TRANSPORT=$(USE_TCP)
FEATURE_FLAGS += -DUSE_UVC_CAMERA=$(USE_CAM)
FEATURE_FLAGS += -DUSE_CAM_SYNC=$(USE_CAM_SYNC)
FEATURE_FLAGS += -DUVC_AUTOSTART=$(UVC_AUTOSTART)

MAIN_SRCS = \
	$(SRCDIR)/main.c            \
	$(SRCDIR)/hub_transport.c   \
	$(SRCDIR)/hub_control.c     \
	$(SRCDIR)/i2c_bus.c         \
	$(SRCDIR)/tof_vl53l8ch.c    \
	$(SRCDIR)/mlx90640_driver.c \
	$(SRCDIR)/camera_sync.c     \
	$(SRCDIR)/v4l2_camera.c		\
    $(SRCDIR)/fast_aec.c

VL53_SRCS = \
	$(VL53SRCDIR)/platform.c                             \
	$(VL53SRCDIR)/vl53lmz_api.c                          \
	$(VL53SRCDIR)/vl53lmz_plugin_cnh.c                   \
	$(VL53SRCDIR)/vl53lmz_plugin_detection_thresholds.c  \
	$(VL53SRCDIR)/vl53lmz_plugin_motion_indicator.c      \
	$(VL53SRCDIR)/vl53lmz_plugin_xtalk.c

MLX_SRCS = \
	$(MLXSRCDIR)/MLX90640_API.c       \
	$(MLXSRCDIR)/mlx90640_linux_hal.c

ALL_SRCS = $(MAIN_SRCS) $(VL53_SRCS) $(MLX_SRCS)
OBJS     = $(patsubst %.c,$(BUILDDIR)/%.o,$(ALL_SRCS))

CFLAGS  = -Wall -Wextra -O2
CFLAGS += -march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard
CFLAGS += -I$(SRCDIR)
CFLAGS += -I$(VL53SRCDIR)
CFLAGS += -Icomponents/STM32duino_VL53L8CH
CFLAGS += -I$(MLXSRCDIR)
CFLAGS += -I./libs_arm/include
CFLAGS += -D_GNU_SOURCE
CFLAGS += $(FEATURE_FLAGS)

LDFLAGS = -static
LIBS    = -lpthread -lm -lrt ./libs_arm/lib/libturbojpeg.a

.PHONY: all clean

all: $(BUILDDIR)/$(TARGET)

$(BUILDDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILDDIR)/$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(STRIP) $@
	@echo "Built: $@"

clean:
	rm -rf $(BUILDDIR)
