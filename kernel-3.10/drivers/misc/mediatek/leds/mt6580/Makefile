include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_LEDS) := leds.o

# add by yangbo for leds
ifeq ($(CONFIG_MTK_FAN5405_SUPPORT),y)
ccflags-y += -DMTK_FAN5405_SUPPORT
endif
