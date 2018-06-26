include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/himax_852xES/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/himax_852xES/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/himax_852xES/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  himax_852xES.o
obj-y	+=  himax_platform.o
