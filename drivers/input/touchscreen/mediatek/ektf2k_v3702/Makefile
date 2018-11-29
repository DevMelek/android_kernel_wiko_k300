include $(srctree)/drivers/misc/mediatek/Makefile.custom

# $(error xxxxlogzhangxian)
# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ektf2k_v3702/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ektf2k_v3702/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y	+=  ektf2k_driver.o


