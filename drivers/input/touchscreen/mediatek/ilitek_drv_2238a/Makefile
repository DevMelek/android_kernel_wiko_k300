#ccflags-y += -Wall
	
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ilitek_drv_2238a/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/inc/

obj-y += ilitek_drv_mtk.o
obj-y += ilitek_drv_iliproc.o
obj-y += ilitek_drv_main.o
obj-y += ilitek_drv_mp_test.o
obj-y += ilitek_drv_update.o

