#
## Makefile for the touchscreen drivers.
#
#
## Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_LGSIC_SW42902)           += touch_sw42902.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_SW42902)           += touch_sw42902_prd.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_SW42902)           += touch_sw42902_abt.o
