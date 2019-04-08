#
# Makefile for misc devices that really don't fit anywhere else.
#
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

obj-$(CONFIG_VIRTUAL_KEY)	+= virtual_key/

ifeq ($(CONFIG_MTK_AH1883),y)
obj-y	+=  ah1883/
endif
ifeq ($(CONFIG_MTK_PIR),y)
obj-y	+=  pir/
endif
ifeq ($(CONFIG_MTK_IR_SWITCH),y)
obj-y	+=  ir_switch/
endif
