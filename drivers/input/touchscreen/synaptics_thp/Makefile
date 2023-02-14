#
# Makefile for the Synaptics TCM touchscreen driver.
#

# Each configuration option enables a list of files.



TCM_CORE=tcm/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/synaptics_thp/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/synaptics_thp/tcm/

synaptics_tcm2-y := syna_tcm2.o

synaptics_tcm2-y += \
					$(TCM_CORE)synaptics_touchcom_core_v1.o \
					$(TCM_CORE)synaptics_touchcom_core_v2.o \
 					$(TCM_CORE)synaptics_touchcom_func_base.o \
 					$(TCM_CORE)synaptics_touchcom_func_touch.o \
					$(TCM_CORE)synaptics_touchcom_func_reflash.o \
					$(TCM_CORE)synaptics_touchcom_func_romboot.o

synaptics_tcm2-y += syna_tcm2_platform_spi.o
synaptics_tcm2-y += syna_tcm2_sysfs.o
synaptics_tcm2-y += syna_tcm2_testing.o
synaptics_tcm2-y += synaptics_common.o
synaptics_tcm2-y += touchpanel_proc.o
synaptics_tcm2-y += touch_comon_api/touch_comon_api.o
synaptics_tcm2-y += touchpanel_autotest/touchpanel_autotest.o
synaptics_tcm2-y += touchpanel_healthinfo/touchpanel_healthinfo.o
synaptics_tcm2-y += touchpanel_healthinfo/touchpanel_exception.o

obj-$(CONFIG_TOUCHSCREEN_SYNA_TCM2) += synaptics_tcm2.o
