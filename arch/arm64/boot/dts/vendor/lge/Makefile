ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
# Per Platform
ifeq ($(CONFIG_MACH_KONA_TIMELM),y)
DTS_PROJECT := kona-timelm
endif

ifeq ($(CONFIG_MACH_KONA_TIMELM_LAO_LDU),y)
DTS_TARGET := kona-timelm_lao_ldu
endif

# Per Device
ifeq ($(CONFIG_MACH_KONA_TIMELM_LAO_COM),y)
DTS_TARGET := kona-timelm_lao_com
endif

ifeq ($(CONFIG_MACH_KONA_TIMELM_DCM_JP),y)
DTS_TARGET := kona-timelm_dcm_jp
endif

ifeq ($(CONFIG_MACH_KONA_TIMELM_SB_JP),y)
DTS_TARGET := kona-timelm_sb_jp
endif

ALL_DTS := $(notdir $(wildcard echo $(srctree)/$(src)/$(DTS_PROJECT)/$(DTS_TARGET)/*.dts))
dtbo-y += $(patsubst %.dts,%.dtbo,$(ALL_DTS))

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtbo,$(DTB_NAMES))
else
DTB_LIST := $(dtbo-y)
endif

always  := $(dtbo-y)

$(obj)/%.dtbo: $(src)/$(DTS_PROJECT)/$(DTS_TARGET)/%.dts FORCE
	$(call if_changed_dep,dtc)
	$(call if_changed,dtbo_verify)

clean-files := *.dtb
endif
