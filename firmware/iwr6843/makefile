###################################################################################
# OpenFlight Stage-1c L3 burst-dump firmware  (IWR6843, MSS / R4F only)
#
# Out-of-tree build against the INSTALLED mmWave SDK. Run inside the iwr-sdk
# container with the repo mounted at /work:
#
#   source $MMWAVE_SDK_INSTALL_PATH/../scripts/unix/setenv.sh   # device=iwr68xx
#   cd /work/firmware/iwr6843
#   make            # -> l3_dump_mss.xer4f   (CCS-loadable / debug)
#   make bin        # -> l3_dump.bin         (flashable meta-image: MSS+radarss)
#
# To get more L3 for the ring buffer, raise the bank count on the make line:
#   make MMWAVE_L3RAM_NUM_BANK=12
###################################################################################

# Pulls in tools, flags, R4F_COMMON_* libs, the %.oer4f build rule,
# buildDirectories, PLATFORM_*, SHMEM_ALLOC, GENERATE_METAIMAGE, etc.
include $(MMWAVE_SDK_INSTALL_PATH)/ti/common/mmwave_sdk.mak

# mmwave_sdk.mak's first target (buildDirectories) would otherwise become the
# default goal, so a bare `make` builds nothing. Pin it to our app.
.DEFAULT_GOAL := all

# Variant builds override capture geometry (make clean first — same objects):
#   make L3_VARIANT_DEFS="--define=LOOPS=16 --define=RING_FRAMES=12"     # B
#   make L3_VARIANT_DEFS="--define=N_SAMPLES=64 --define=RING_FRAMES=12" # C
R4F_CFLAGS += $(L3_VARIANT_DEFS)

# What the board image leaves out to fit DATA_RAM (0x30000 B, shared by the
# 96 KB IQ16 scratch, the 32 KB heap, .data, .bss and the stacks). The code
# stays in the tree and the host build (firmware_host.py) keeps all of it, so
# the replay and the tests still run it. Put one back on the make line, e.g.
#   make L3_FEATURE_DEFS="--define=L3_BALL_HYPOTHESES=1 --define=L3_BALL_RECOVER=1"
# (make clean first; this replaces the whole list).
#   L3_BALL_HYPOTHESES=0  the ball-hypothesis search: off at run time until
#                         the recorded captures say otherwise (~1.6 KB)
#   L3_BALL_RECOVER=0     the post-impact target history and backward ball
#                         recovery (needs L3_BALL_HYPOTHESES)
#   L3_TRIG_TRACE_DEPTH=24 trigger raw-input trace, host 64 (~1.3 KB)
L3_FEATURE_DEFS ?= --define=L3_BALL_HYPOTHESES=0 --define=L3_BALL_RECOVER=0 --define=L3_TRIG_TRACE_DEPTH=24U
R4F_CFLAGS += $(L3_FEATURE_DEFS)

# Image identity for the CLI "stats version" sub-mode (see fw_version.h). The
# top-level firmware/Makefile stamps all three from firmware/VERSION, git and
# the clock; values must not contain spaces.
L3_FW_VERSION ?= 0.0.0-dev
L3_FW_GIT ?= unknown
L3_FW_BUILT ?= unknown
R4F_CFLAGS += --define=L3_FW_VERSION='"$(L3_FW_VERSION)"'
R4F_CFLAGS += --define=L3_FW_GIT='"$(L3_FW_GIT)"' --define=L3_FW_BUILT='"$(L3_FW_BUILT)"'

APPNAME = l3_dump

# Sources live alongside this makefile.
vpath %.c .

###################################################################################
# Libraries. R4F_COMMON_STD_LIB already provides osal/esm/soc; we add:
#   pinmux+uart+cli : dual-UART I/O (CLI trigger in, data burst out)
#   crc+mailbox+mmwavelink+mmwave : RF front-end config (self-config on boot)
#   dma+edma+adcbuf : raw ADCBUF -> L3 capture
#   hwa             : optional range-FFT smoke/prototype path
###################################################################################
L3_STD_LIBS = $(R4F_COMMON_STD_LIB)                                        \
        -llibpinmux_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)               \
        -llibuart_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)                 \
        -llibcli_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)                  \
        -llibcrc_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)                  \
        -llibdma_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)                  \
        -llibedma_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)                 \
        -llibadcbuf_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)               \
        -llibhwa_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)                  \
        -llibmailbox_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)              \
        -llibmmwavelink_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)           \
        -llibmmwave_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)

L3_LOC_LIBS = $(R4F_COMMON_LOC_LIB)                                        \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/pinmux/lib                 \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/uart/lib                   \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/cli/lib                      \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/crc/lib                    \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/dma/lib                    \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/edma/lib                   \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/adcbuf/lib                 \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/hwa/lib                    \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/mailbox/lib                \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwavelink/lib             \
        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwave/lib

CFG        = mss.cfg
CONFIGPKG  = mss_configPkg_$(MMWAVE_SDK_DEVICE_TYPE)
MAP        = $(APPNAME)_mss.map
OUT        = $(APPNAME)_mss.$(R4F_EXE_EXT)
BIN        = $(APPNAME).bin
APP_CMD    = mss_linker.cmd
SOURCES    = l3_dump.c track_select.c detect_queue.c capture_plan.c compact_iq16.c live_selector.c l3_text.c l3_frames.c l3_angle.c l3_observation.c l3_band.c l3_trigger.c l3_ball.c l3_club_track.c l3_track_kf.c l3_impact_fit.c l3_ball_anchor.c l3_ball_hyp.c l3_ball_recover.c l3_impact.c l3_leave.c l3_confirm.c l3_scan.c l3_shot.c l3_ball_track.c l3_ball_fit.c l3_launch.c l3_result.c l3_profile.c l3_adaptive.c l3_iq8.c l3_retain.c l3_iq16_stats.c l3_bin_score.c l3_channels.c l3_angle_queue.c l3_dsp_ipc.c l3_detect_core.c l3_timing.c l3_window.c
DEPENDS    = $(addprefix $(PLATFORM_OBJDIR)/, $(SOURCES:.c=.$(R4F_DEP_EXT)))
OBJECTS    = $(addprefix $(PLATFORM_OBJDIR)/, $(SOURCES:.c=.$(R4F_OBJ_EXT)))

# --- DSS (C674x) image, built by firmware/iwr6843/dss/makefile ---------------
# Task 2 of the on-chip solve plan: a minimal DSS image that boots alongside
# this MSS image, to prove the three-image meta-image build/link plumbing
# before any solve code is ported onto the DSP.
DSS_DIR = dss
DSS_OUT = $(DSS_DIR)/l3_dump_dss.$(C674_EXE_EXT)

.PHONY: all bin dssAll clean cleanApp help appRTSC

# --- SYS/BIOS (RTSC) configuration -------------------------------------------
appRTSC:
	@echo 'Configuring RTSC packages...'
	$(XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(R4F_XSFLAGS) -o $(CONFIGPKG) $(CFG)
	@echo 'Finished configuring packages'

# --- link the MSS executable -------------------------------------------------
all: BUILD_CONFIGPKG=$(CONFIGPKG)
all: R4F_CFLAGS += --cmd_file=$(BUILD_CONFIGPKG)/compiler.opt
all: buildDirectories appRTSC $(OBJECTS)
	$(R4F_LD) $(R4F_LDFLAGS) $(L3_LOC_LIBS) $(L3_STD_LIBS)                  \
	-l$(CONFIGPKG)/linker.cmd --map_file=$(MAP) $(OBJECTS)                  \
	$(PLATFORM_R4F_LINK_CMD) $(APP_CMD) $(R4F_LD_RTS_FLAGS) -o $(OUT)
	@echo '******************************************************************'
	@echo 'Built $(OUT)'
	@echo '******************************************************************'

# --- link the DSS executable (delegates to dss/makefile) ---------------------
dssAll:
	$(MAKE) -C $(DSS_DIR) all

# --- flashable meta-image (MSS executable + DSS executable + radarss BSS ----
# firmware). generateMetaImage.sh's documented argument order is:
#   <FLASHIMAGE> <SHMEM_ALLOC> <MSS_IMAGE_OUT> <BSS_IMAGE_BIN> <DSS_IMAGE_OUT>
# (scripts/unix/generateMetaImage.sh, header comment) -- each *_OUT/_BIN slot
# takes the keyword NULL if that core is not present. We previously omitted
# the trailing DSS_IMAGE_OUT arg entirely (rather than passing NULL) because
# we had no DSS image; now that dssAll produces one, it is appended as the
# 5th argument.
bin: all dssAll
	$(GENERATE_METAIMAGE) $(BIN) $(SHMEM_ALLOC) $(OUT) $(XWR68XX_RADARSS_IMAGE_BIN) $(DSS_OUT)
	@echo 'Built flashable $(BIN)'

clean:
	@$(DEL) $(OBJECTS) $(DEPENDS) $(MAP) $(OUT) $(BIN)
	@$(DEL) $(CONFIGPKG)
	@$(DEL) $(PLATFORM_OBJDIR)
	@$(MAKE) -C $(DSS_DIR) clean

# Same-geometry firmware iterations can reuse the generated RTSC/SYS/BIOS
# package. Variant switches must continue to use the full `clean` target.
cleanApp:
	@$(DEL) $(OBJECTS) $(DEPENDS) $(MAP) $(OUT) $(BIN)
	@$(DEL) $(PLATFORM_OBJDIR)
	@$(MAKE) -C $(DSS_DIR) cleanApp

help:
	@echo '****************************************************************'
	@echo '* OpenFlight L3-dump firmware targets'
	@echo 'all      -> build the MSS executable (l3_dump_mss.xer4f)'
	@echo 'dssAll   -> build the DSS executable (dss/l3_dump_dss.xe674)'
	@echo 'bin      -> build the flashable meta-image (l3_dump.bin): MSS+DSS+radarss'
	@echo 'clean -> remove build artifacts'
	@echo 'cleanApp -> remove app objects but retain generated RTSC packages'
	@echo '****************************************************************'

-include $(DEPENDS)
