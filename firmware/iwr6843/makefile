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
SOURCES    = l3_dump.c track_select.c compact_iq16.c live_selector.c
DEPENDS    = $(addprefix $(PLATFORM_OBJDIR)/, $(SOURCES:.c=.$(R4F_DEP_EXT)))
OBJECTS    = $(addprefix $(PLATFORM_OBJDIR)/, $(SOURCES:.c=.$(R4F_OBJ_EXT)))

.PHONY: all bin clean cleanApp help appRTSC

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

# --- flashable meta-image (MSS executable + radarss BSS firmware) ------------
# Same generator + arg order as the mmw demo `all` target, minus the DSS image
# (we are MSS-only). VERIFY: generateMetaImage.sh must accept a 4-arg (no-DSS)
# invocation; if it insists on a DSS image we pass a null/empty placeholder.
bin: all
	$(GENERATE_METAIMAGE) $(BIN) $(SHMEM_ALLOC) $(OUT) $(XWR68XX_RADARSS_IMAGE_BIN)
	@echo 'Built flashable $(BIN)'

clean:
	@$(DEL) $(OBJECTS) $(DEPENDS) $(MAP) $(OUT) $(BIN)
	@$(DEL) $(CONFIGPKG)
	@$(DEL) $(PLATFORM_OBJDIR)

# Same-geometry firmware iterations can reuse the generated RTSC/SYS/BIOS
# package. Variant switches must continue to use the full `clean` target.
cleanApp:
	@$(DEL) $(OBJECTS) $(DEPENDS) $(MAP) $(OUT) $(BIN)
	@$(DEL) $(PLATFORM_OBJDIR)

help:
	@echo '****************************************************************'
	@echo '* OpenFlight L3-dump firmware targets'
	@echo 'all   -> build the MSS executable (l3_dump_mss.xer4f)'
	@echo 'bin   -> build the flashable meta-image (l3_dump.bin)'
	@echo 'clean -> remove build artifacts'
	@echo 'cleanApp -> remove app objects but retain generated RTSC packages'
	@echo '****************************************************************'

-include $(DEPENDS)
