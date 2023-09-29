#
# Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# This software, including source code, documentation and related
# materials ("Software") is owned by Cypress Semiconductor Corporation
# or one of its affiliates ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products.  Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

#
# Basic Configuration
#
APPNAME=BLE_HID_Remote
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

# default target
TARGET=CYW920835M2EVB-01

SUPPORTED_TARGETS = \
  CYW920835M2EVB-01 \
  CYBLE-343072-EVAL-M2B \
  CYBLE-333074-EVAL-M2B

#
# Advanced Configuration
#
SOURCES=
INCLUDES=
DEFINES=
VFP_SELECT=
CFLAGS=
CXXFLAGS=
ASFLAGS=
LDFLAGS=
LDLIBS=
LINKER_SCRIPT=
PREBUILD=
POSTBUILD=
FEATURES=

#
# Define basic library COMPONENTS
#
COMPONENTS += bsp_design_modus
COMPONENTS += hidd_lib2

#######################################################################################
# App compile flag defaults
#
# All flags can be defined in command line. When it is not defined, the following default value is used.
# Set to 1 to enable, 0 to disable.
#

####################################################
# To use ClientControl to control the device via HCI_UART port, TESTING_USING_HCI flag must be turned
#
TESTING_USING_HCI_DEFAULT=1

####################################################
# SLEEP_ALLOWED
#  SLEEP_ALLOWED=0  Disable sleep function
#  SLEEP_ALLOWED=1  Allow sleep without shutdown
#  SLEEP_ALLOWED=2  Allow sleep with shutdown
#
ifneq ($(filter CYW920835M2EVB-01 CYBLE-343072-EVAL-M2B CYBLE-333074-EVAL-M2B,$(TARGET)),)
 SLEEP_ALLOWED_DEFAULT=1
else
 SLEEP_ALLOWED_DEFAULT=2
endif

####################################################
# LED
#  LED=0  Disable LED functions (Good for power consumption measurement)
#  LED=1  Use LED for status indication
#
LED_DEFAULT=1

####################################################
# Use OTA_FW_UPGRADE=1 to enable Over-the-air firmware upgrade functionality
# Use OTA_SEC_FW_UPGRADE=1 in the make target to use secure OTA procedure.
# OTA_SEC_FW_UPGRADE_DEFAULT takes effect only when OTA_FW_UPGRADE_DEFAULT=1
#
OTA_FW_UPGRADE_DEFAULT=0
OTA_SEC_FW_UPGRADE_DEFAULT=0

####################################################
# AUDIO=XXXX where XXXX is:
#
#            - leave empty to disable audio
#   OPUS     - use OPUS CELT encoder
#   GOOGLE04 - use ADPCM encoder, Google Voice 0.4
#   GOOGLE   - use ADPCM encoder, Google Voice 1.0
#   MSBC     - use mSBC encoder
#
# Use PDM=1 to enable digital microphone, PDM=0 to use analog microphone
#
AUDIO_DEFAULT=GOOGLE
PDM_DEFAULT=0

####################################################
# Link related flags

# Use AUTO_RECONNECT=1 to automatically reconnect when connection drops
AUTO_RECONNECT_DEFAULT=0

# Use SKIP_PARAM_UPDATE=1 to not request connection parameter update immediately when
# received LE conn param update complete event with non-preferred values
SKIP_PARAM_UPDATE_DEFAULT=1

# Use START_ADV_ON_POWERUP=1 to start advertising on power up. If paired it reconnect when power up.
START_ADV_ON_POWERUP_DEFAULT=0

# Use ENABLE_CONNECTED_ADV=1 to enable advertising while connected
ENABLE_CONNECTED_ADV_DEFAULT=0

# Use ENDLESS_ADV=1 to enable endless advertising. Otherwise, the advertisement expires in given period.
ENDLESS_ADV_DEFAULT=0

# Use ASSYMETRIC_PERIPHERAL_LATENCY=1 if central won't accept peripheral connection parameter update request.
ASSYMETRIC_PERIPHERAL_LATENCY_DEFAULT=0

# Use LE_LOCAL_PRIVACY=1 to advertise with Resolvable Private Address (RPA)
LE_LOCAL_PRIVACY_DEFAULT=0

####################################################
#Perpherals and other features

# Use ENABLE_IR=1 to enable IR support
ENABLE_IR_DEFAULT=0

# Use ENABLE_EASY_PAIR=1 to enable Easy Pair feature
ENABLE_EASY_PAIR_DEFAULT=0

# Use ENABLE_FINDME=1 to enable Find Me profile support
ENABLE_FINDME_DEFAULT=0

#######################################################################################
XIP?=xip
BT_DEVICE_ADDRESS?=default
UART?=AUTO
TESTING_USING_HCI ?= $(TESTING_USING_HCI_DEFAULT)
OTA_FW_UPGRADE ?= $(OTA_FW_UPGRADE_DEFAULT)
OTA_SEC_FW_UPGRADE ?= $(OTA_SEC_FW_UPGRADE_DEFAULT)
AUDIO ?= $(AUDIO_DEFAULT)
PDM ?= $(PDM_DEFAULT)
AUTO_RECONNECT ?= $(AUTO_RECONNECT_DEFAULT)
ENABLE_IR ?= $(ENABLE_IR_DEFAULT)
AUTO_RECONNECT ?= $(AUTO_RECONNECT_DEFAULT)
SKIP_PARAM_UPDATE ?= $(SKIP_PARAM_UPDATE_DEFAULT)
ENABLE_EASY_PAIR ?= $(ENABLE_EASY_PAIR_DEFAULT)
LE_LOCAL_PRIVACY ?= $(LE_LOCAL_PRIVACY_DEFAULT)
START_ADV_ON_POWERUP ?= $(START_ADV_ON_POWERUP_DEFAULT)
ENABLE_CONNECTED_ADV ?= $(ENABLE_CONNECTED_ADV_DEFAULT)
ENDLESS_ADV ?= $(ENDLESS_ADV_DEFAULT)
ENABLE_FINDME ?= $(ENABLE_FINDME_DEFAULT)
ASSYMETRIC_PERIPHERAL_LATENCY ?= $(ASSYMETRIC_PERIPHERAL_LATENCY_DEFAULT)
SLEEP_ALLOWED ?= $(SLEEP_ALLOWED_DEFAULT)
LED ?= $(LED_DEFAULT)

ifeq ($(PDM),1)
 ifneq ($(AUDIO),)
  ifneq ($(filter CYW920835M2EVB-01 CYBLE-343072-EVAL-M2B CYBLE-333074-EVAL-M2B,$(TARGET)),)
   ifneq ($(LED), 0)
     $(warning $(TARGET) base board DMIC is shared with LED pins. Enabling DMIC will disable LED)
     LED=0
   endif
  endif
 endif
 CY_APP_DEFINES += -DSUPPORT_DIGITAL_MIC
 MIC=Digital
else
 MIC=Analog
endif

#
# App defines
#
CY_APP_DEFINES += \
  -DSUPPORT_KEYSCAN \
  -DBATTERY_REPORT_SUPPORT \
  -DBLE_SUPPORT \
  -DSUPPORT_KEY_REPORT \
  -DWICED_BT_TRACE_ENABLE \
  -DLED_SUPPORT=$(LED) \
  -DSLEEP_ALLOWED=$(SLEEP_ALLOWED)

ifneq ($(TARGET), CYW955572BTEVK-01)
 CY_APP_PATCH_LIBS += wiced_hidd_lib.a
endif

#ifeq ($(TARGET), CYW920820REF-RM-01)
# CY_APP_DEFINES += -DREMOTE_PLATFORM
#endif

ifeq ($(FASTPAIR_ENABLE),1)
 CY_APP_DEFINES += -DFASTPAIR_ENABLE -DFASTPAIR_ACCOUNT_KEY_NUM=5
 COMPONENTS += gfps_provider
endif

ifeq ($(hci_dump), 1)
 CY_APP_DEFINES += -DHCI_TRACES_ENABLED
else
 ifeq ($(TESTING_USING_HCI),1)
  CY_APP_DEFINES += -DTESTING_USING_HCI
 endif
endif

ifeq ($(ASSYMETRIC_PERIPHERAL_LATENCY),1)
 CY_APP_DEFINES += -DASSYM_PERIPHERAL_LATENCY
endif

ifeq ($(SKIP_PARAM_UPDATE),1)
 CY_APP_DEFINES += -DSKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
endif

ifeq ($(ENDLESS_ADV),1)
 CY_APP_DEFINES += -DENDLESS_LE_ADVERTISING
endif

ifeq ($(TESTING_USING_HCI),1)
 CY_APP_DEFINES += -DTESTING_USING_HCI
endif

ifeq ($(AUDIO),OPUS)
 CY_APP_DEFINES += -DSUPPORT_AUDIO -DENABLE_ADC_AUDIO_ENHANCEMENTS -DCELT_ENCODER -DHID_AUDIO -DATT_MTU_SIZE_180
 # for *.mk to include CYWxxxxxx_OPUS_CELT.cgs instead of CYWxxxxxxx.cgs
 OPUS_CELT_ENCODER = 1
 $(info Audio OPUS encoding $(MIC) MIC)
else
 ifeq ($(AUDIO),GOOGLE04)
  CY_APP_DEFINES += -DSUPPORT_AUDIO -DENABLE_ADC_AUDIO_ENHANCEMENTS -DADPCM_ENCODER -DANDROID_AUDIO
  CY_APP_PATCH_LIBS += adpcm_lib.a
  $(info Google Voice 0.4 $(MIC) MIC)
 else
  ifeq ($(AUDIO),GOOGLE)
   CY_APP_DEFINES += -DSUPPORT_AUDIO -DENABLE_ADC_AUDIO_ENHANCEMENTS -DADPCM_ENCODER -DANDROID_AUDIO -DANDROID_AUDIO_1_0
   CY_APP_PATCH_LIBS += adpcm_lib.a
   $(info Google Voice 1.0 $(MIC) MIC)
  else
   ifeq ($(AUDIO),MSBC)
    CY_APP_DEFINES += -DSUPPORT_AUDIO -DENABLE_ADC_AUDIO_ENHANCEMENTS -DSBC_ENCODER -DHID_AUDIO -DATT_MTU_SIZE_180
    $(info Audio mSBC encoding $(MIC) MIC)
   else
    ifneq ($(AUDIO),)
     $(error AUDIO=xxxx, where xxxx must be 'OPUS', 'GOOGLE', 'GOOGLE04', or 'MSBC')
    else
     $(info Audio disabled)
    endif
   endif
  endif
 endif
endif

ifeq ($(AUTO_RECONNECT),1)
 CY_APP_DEFINES += -DAUTO_RECONNECT
endif

ifeq ($(ENABLE_EASY_PAIR),1)
 CY_APP_DEFINES += -DEASY_PAIR
endif

ifeq ($(START_ADV_ON_POWERUP),1)
 CY_APP_DEFINES += -DSTART_ADV_WHEN_POWERUP_NO_CONNECTED
endif

ifeq ($(ENABLE_CONNECTED_ADV),1)
 CY_APP_DEFINES += -DCONNECTED_ADVERTISING_SUPPORTED
endif

ifeq ($(LE_LOCAL_PRIVACY),1)
 CY_APP_DEFINES += -DLE_LOCAL_PRIVACY_SUPPORT
endif

ifeq ($(ENABLE_FINDME),1)
 CY_APP_DEFINES += -DSUPPORTING_FINDME
endif

ifeq ($(ENABLE_IR),1)
 CY_APP_DEFINES += -DSUPPORT_IR
endif

ifeq ($(OTA_FW_UPGRADE),1)
 # DEFINES
 CY_APP_DEFINES += -DOTA_FIRMWARE_UPGRADE
 CY_APP_DEFINES += -DDISABLED_PERIPHERAL_LATENCY_ONLY
 CY_APP_DEFINES += -DOTA_SKIP_CONN_PARAM_UPDATE
 ifeq ($(OTA_SEC_FW_UPGRADE), 1)
  CY_APP_DEFINES += -DOTA_SECURE_FIRMWARE_UPGRADE
 endif # OTA_SEC_FW_UPGRADE
 # COMPONENTS
 COMPONENTS += fw_upgrade_lib
else
 ifeq ($(OTA_SEC_FW_UPGRADE),1)
  $(error setting OTA_SEC_FW_UPGRADE=1 requires OTA_FW_UPGRADE also set to 1)
 endif # OTA_SEC_FW_UPGRADE
endif # OTA_FW_UPGRADE

################################################################################
# Paths
################################################################################

# Path (absolute or relative) to the project
CY_APP_PATH=.

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI><COMMIT><LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler (Default: GCC in the tools)
CY_COMPILER_PATH=

# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_* \
    $(CY_IDE_TOOLS_DIR))

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

# tools that can be launched with "make open CY_OPEN_TYPE=<tool>
CY_BT_APP_TOOLS=BTSpy ClientControl

-include internal.mk
ifeq ($(filter $(TARGET),$(SUPPORTED_TARGETS)),)
$(error TARGET $(TARGET) not supported for this application. Edit SUPPORTED_TARGETS in the code example makefile to add new BSPs)
endif
include $(CY_TOOLS_DIR)/make/start.mk
