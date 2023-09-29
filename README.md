# LE Remote Control Sample Application

## Overview

The LE Remote Control application is a single chip SoC compliant with HID over GATT Profile (HOGP). Supported features include key, microphone (voice over HOGP), Infrared Transmit (IR TX), and TouchPad.

During initialization the app registers with the LE stack, the AIROC&#8482; HID Device Library, and keyscan and external HW peripherals to receive various notifications including bonding complete, connection status change, peer GATT requests/commands, interrupts for key pressed/released, ADC audio, and Touchpad. Pressing any key will start LE advertising. When the device is successfully bonded, the app saves the bonded host's information in NVRAM. When the user presses/releases any key, a key report will be sent to the host. On connection up or battery level changed, a battery report will be sent to the host. When battery level is below shutdown voltage, device will critical shutdown. When the user presses and holds the microphone key, voice streaming starts until the user releases the microphone key. When the user presses and holds the power key, IR TX starts until the power key is released.

## Features demonstrated
- GATT database and device configuration initialization
- Registration with the LE stack for various events
- Sending HID reports to the host
- Processing write requests from the host
- Low power management
- Over the air firmware update (OTAFU)

# Instructions
To demonstrate the app, walk through the following steps:

1. Plug the AIROC&#8482; board or Remote Control HW into your computer.
2. Build and download the application.
3. If the download failed due to not able to detecting device, make sure the HCI COM port is not opened by other application (such as ClientControl) and repeat step 2.
   If it still fails, try the following recovery procedure:
     a. Place a jumper to bypass Serial Flash in remote control (or press and hold the Recovery button in AIROC&#8482; board), then power up the hardware (or press reset button in AIROC&#8482; board).
     b. Remove the jumper (or release the Recovery button in AIROC&#8482; board) so that download procedure below can write to Serial Flash. Go back to step 2.
4. Unplug the EVAL board or Remote Control HW from your computer (i.e. unplug the UART cable).
5. Power cycle the EVAL board or Remote Control HW.
6. Press any key to start LE advertising, then pair with a Host.
7. Once connected, it becomes the remote control of the Host.

In case what you have is the AIROC&#8482; EVAL board, you can either use fly wire to connect to GPIOs to simulate key press and release. Or use the ClientControl tool in the tools to simulate key press and release.

To use ClientControl tool + AIROC&#8482; EVAL board to simulate key press and release, launch ClientControl from MTB Tools section of the QuickPanel. Make sure you use "TESTING\_USING\_HCI=1" in the application makefile. The following steps shows how to establish communication between ClientControl and the device.

For 20835 devices:

1. Plug the hardware into your computer.
2. Build and download the application.
3. Run ClientControl.exe.
4. Choose 3M as the baudrate and select the serial port in ClientControl tool window.
5. Reset the device. (Press reset button or unplug/plug the USB cable). Within 2 seconds, before the device enters deep sleep, open the port. If HIDD tab is not activated, close the port and repeat step 5. Once the HID tab is activated, the HID buttons will become available.
6. Click on "Enter Pairing Mode" to start advertisement, then pair with a PC or Tablet.
7. Once connected, it becomes the remote of the PC or Tablet.

The key report can be sent by Clicking on the key buttons. For example, click on '1' button, the report should be:<br/>
01 00 00 1e 00 00 00 00 00<br/>
When the key is released, it should send all keys up:<br/>
01 00 00 00 00 00 00 00 00.

## Application Settings

- TESTING\_USING\_HCI
    - Use this option for testing with ClientControl. The ClientControl UI can be used to provide input. When this option is enabled, the device will not enter SDS/ePDS for power saving.

- OTA\_FW\_UPGRADE
    - Use this option for enabling firmware upgrade over the Air (OTA) capability. The peer tool applications in wiced\_btsdk\tools\btsdk-peer-apps-ota can be used to upgrade OTA firmware image. The OTA firmware image with extention *.ota.bin is created under the build folder.

 - OTA\_SEC\_FW\_UPGRADE
    - Use this option for secure OTA firmware upgrade. OTA\_FW\_UPGRADE option must be enabled for this option to take effect. The OTA firmware image with extention *.ota.bin.signed must be used for OTA firmware upgrade. Please follow the instruction in wiced\_btsdk\tools\btsdk-peer-apps-ota\readme.txt to create *.ota.bin.signed file image.

- LE\_LOCAL\_PRIVACY
    - When enabled, the device uses RPA (Random Private Address). When disabled, the device uses Public static address.

- AUTO\_RECONNECT
    - Use this option to enable auto reconnect. By enabling this option, the device will always stay connected. If it is disconnected, it wil try to reconnect until it is connected. This option should be used together with  DISCONNECTED\_ENDLESS\_ADV. When this option is enabled, the HID device will always try to maintain a connection with the paired HID host; therefore, if the link is down, it will continuously try to reconnect. To conserve power, it should allow entering SDS/ePDS while advertising; thus, the ENDLESS\_ADV option should be enabled; otherwise, it may drain the battery quickly if the host was not available to reconnect.

- ENDLESS\_ADV
    - Use this option to enable disconnected endless advertisement. When the link is disconnected, if AUTO\_RECONNECT is set, it will automatically start to advertise for reconnect. Otherwise, it will start to advertise after an external event, such as a key press event; i.e. the user will need to press a key to start reconnect. Once it starts to advertise, it will attempt to reconnect forever until it is connected. To conserve power, it allows SDS/ePDS while in low density advertisement with a long interval.

- SKIP\_PARAM\_UPDATE
    - Use this option to skip sending a link parameter update request. When this option is disabled, if the peer device (central) assigned link parameter is not within the device's preferred range, the device will send a request for the desired link parameter change. This option can be enabled to stop the device from sending the request and accept the given link parameter as is.

    - Background: In some OS (peer host), after the link is up, it continuously sends different parameters of LINK\_PARAM\_CHANGE over and over for some time. When the parameter is not in our device preferred range, the firmware was rejecting and renegotiating for new preferred parameters. It can lead up to endless and unnecessary overhead in link parameter changes. Instead of continuously rejecting the link parameter, by using this option, we accept peer requested link parameter as is and start a timer to send the final link parameter change request later when the peer host settles down in link parameter change.

- START\_ADV\_ON\_POWERUP
    - Use this option to start advertisements after power up (cold boot). By enabling this option, after power up, the device will automatically try to connect to the paired host if it is already paired. If it is not paired, it will enter discovery mode for pairing.

- ENABLE\_CONNECTED\_ADV
    - Use this option to allow advertisements for new host pairing while connected to a host.

- ENABLE\_EASY\_PAIR
    - Use this option to enable easy pairing. This is a proprietary method of pairing to bond connection between HID host and HID device. Instead sending advertising, the HID device will do scanning to find the preferred hosts in a list. Then select the strongest RSSI preferred host to bond the connection. To enable this function, the HID hosts must also need to enable EASY\_PAIRING method.

- ASSYMETRIC\_PERIPHERAL\_LATENCY
    - Use this option to enable asymmetric peripheral latency.
    - Background: In early days, some HID host devices will always reject HID peripheral's link parameter update request. Because of this, the HID device will end up consuming high power when peripheral latency was short. To work around this issue, we use Asymmetric Peripheral Latency method to save power by waking up only at multiple time of the communication anchor point. When this option is enabled:

        * We do not send LL\_CONNECTION\_PARAM\_REQ.
        * We simply start Asymmetric Peripheral Latency by waking up at multiple times of given peripheral latency.

    - Since this is not a standard protocol, we do not recommend enabling this option unless if it is necessary to save power to work around some HID hosts.

- ENABLE\_FINDME
    - Use this option to enable Find Me profile. To test Find Me Profile, please download CySmart app as host for testing. Mild/High alert will do slow/fast LED blink. The blinking times out in 30 sec.

- AUDIO=XXXX
    - Use this option to enable audio. Where XXXX is:

      -            leave empty to disable audio
      - OPUS       use OPUS CELT encoder
      - GOOGLE04   use ADPCM encoder, Google Voice 0.4
      - GOOGLE     use ADPCM encoder, Google Voice 1.0
      - MSBC       use mSBC encoder

Please note that mSBC or OPUS encoding for audio can be testing only if the host supports the encoding. There is no known off shelf products can be used for testing without customization.

- PDM
    - Use this option to enable/disable digital microphone (PDM=1/0). AUDIO option must be enabled for this option to take effect.
    - When enabling digital MIC in CYW920835M2EVB-01 platform, since the hardware pin is shared with LEDs, make sure SW4 is switched to DMIC and the compiler option LED is disabled.

- ENABLE\_TOUCHPAD
    - Use this option to enable touchpad functions. The option requires actual demo remote hardware to be functional.

- ENABLE\_IR
    - Use this option to enable IR functions. The option requires actual demo remote hardware to be functional. A sample dummy IR protocol is used. The developer will need to decide and implement the actual IR protocol.

## Key Matrix

When TESTING\_USING\_HCI is enabled, the key Matrix is disabled.

Key Matrix (ROW,COL)<br/>
Key index n = col*ROW + row<br/>
row = n % ROW<br/>
col = n / ROW<br/>

For example, to find the key index with row=1 and col=3 with key matrix 5x4, (ROW, COL)=(5, 4) key index n = col*ROW + row = 3*5 + 1 = 16

To find out row and col for index 16, row = 16 % 5 = 1, col = 16 / 3 = 3

20835:<br/>
Platform: CYW920835M2EVB-01,CYBLE-343072-EVAL-M2B,CYBLE-333074-EVAL-M2B<br/>
Key Matrix (ROW,COL) = (5,4) = (P00..P04, P08..P11)

AUDIO Key Index     15 = (row=1, col=3) = (P00, P11) = (D2, A3)<br/>
IR Key Index        17 = (row=2, col=3) = (P02, P11) = (D4, A3)<br/>
Pairing Key Index   18 = (row=3, col=3) = (P03, P11) = (D5, A3)<br/>

When testing with keys using ClientControl with key buttons, the device should be paired with a separated PC that is not running ClientControl. The reason is because when clicking a key, '1', for example, the key '1' will be send to host immediately. At that moment, since ClientControl is the active window, the key will be sent to ClientControl application and get ingored. Thus, it cannot be be sent to the other application, Notepad, for example, that expecting the key.

## BTSTACK version

BTSDK AIROC&#8482; chips contain the embedded AIROC&#8482; Bluetooth&#174; stack, BTSTACK. Different chips use different versions of BTSTACK, so some assets may contain variant sets of files targeting the different versions in COMPONENT\_btstack\_vX (where X is the stack version). Applications automatically include the appropriate folder using the COMPONENTS make variable mechanism, and all BSPs declare which stack version should be used in the BSP .mk file, with a declaration such as:<br>
> COMPONENTS+=btstack\_v1<br>
or:<br>
> COMPONENTS+=btstack\_v3

## Common application settings

Application settings below are common for all BTSDK applications and can be configured via the makefile of the application or passed in via the command line.

##### BT\_DEVICE\_ADDRESS
> Set the BDA (Bluetooth&#174; Device Address) for your device. The address is 6 bytes, for example, 20819A10FFEE. By default, the SDK will set a BDA for your device by combining the 7 hex digit device ID with the last 5 hex digits of the host PC MAC address.

##### UART
> Set to the UART port you want to use to download the application. For example 'COM6' on Windows or '/dev/ttyWICED\_HCI\_UART0' on Linux or '/dev/tty.usbserial-000154' on macOS. By default, the SDK will auto-detect the port.

##### ENABLE_DEBUG
> For HW debugging, configure ENABLE\_DEBUG=1. See the document [AIROC&#8482;-Hardware-Debugging](https://infineon.github.io/btsdk-docs/BT-SDK/AIROC-Hardware-Debugging.pdf) for more information. This setting configures GPIO for SWD.<br>
>
   - CYW920819EVB-02/CYW920820EVB-02: SWD signals are shared with D4 and D5, see SW9 in schematics.
   - CYBT-213043-MESH/CYBT-213043-EVAL/CYBT-253059-EVAL: SWD signals are routed to P12=SWDCK and P13=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYBT-223058-EVAL/CYW920835M2EVB-01/CYBT-243053-EVAL/CYBLE-343072-EVAL-M2B/CYBLE-333074-EVAL-M2B/CYBLE-343072-MESH: SWD signals are routed to P02=SWDCK and P03=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYBT-263065-EVAL/CYBT-273063-EVAL: SWD signals are routed to P02=SWDCK and P04=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYBT-343026-EVAL/CYBT-353027-EVAL/CYBT-333047-EVAL: SWD signals are routed to P11=SWDCK and P15=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYBT-413055-EVAL/CYBT-413061-EVAL: SWD signals are routed to P16=SWDCK and P17=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYW989820EVB-01: SWDCK (P02) is routed to the J13 DEBUG connector, but not SWDIO. Add a wire from J10 pin 3 (PUART CTS) to J13 pin 2 to connect GPIO P10 to SWDIO.
   - CYW920719B2Q40EVB-01: PUART RX/TX signals are shared with SWDCK and SWDIO. Remove RX and TX jumpers on J10 when using SWD. PUART and SWD cannot be used simultaneously on this board unless these pins are changed from the default configuration.
   - CYW920721M2EVK-02/CYW920721M2EVB-03: The default setup uses P03 for SWDIO and P05 for SWDCK. Check the position of SW15 if using JLink with the DEBUG connector.
   - CYW920706WCDEVAL: SWD debugging requires fly-wire connections. The default setup P15 (J22 pin 3 or J24 pin 1) for SWDIO and P11 (J23 pin 5
    or J22 pin 4) for SWDCK.
   - CYW920736M2EVB-01: SWD hardware debugging requires fly-wire connections. The only option is using P14 for SWDCK and P15 for SWDIO. These route to Arduino header J2, A1 and A0. These can be fly-wired to Arduino header J4, D4 and D5. From there the signals connect to the KitProg3 SWD bridge. In addition, the debug macros (SETUP\_APP\_FOR\_DEBUG\_IF\_DEBUG\_ENABLED and BUSY\_WAIT\_TILL\_MANUAL\_CONTINUE\_IF\_DEBUG\_ENABLED) are placed in sparinit.c in code common to all applications for this device. Most applications for this device call bleprofile\_GPIOInit() in subsequent code, overwriting the SWD pin configuration. To use hardware debugging after the call to bleprofile\_GPIOInit(), place the debug macros in code after that call.
   - CYW943012B2EVK-01: SWD signals are shared with D4 and D5.
   - CYW920820M2EVB-01 & CYW920819M2EVB-01: The default setup uses P03 for SWDIO and P02 for SWDCK. Check the position of SW15 if using JLink with the DEBUG connector.
   - CYW989820M2EVB-01: SWD hardware debugging requires a fly-wire connection to use P14 for SWDIO. P2 is connected directly to SWDCK / ARD_D4. Fly-wire P14 / ARD_D8 on J3.10 to J4.3 / ARD_D5 to connect SWDIO.

   - SWD hardware debugging is not supported on the following:
      - CYW920721M2EVK-01
      - CYW920835REF-RCU-01
      - CYW9M2BASE-43012BT
      - CYBT-423054-EVAL
      - CYBT-423060-EVAL
      - CYBT-483056-EVAL
      - CYBT-483062-EVAL
      - CYW955572BTEVK-01
      - CYW943022BTEVK-01

##### DIRECT_LOAD
> BTSDK chips support downloading applications either to FLASH storage or to RAM storage. Some chips support only one or the other, and some chips support both.

> If a chip only supports one or the other, this variable is not applicable, applications will be downloaded to the appropriate storage supported by the device.

> If a chip supports both FLASH and RAM downloads, the default is to download to FLASH, and the DIRECT_LOAD make variable may be set to 1 in the application makefile (or in the command line make command) to override the default and download to RAM.

> Currently, the following chips support both FLASH and RAM download and can set DIRECT_LOAD=1 if desired:
>
   - CYW20835
   - CYW20706

## Building and downloading code examples

**Using the ModusToolbox&#8482; Eclipse IDE**

1. Install ModusToolbox&#8482; 2.4.1 (or higher).
2. In the ModusToolbox&#8482; Eclipse IDE, click the **New Application** link in the Quick Panel (or, use **File > New > ModusToolbox IDE Application**).
3. Pick your board for BTSDK under AIROC&#8482; Bluetooth&#174; BSPs.
4. Select the application in the IDE.
5. In the Quick Panel, select **Build** to build the application.
6. To program the board (download the application), select **Program** in the Launches section of the Quick Panel.

**Using command line**

1. Install ModusToolbox&#8482; 2.4.1 (or higher).
2. On Windows, use Cygwin from \ModusToolbox\tools_2.x\modus-shell\Cygwin.bat to build apps.
3. Use the tool 'project-creator-cli' under \ModusToolbox\tools_2.x\project-creator\ to create your application.<br/>
   > project-creator-cli --board-id (BSP) --app-id (appid) -d (dir) <br/>
   See 'project-creator-cli --help' for useful options to list all available BSPs, and all available apps per BSP.<br/>
   For example:<br/>
   > project-creator-cli --app-id mtb-example-btsdk-empty --board-id CYW920706WCDEVAL -d .<br/>
4. To build the app call make build. For example:<br/>
   > cd mtb-examples-btsdk-empty<br/>
   > make build<br/>
5. To program (download to) the board, call:<br/>
   > make qprogram<br/>
6. To build and program (download to) the board, call:<br/>
   > make program<br/><br>
   Note: make program = make build + make qprogram

If you have issues downloading to the board, follow the steps below:

- Press and hold the 'Recover' button on the board.
- Press and hold the 'Reset' button on the board.
- Release the 'Reset' button.
- After one second, release the 'Recover' button.

Note: this is only applicable to boards that download application images to FLASH storage. Boards that only support RAM download (DIRECT_LOAD) such as CYW9M2BASE-43012BT or CYW943022BTEVK-01 can be power cycled to boot from ROM.

## Over The Air (OTA) Firmware Upgrade
Applications that support OTA upgrade can be updated via the peer OTA app in:<br>
>\<Workspace Dir>\mtb\_shared\wiced\_btsdk\tools\btsdk-peer-apps-ota<br>

See the readme.txt file located in the above folder for instructions.<br>
To generate the OTA image for the app, configure OTA\_FW\_UPGRADE=1 in the app
makefile, or append OTA\_FW\_UPGRADE=1 to a build command line, for example:
> make PLATFORM=CYW920706WCDEVAL OTA\_FW\_UPGRADE=1 build<br>

This will the generate \<app>.bin file in the 'build' folder.

## SDK software features

- Dual-mode Bluetooth&#174; stack included in the ROM (BR/EDR and LE)
- Bluetooth&#174; stack and profile level APIs for embedded Bluetooth&#174; application development
- AIROC&#8482; HCI protocol to simplify host/MCU application development
- APIs and drivers to access on-board peripherals
- Bluetooth&#174; protocols include GAP, GATT, SMP, RFCOMM, SDP, AVDT/AVCT, LE Mesh
- LE and BR/EDR profile APIs, libraries, and sample apps
- Support for Over-The-Air (OTA) upgrade
- Device Configurator for creating custom pin mapping
- Bluetooth&#174; Configurator for creating LE GATT Database
- Peer apps based on Android, iOS, Windows, etc. for testing and reference
- Utilities for protocol tracing, manufacturing testing, etc.
- Documentation for APIs, datasheets, profiles, and features
- BR/EDR profiles: A2DP, AVRCP, HFP, HSP, HID, SPP, MAP, PBAP, OPP
- LE profiles: Mesh profiles, HOGP, ANP, BAP, HRP, FMP, IAS, ESP, LE COC
- Apple support: Apple Media Service (AMS), Apple Notification Center Service (ANCS), iBeacon, Homekit, iAP2
- Google support: Google Fast Pair Service (GFPS), Eddystone
- Amazon support: Alexa Mobile Accessories (AMA)

Note: this is a list of all features and profiles supported in BTSDK, but some AIROC&#8482; devices may only support a subset of this list.

## List of boards available for use with BTSDK

- [CYW20819A1 chip](https://github.com/Infineon/20819A1)
    - [CYW920819EVB-02](https://github.com/Infineon/TARGET_CYW920819EVB-02), [CYW920819M2EVB-01](https://github.com/Infineon/TARGET_CYW920819M2EVB-01), [CYBT-213043-MESH](https://github.com/Infineon/TARGET_CYBT-213043-MESH), [CYBT-213043-EVAL](https://github.com/Infineon/TARGET_CYBT-213043-EVAL), [CYBT-223058-EVAL](https://github.com/Infineon/TARGET_CYBT-223058-EVAL), [CYBT-263065-EVAL](https://github.com/Infineon/TARGET_CYBT-263065-EVAL), [CYBT-273063-EVAL](https://github.com/Infineon/TARGET_CYBT-273063-EVAL)
- [CYW20820A1 chip](https://github.com/Infineon/20820A1)
    - [CYW920820EVB-02](https://github.com/Infineon/TARGET_CYW920820EVB-02), [CYW989820M2EVB-01](https://github.com/Infineon/TARGET_CYW989820M2EVB-01), [CYW989820EVB-01](https://github.com/Infineon/TARGET_CYW989820EVB-01), [CYBT-243053-EVAL](https://github.com/Infineon/TARGET_CYBT-243053-EVAL), [CYBT-253059-EVAL](https://github.com/Infineon/TARGET_CYBT-253059-EVAL), [CYW920820M2EVB-01](https://github.com/Infineon/TARGET_CYW920820M2EVB-01)
- [CYW20721B2 chip](https://github.com/Infineon/20721B2)
    - [CYW920721M2EVK-01](https://github.com/Infineon/TARGET_CYW920721M2EVK-01), [CYW920721M2EVK-02](https://github.com/Infineon/TARGET_CYW920721M2EVK-02), [CYW920721M2EVB-03](https://github.com/Infineon/TARGET_CYW920721M2EVB-03), [CYBT-423060-EVAL](https://github.com/Infineon/TARGET_CYBT-423060-EVAL), [CYBT-483062-EVAL](https://github.com/Infineon/TARGET_CYBT-483062-EVAL), [CYBT-413061-EVAL](https://github.com/Infineon/TARGET_CYBT-413061-EVAL)
- [CYW20719B2 chip](https://github.com/Infineon/20719B2)
    - [CYW920719B2Q40EVB-01](https://github.com/Infineon/TARGET_CYW920719B2Q40EVB-01), [CYBT-423054-EVAL](https://github.com/Infineon/TARGET_CYBT-423054-EVAL), [CYBT-413055-EVAL](https://github.com/Infineon/TARGET_CYBT-413055-EVAL), [CYBT-483056-EVAL](https://github.com/Infineon/TARGET_CYBT-483056-EVAL)
- [CYW20706A2 chip](https://github.com/Infineon/20706A2)
    - [CYW920706WCDEVAL](https://github.com/Infineon/TARGET_CYW920706WCDEVAL), [CYBT-353027-EVAL](https://github.com/Infineon/TARGET_CYBT-353027-EVAL), [CYBT-343026-EVAL](https://github.com/Infineon/TARGET_CYBT-343026-EVAL), [CYBT-333047-EVAL](https://github.com/Infineon/TARGET_CYBT-333047-EVAL)
- [CYW20835B1 chip](https://github.com/Infineon/20835B1)
    - [CYW920835REF-RCU-01](https://github.com/Infineon/TARGET_CYW920835REF-RCU-01), [CYW920835M2EVB-01](https://github.com/Infineon/TARGET_CYW920835M2EVB-01), [CYBLE-343072-EVAL-M2B](https://github.com/Infineon/TARGET_CYBLE-343072-EVAL-M2B), [CYBLE-333074-EVAL-M2B](https://github.com/Infineon/TARGET_CYBLE-333074-EVAL-M2B), [CYBLE-343072-MESH](https://github.com/Infineon/TARGET_CYBLE-343072-MESH)
- [CYW43012C0 chip](https://github.com/Infineon/43012C0)
    - [CYW9M2BASE-43012BT](https://github.com/Infineon/TARGET_CYW9M2BASE-43012BT), [CYW943012BTEVK-01](https://github.com/Infineon/TARGET_CYW943012BTEVK-01)
- [CYW43022C1 chip](https://github.com/Infineon/43022C1)
    - [CYW943022BTEVK-01](https://github.com/Infineon/TARGET_CYW943022BTEVK-01)
- [CYW20736A1 chip](https://github.com/Infineon/20736A1)
    - [CYW920736M2EVB-01](https://github.com/Infineon/TARGET_CYW920736M2EVB-01)
- [CYW30739A0 chip](https://github.com/Infineon/30739A0)
    - [CYW930739M2EVB-01](https://github.com/Infineon/TARGET_CYW930739M2EVB-01)
- [CYW55572A1 chip](https://github.com/Infineon/55572A1)
    - [CYW955572BTEVK-01](https://github.com/Infineon/TARGET_CYW955572BTEVK-01)


## Folder structure

All BTSDK code examples need the 'mtb\_shared\wiced\_btsdk' folder to build and test the apps. 'wiced\_btsdk' includes the 'dev-kit' and 'tools' folders. The contents of the 'wiced\_btsdk' folder will be automatically populated incrementally as needed by the application being used.

**dev-kit**

This folder contains the files that are needed to build the embedded Bluetooth&#174; apps.

* baselib: Files for chips supported by BTSDK. For example CYW20819, CYW20719, CYW20706, etc.

* bsp: Files for BSPs (platforms) supported by BTSDK. For example CYW920819EVB-02, CYW920706WCDEVAL etc.

* btsdk-include: Common header files needed by all apps and libraries.

* btsdk-tools: Build tools needed by BTSDK.

* libraries: Profile libraries used by BTSDK apps such as audio, LE, HID, etc.

**tools**

This folder contains tools and utilities need to test the embedded Bluetooth&#174; apps.

* btsdk-host-apps-bt-ble: Host apps (Client Control) for LE and BR/EDR embedded apps, demonstrates the use of AIROC&#8482; HCI protocol to control embedded apps.

* btsdk-host-peer-apps-mesh: Host apps (Client Control) and Peer apps for embedded Mesh apps, demonstrates the use of AIROC&#8482; HCI protocol to control embedded apps, and configuration and provisioning from peer devices.

* btsdk-peer-apps-ble: Peer apps for embedded LE apps.

* btsdk-peer-apps-ota: Peer apps for embedded apps that support Over The Air Firmware Upgrade.

* btsdk-utils: Utilities used in BTSDK such as BTSpy, wmbt, and ecdsa256.

See README.md in the sub-folders for more information.

## Software Tools
The following tool applications are installed on your computer either with ModusToolbox&#8482;, or by creating an application in the workspace that can use the tool.

**BTSpy:**<br>
>   BTSpy is a trace viewer utility that can be used with AIROC&#8482; Bluetooth&#174; platforms to
    view protocol and application trace messages from the embedded device. The
    utility is located in the folder below. For more information, see readme.txt in the same folder.<br>
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox&#8482; QuickPanel, or by right-clicking
    a project in the Project Explorer pane and selecting the ModusToolbox&#8482;
    context menu.<br>
    It is supported on Windows, Linux and macOS.<br>
    Location:  \<Workspace Dir>\wiced_btsdk\tools\btsdk-utils\BTSpy

**Bluetooth&#174; Classic and LE Profile Client Control:**<br>
>   This application emulates host MCU applications for LE and BR/EDR profiles.
    It demonstrates AIROC&#8482; Bluetooth&#174; APIs. The application communicates with embedded
    apps over the "WICED HCI UART" interface. The application is located in the folder
    below. For more information, see readme.txt in the same folder.<br>
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox&#8482; QuickPanel, or by right-clicking
    a project in the Project Explorer pane and selecting the ModusToolbox&#8482;
    context menu.<br>
    It is supported on Windows, Linux, and macOS.<br>
    Location:  \<Workspace Dir>\wiced\_btsdk\tools\btsdk-host-apps-bt-ble\client_control

**LE Mesh Client Control:**<br>
>   Similar to the above app, this application emulates host MCU applications
    for LE Mesh models. It can configure and provision mesh devices and create
    mesh networks. The application is located in the folder below. For more
    information, see readme.txt in the same folder.<br>
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox&#8482; QuickPanel (if a mesh-capable
    project is selected in the Project Explorer pane), or by right-clicking
    a mesh-capable project in the Project Explorer pane and selecting the
    ModusToolbox&#8482; context menu.<br>
    The full version is provided for Windows (VS\_ClientControl) supporting all
    Mesh models.<br>
    A limited version supporting only the Lighting model (QT\_ClientControl) is
    provided for Windows, Linux, and macOS.<br>
    Location:  \<Workspace Dir>\wiced_btsdk\tools\btsdk-host-peer-apps-mesh\host

**Peer apps:**<br>
>   Applications that run on Windows, iOS or Android and act as peer Bluetooth&#174; apps to
    demonstrate specific profiles or features, communicating with embedded apps
    over the air.<br>
    LE apps location:  \<Workspace Dir>\wiced\_btsdk\tools\btsdk-peer-apps-ble<br>
    LE Mesh apps location:  \<Workspace Dir>\wiced\_btsdk\tools\btsdk-host-peer-apps-mesh\peer<br>
    OTA apps location:  \<Workspace Dir>\wiced\_btsdk\tools\btsdk-peer-apps-ota

**Device Configurator:**<br>
>   Use this GUI tool to create source code for a custom pin mapping for your device. Run this tool
    from the Tools section of the ModusToolbox&#8482; QuickPanel, or by
    right-clicking a project in the Project Explorer pane and selecting the
    ModusToolbox&#8482; context menu.<br>
    It is supported on Windows, Linux and macOS.<br>
    Note: The pin mapping is based on wiced\_platform.h for your board.<br>
    Location:  \<Install Dir>\tools_2.x\device-configurator

Note: Not all BTSDK chips support Device Configurator. BSPs using the following devices do not currently support Device Configurator: CYW20706, CYW20736

**Bluetooth&#174; Configurator:**<br>
>   Use this GUI tool to create and configure the LE GATT Database and the BR/EDR SDP Database, generated as source code for your
    application.<br>
    Run this tool from the Tools section of the ModusToolbox&#8482; QuickPanel, or
    by right-clicking a project in the Project Explorer pane and selecting
    the ModusToolbox&#8482; context menu.<br>
    It is supported on Windows, Linux and macOS.<br>
    Location:  \<Install Dir>\tools_2.x\bt-configurator

## Tracing
To view application traces, there are 2 methods available. Note that the
application needs to configure the tracing options.<br>

1. "WICED Peripheral UART" - Open this port on your computer using a serial port
utility such as TeraTerm or PuTTY (usually using 115200 baud rate for non-Mesh apps, and 921600 for Mesh apps).<br>
2. "WICED HCI UART" - Open this port on your computer using the Client Control
application mentioned above (usually using 3M baud rate). Then run the BTSpy
utility mentioned above.

## Using BSPs (platforms)

BTSDK BSPs are located in the \mtb\_shared\wiced\_btsdk\dev-kit\bsp\ folder by default.

#### a. Selecting an alternative BSP

The application makefile has a default BSP. See "TARGET". The makefile also has a list of other BSPs supported by the application. See "SUPPORTED_TARGETS". To select an alternative BSP, use Library Manager from the Quick Panel to deselect the current BSP and select an alternate BSP. Then right-click the newly selected BSP and choose 'Set Active'.  This will automatically update TARGET in the application makefile.

#### b. Custom BSP

To create a custom BSP from a BSP template for BTSDK devices, see the following KBA article: [KBA238530](https://community.infineon.com/t5/Knowledge-Base-Articles/Create-custom-BTSDK-BSP-using-ModusToolbox-version-3-x-KBA238530/ta-p/479355)

## Using libraries

The libraries needed by the app can be found in in the mtb\_shared\wiced\_btsdk\dev-kit\libraries folder. To add an additional library to your application, launch the Library Manager from the Quick Panel to add a library. Then update the makefile variable "COMPONENTS" of your application to include the library. For example:<br/>
   COMPONENTS += fw\_upgrade\_lib


## Documentation

BTSDK API documentation is available [online](https://infineon.github.io/btsdk-docs/BT-SDK/index.html)

Note: For offline viewing, git clone the [documentation repo](https://github.com/Infineon/btsdk-docs)

BTSDK Technical Brief and Release Notes are available [online](https://community.infineon.com/t5/Bluetooth-SDK/bd-p/ModusToolboxBluetoothSDK)

<br>
<sup>The Bluetooth&#174; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.</sup>
