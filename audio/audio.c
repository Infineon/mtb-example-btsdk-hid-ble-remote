/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * audio.c
 *
 * This file contains audio function for ble_remote
 *
 */

#ifdef SUPPORT_AUDIO
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "hidd_lib.h"
#include "app.h"

/////////////////////////////////////////////////////////////////////////////////
#define AUDIO_FIFO_CNT    4  // 30 in android_o_remote

#define AUDIO_START_DELAY_IN_MS         100    // wait for 100 ms to start send audio data
#define AUDIO_ADC_GAIN_IN_DB            30     // Audio ADC Gain in dB, When DRC is disabled 21 dB is recommended
#define AUDIO_MODE (audio.sendMsg ? WICED_HIDD_AUDIO_BUTTON_SEND_MSG : WICED_HIDD_AUDIO_BUTTON_SEND_PCM)

// Digital MIC port defines: PDM_CLK and PDM_DATA should defined in platform. If it is not defined, we will define it here
#ifndef PDM_CLK
#define PDM_CLK     WICED_P17
#define PDM_DATA    WICED_P15
#endif

typedef struct
{
    /// audio flags
    uint8_t sendMsg:1;                // send start/stop request to host to start/stop audio
    uint8_t autoStopOnly: 1;          // When set, audio always playing at fixed duration. It stops automatically with duration specified in auto_stop_in_ms expires.
                                      // When set, this option ignores buttonHoldToTalk setting. When clear, buttonHoldToTalk controls when to stop.
    uint8_t buttonHoldToTalk:1;       // This option takes effect only if auoStopOnly is 0.
                                      // When buttonHoldToTalk is set, audio is active while button is pressed down. Otherwise, button is start/stop toggle button.
    uint16_t auto_stop_in_ms;         // Automatically stop audio in ms. Use 0 to disable.
    wiced_timer_t mic_stop_command_pending_timer;

    HidEventUserDefine  voiceEvent;
#ifdef AUDIO_QUEUE_EVENT
    uint8_t packetInQueue;
    HidEventUserDefine  voiceCtrlEvent;
#endif
#ifdef SUPPORT_AUDIO_REG_RD_WR
    uint8_t codecSettingMsg_type;
    uint8_t codecSettingMsg_dataCnt;
    uint8_t codecSettingMsg_dataBuffer[6];
#endif

} tAudioState;

/*****************************************************************************
 * Application Audio related config
 ****************************************************************************/
hidd_microphone_enhanced_config_t blehid_audiocfg =
{
    //# audio enc type: 0=PCM, 1=mSBC, 2=OPUS CELT, 3=ADPCM
#ifdef ADPCM_ENCODER
    .audioEncType = WICED_HIDD_AUDIO_ENC_TYPE_ADPCM,
#else
 #ifdef CELT_ENCODER
    .audioEncType = WICED_HIDD_AUDIO_ENC_TYPE_CELT,
 #else
  #ifdef SBC_ENCODER
    .audioEncType = WICED_HIDD_AUDIO_ENC_TYPE_mSBC,
  #else
    .audioEncType = WICED_HIDD_AUDIO_ENC_TYPE_PCM,
  #endif
 #endif
#endif
    .drcSettings = {
        //# DRC settings
        .enable = 1,            // 1 Enable DRC, 0 Disable DRC
        .waitTime = 0x02EE,     // Wait time in mSec, 0x2EE = 750 mSec.
        .knee1 = 70,            // Knee 1, 68.5dB,       2660, in 1/2 dB steps.  10^((RSSI_target/2 + 30)/20).
        .knee2 = 85,            // Knee 2, 75dB,         5623, in 1/2 dB steps.  10^((RSSI_target/2 + 30)/20).
        .knee3 = 95,            // Knee 3, 81dB,        11220, in 1/2 dB steps.  10^((RSSI_target/2 + 30)/20).
        .attackTime = 0x03E8,   // Attack time in mSec.  0x03E8 = 1000 mSec
        .decayTime = 0x001F,    // Decay time in mSec.  0x001F = 31 mSec.
        .saturationLevel = 0x6800, // Saturation Level, 0x6800 = 26624.  This will be the max output level.
                   // The DRC will behave like an AGC when the DRC curve exceeds this amount.
                   // This value will be used when the pga gain is set lower than 18dB by the DRC loop.
    },
    //# DRC custom gain boost. Default value = 1000
    .custom_gain_boost = 1496,
    //# End of DRC settings

#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
    //#Anti Alias Audio Filter Coefficients.  Set index 0 to 0x00 0x00 for default filter settings
    .audioFilterData = {
        .audio_aux_filter_coef = {
            0x6D00,  //#index 0
            0xB5FF,  //#index 1
            0x23FF,  //#index 2
            0xE4FF,  //#index 3
            0x1B01,  //#index 4
            0x4E00,  //#index 5
            0x4EFE,  //#index 6
            0x10FF,  //#index 7
            0x3E02,  //#index 8
            0xEC01,  //#index 9
            0x2BFD,  //#index 10
            0x70FC,  //#index 11
            0x5C03,  //#index 12
            0x6606,  //#index 13
            0x36FC,  //#index 14
            0x87F3,  //#index 15
            0x1204,  //#index 16
            0x5D28,  //#index 17
            0xD53B,  //#index 18
        },
        .biQuadEqFilterCoeffs = {},

    /*# EQ Filter 1, 116 Coefficients(int16_t)
    #        1       2          3        4         5          6         7         8        9         10
    #    LSB  MSB LSB  MSB   LSB  MSB LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB
    #   --------- --------- --------- --------- --------- --------- --------- --------- --------- ---------
    #   To disable EQ filter, set the first two bytes below to 0.
    #   Customer should fill in the actually EQ coeff's based on specific test setup and HW */
        .eqFilter = {},
    }
#endif
};

//extern hidd_microphone_enhanced_config_t blehid_audiocfg;

// data defines
uint8_t             voice_rpt[AUDIO_BUFF_LEN] = {};
hidd_voice_report_t audioData[AUDIO_FIFO_CNT] = {};
uint16_t            dataCount[AUDIO_FIFO_CNT] = {};
hidd_microphone_config_t audioConfig = {
    .mic_codec = NULL,
    .audio_fifo = audioData,
    .data_count = dataCount,
    .audio_delay = AUDIO_START_DELAY_IN_MS,
    .fifo_count = AUDIO_FIFO_CNT,
    .audio_gain = AUDIO_ADC_GAIN_IN_DB,
#ifdef ANDROID_AUDIO
    .codec_sampling_freq = WICED_HIDD_CODEC_SAMP_FREQ_8K,
#else
    .codec_sampling_freq = WICED_HIDD_CODEC_SAMP_FREQ_16K,
#endif
    .enable = TRUE,
    .audio_boost = TRUE,               // not used
};

tAudioState audio = {
    .sendMsg = TRUE,
    .autoStopOnly = FALSE,
    .buttonHoldToTalk = TRUE,
#ifdef ANDROID_AUDIO
    .auto_stop_in_ms = 7000,           // max audio play time is 7 sec.
#else
    .auto_stop_in_ms = 0,              // no auto stop
#endif
};

#ifdef HID_AUDIO
uint8_t app_voice_ctrl_input_rpt[sizeof(hidd_voice_control_report_t)-1] = {0,};
uint8_t app_voice_ctrl_feature_rpt[sizeof(hidd_voice_control_report_t)-1] = {0,};
#endif

#ifdef AUDIO_QUEUE_EVENT
////////////////////////////////////////////////////////////////////////////////
/// Queue event
////////////////////////////////////////////////////////////////////////////////
void audio_queue_event(uint8_t type)
{
    audio.voiceCtrlEvent.eventInfo.eventType = type;
    app_queueEvent((app_queue_t *)&audio.voiceCtrlEvent);
}

////////////////////////////////////////////////////////////////////////////////
/// This function flushes all queued events and clears all reports.
////////////////////////////////////////////////////////////////////////////////
void audio_flush_data(void)
{
    //reset the counter that keeps track of voice events # in the event queue
    audio.eventInQueue = 0;
}
#endif


////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for mic_stop_command_pending_timer
////////////////////////////////////////////////////////////////////////////////
void mic_stop_command_pending_timeout( uint32_t arg )
{
    WICED_BT_TRACE("\nMIC Command Pending Timeout");
    if (audio_is_active())
    {
        audio_STOP_REQ();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// This function will be called from blehid_app_init() during start up.
/////////////////////////////////////////////////////////////////////////////////////////////
void audio_init(void (*activityDetectedPtr)())
{
#ifdef AUDIO_QUEUE_EVENT
    audio.evtQueueFuncPtr = ptr;
#endif
#ifdef SUPPORT_DIGITAL_MIC
    WICED_BT_TRACE("\ndigtalMIC clk=p%d, data=p%d", PDM_CLK, PDM_DATA);
    /* assign PDM pins */
    hidd_mic_assign_mic_pdm_pins(PDM_CLK, PDM_DATA); // pdm clk & data pin assignment
#endif

    /* assign Audio Enable gpio pin if it is used */
    //hidd_mic_assign_mic_en_pin(WICED_P37 gpio, 0);

    audio.voiceEvent.eventInfo.eventType = HID_EVENT_VOICE_DATA_AVAILABLE;
    hidd_mic_audio_config(&audioConfig);
    hidd_mic_audio_config_enhanced((uint8_t *)&blehid_audiocfg);
    wiced_init_timer( &audio.mic_stop_command_pending_timer, mic_stop_command_pending_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    hidd_mic_audio_init(activityDetectedPtr, NULL);

}

////////////////////////////////////////////////////////////////////////////////
/// This function is called when battery voltage drops below the configured threshold.
////////////////////////////////////////////////////////////////////////////////
void audio_shutdown(void)
{
    //stop audio
    hidd_mic_audio_stop();
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
static void audio_send_data(uint8_t * ptr, uint16_t len)
{
    memcpy(voice_rpt, ptr, len);
#ifdef ANDROID_AUDIO
    android_send_data(voice_rpt, len);
#else
    hidd_blelink_send_report(WICED_HIDD_VOICE_REPORT_ID, WICED_HID_REPORT_TYPE_INPUT, voice_rpt, len);
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// This function process the voice data event from queue
/////////////////////////////////////////////////////////////////////////////////
void audio_procEvtVoice()
{
    if (audio_is_active())
    {
        hidd_voice_report_t * audioPtr = (hidd_voice_report_t *) audio.voiceEvent.userDataPtr;
        uint8_t audio_outData[DECODE_BUFF_SIZE];
        uint8_t * dataPtr = audio_outData;

        uint16_t len = hidd_mic_audio_get_audio_out_data(audioPtr, dataPtr);

        // as long as the size is larger than audio MTU size, we fragement the data
        while (len >= AUDIO_MTU_SIZE)
        {
            audio_send_data(dataPtr, AUDIO_MTU_SIZE);
            dataPtr += AUDIO_MTU_SIZE;
            len -= AUDIO_MTU_SIZE;
        }
        // if any remainding residual data, finish it up
        if (len)
        {
            audio_send_data(dataPtr, len);
        }
    }

#ifdef AUDIO_QUEUE_EVENT
    if (audio.packetInQueue)
    {
        audio.packetInQueue--;
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles start audio request
///
/// if we send host to request stop, when issue stop message to host.
/// Otherwise, we just stop it.
/////////////////////////////////////////////////////////////////////////////////
void audio_start_request(void)
{
//    WICED_BT_TRACE("\nHID_EVENT_RC_MIC_START_REQ");
#ifdef ANDROID_AUDIO
    android_send_ctl(ATV_VOICE_SERVICE_START_SEARCH);
#else
    if (audio.sendMsg)
    {
        audio_voiceCtlSend(WICED_HIDD_RC_MIC_START_REQ);
    }
    else
    {
        audio_start();
    }
#endif
    if (audio.auto_stop_in_ms)
    {
        wiced_start_timer(&audio.mic_stop_command_pending_timer, audio.auto_stop_in_ms);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles stop audio request
///
/// if we send host to request stop, when issue stop message to host.
/// Otherwise, we just stop it.
/////////////////////////////////////////////////////////////////////////////////
void audio_stop_request(void)
{
//    WICED_BT_TRACE("\nHID_EVENT_RC_MIC_STOP_REQ");
#ifdef ANDROID_AUDIO
    android_send_ctl(ATV_VOICE_SERVICE_AUDIO_STOP);
    audio_stop();
#else
    if (audio.sendMsg)
    {
        audio_voiceCtlSend(WICED_HIDD_RC_MIC_STOP_REQ);
    }
    else
    {
        audio_stop();
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles the voice control message.
/// \param voice control report
/////////////////////////////////////////////////////////////////////////////////
void audio_procEvent(uint8_t eventType)
{
    switch( eventType )
    {
    case HID_EVENT_VOICE_DATA_AVAILABLE: //audio data
        audio_procEvtVoice();
        break;

 #ifdef SUPPORT_AUDIO_REG_RD_WR
    case HID_EVENT_AUDIO_CODEC_RD:
        audio_voiceReadCodecSetting();
        break;

    case HID_EVENT_AUDIO_CODEC_WT:
        audio_voiceWriteCodecSetting();
        break;
 #endif
 #ifdef HID_AUDIO
    case HID_EVENT_AUDIO_MODE:
        audio_voiceCtlSend(WICED_HIDD_RC_VOICEMODE_RD_ACK);
        break;
 #endif
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function physically starts audio
/////////////////////////////////////////////////////////////////////////////////
void audio_start(void)
{
    WICED_BT_TRACE("\naudio_start...");

    audio_flush_data();

#ifdef ANDROID_AUDIO
    android_send_ctl(ATV_VOICE_SERVICE_AUDIO_START);
#endif

    // audio is inactive, activate
    hidd_blelink_enable_poll_callback(WICED_FALSE); // disable polling and use audio interrupt
    hidd_mic_audio_set_active(WICED_TRUE);
    audio_pollActivityVoice();                      // kick off start poll
}

/////////////////////////////////////////////////////////////////////////////////
/// This function physically stops audio
/////////////////////////////////////////////////////////////////////////////////
void audio_stop(void)
{
    WICED_BT_TRACE("\naudio_stop");
    //stop audio codec
    if( audio_is_active() )
    {
        hidd_mic_audio_stop();
        WICED_BT_TRACE("\noverflow = %d", hidd_mic_audio_is_overflow());
    }
    wiced_stop_timer(&audio.mic_stop_command_pending_timer);
    hidd_blelink_enable_poll_callback(WICED_TRUE);      // re-enable back
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t audio_button(uint8_t key, wiced_bool_t down)
{
#ifdef ANDROID_AUDIO
    static wiced_bool_t audio_key_pressed = 0;
#endif

#ifdef ANDROID_AUDIO
    if (audio_key_pressed)
    {
        audio_key_pressed = 0;
        audio_START_REQ();
    }
#endif

    if (key == AUDIO_KEY_INDEX)
    {
        WICED_BT_TRACE("\naudio_button %s", down ? "down" : "up");

#ifdef ANDROID_AUDIO
        if (down)
        {
            audio_key_pressed = 1;
        }
#else // !ANDROID_AUDIO
        // button down
        if (down)
        {
            if (audio.buttonHoldToTalk || !audio_is_active())
            {
                audio_START_REQ();
            }
            // otherwise, audio button is toggle switch to start or to stop
            else if (!audio.autoStopOnly)
            {
                audio_STOP_REQ();
            }
        }
        // button release
        else
        {
            // if buttonHoldToTalk is true, it means audio is active while key is pressed down.
            // Since the button is released, we stop
            if (audio.buttonHoldToTalk && !audio.autoStopOnly)
            {
                audio_STOP_REQ();
            }
        }
        return TRUE;
#endif
    }
    return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function polls for voice activity and queues any events in the
/// FW event queue.
/////////////////////////////////////////////////////////////////////////////////
void audio_pollActivityVoice(void)
{
    //voice is active, poll and queue voice data
    if ( audio_is_active() )
    {
        while (hidd_mic_audio_poll_activity((void *)&audio.voiceEvent))
        {
            WICED_BT_TRACE("a");
            audio_DATA();
        }
    }
}

#ifdef HID_AUDIO
 #ifdef SUPPORT_AUDIO_REG_RD_WR
/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the voice control report(WICED_HIDD_RC_CODECSETTINGS_RD_ACK)
/////////////////////////////////////////////////////////////////////////////////
void audio_voiceReadCodecSetting(void)
{
    hidd_voice_control_report_t audioCodecRsp;
    memset(&audioCodecRsp, 0, sizeof(hidd_voice_control_report_t));
    audioCodecRsp.reportId = WICED_HIDD_VOICE_CTL_REPORT_ID;
    audioCodecRsp.format =  WICED_HIDD_RC_CODECSETTINGS_RD_ACK;
    audioCodecRsp.rsvd = AUDIO_MODE; // This information is saved in rxData
    if (hidd_mic_audio_read_codec_setting(&audioCodecRsp))
    {
        //set gatt attribute value here before sending the report
        memcpy(app_voice_ctrl_input_rpt, &audioCodecRsp.format, sizeof(hidd_voice_control_report_t) - sizeof(audioCodecRsp.reportId));

        hidd_blelink_send_report(WICED_HIDD_VOICE_CTL_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT,&audioCodecRsp.format,
                sizeof(hidd_voice_control_report_t) - sizeof(audioCodecRsp.reportId));
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the voice control report(WICED_HIDD_RC_CODECSETTINGS_WT_ACK)
/////////////////////////////////////////////////////////////////////////////////
void audio_voiceWriteCodecSetting(void)
{
    hidd_voice_control_report_t audioCodecRsp;
    memset(&audioCodecRsp, 0, sizeof(hidd_voice_control_report_t));
    audioCodecRsp.reportId = WICED_HIDD_VOICE_CTL_REPORT_ID;
    audioCodecRsp.format = WICED_HIDD_RC_CODECSETTINGS_WT_ACK;

    audioCodecRsp.rsvd = AUDIO_MODE;
    if (audio.codecSettingMsg_dataCnt > 0)
    {
        // First Write
        hidd_mic_audio_write_codec_setting(audioCodecRsp.rsvd, audio.codecSettingMsg_dataBuffer);

        // Now read the values from the codec registers
        if (hidd_mic_audio_read_codec_setting(&audioCodecRsp))
        {
            //set gatt attribute value here before sending the report
            memcpy(app_voice_ctrl_input_rpt, &audioCodecRsp.format, sizeof(hidd_voice_control_report_t) - sizeof(audioCodecRsp.reportId));

            hidd_blelink_send_report(WICED_HIDD_VOICE_CTL_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT,&audioCodecRsp.format,
                sizeof(hidd_voice_control_report_t) - sizeof(audioCodecRsp.reportId));
        }
    }
}
 #endif // SUPPORT_AUDIO_REG_RD_WR

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the voice control report(WICED_HIDD_RC_VOICEMODE_RD_ACK)
/////////////////////////////////////////////////////////////////////////////////
void audio_voiceCtlSend(uint8_t eventType)
{
    hidd_voice_control_report_t audioMsgReq;
    memset(&audioMsgReq, 0, sizeof(hidd_voice_control_report_t));
    audioMsgReq.reportId = WICED_HIDD_VOICE_CTL_REPORT_ID;
    audioMsgReq.format = eventType;
    audioMsgReq.rsvd = AUDIO_MODE; //put "audio mode" info in the reserved byte

    //set gatt attribute value here before sending the report
    memcpy(app_voice_ctrl_input_rpt, &audioMsgReq.format, sizeof(hidd_voice_control_report_t) - sizeof(audioMsgReq.reportId));
    hidd_blelink_send_report(WICED_HIDD_VOICE_CTL_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT,&audioMsgReq.format,
                        sizeof(hidd_voice_control_report_t) - sizeof(audioMsgReq.reportId));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles the voice control message.
/// \param voice control report
/////////////////////////////////////////////////////////////////////////////////
void audio_handleVoiceCtrlMsg(hidd_voice_control_report_t* voiceCtrlRpt)
{
    switch( voiceCtrlRpt->format )
    {
        case WICED_HIDD_MIC_START:
            audio_START();
            break;

        case WICED_HIDD_MIC_STOP:
            audio_STOP();
            break;

 #ifdef SUPPORT_AUDIO_REG_RD_WR
        case WICED_HIDD_RC_CODECSETTINGS_RD_REQ:
            audio_CODEC_RD();
            break;

        case WICED_HIDD_RC_CODECSETTINGS_WT_REQ:
            {
                uint8_t j;
                audio.codecSettingMsg_type     = voiceCtrlRpt->rsvd;
                audio.codecSettingMsg_dataCnt  = voiceCtrlRpt->dataCnt;
                for (j=0; j<voiceCtrlRpt->dataCnt; j++)
                {
                    audio.codecSettingMsg_dataBuffer[j] = voiceCtrlRpt->dataBuffer[j];
                }
                audio_CODEC_WR();
            }
            break;
 #endif // SUPPORT_AUDIO_REG_RD_WR
        case WICED_HIDD_RC_VOICEMODE_RD_REQ:
            audio_MODE_RD();
            break;

        case WICED_HIDD_SPK_START:
        case WICED_HIDD_SPK_STOP:
        case WICED_HIDD_PHONECALL_START:
        case WICED_HIDD_PHONECALL_STOP:
            default:
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function retrieves the voice control event, generates reports as
/// necessary
/////////////////////////////////////////////////////////////////////////////////
void audio_procEvtVoiceStartStop(wiced_hidd_app_event_queue_t * eventQ, uint8_t eventType)
{
    HidEventUserDefine *voicectrl_event;
    hidd_voice_control_report_t audioMsgReq;

    //if detecting continuous voice control event, only process once.
    while (((voicectrl_event = (HidEventUserDefine *)wiced_hidd_event_queue_get_current_element(eventQ))!=NULL) &&
           (voicectrl_event->eventInfo.eventType == eventType))
    {
        // Delete duplicated event
        wiced_hidd_event_queue_remove_current_element(eventQ);
    }

    switch (eventType) {
    case HID_EVENT_RC_MIC_START_REQ:
        audio_start_request();
        break;

    case HID_EVENT_RC_MIC_STOP_REQ:
        audio_stop_request();
        break;

    case HID_EVENT_MIC_START:
        audio_start();
        break;

    case HID_EVENT_MIC_STOP:
        audio_stop();
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// audio_event
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t audio_event(wiced_hidd_app_event_queue_t * eventQ)
{
    wiced_bool_t handled = FALSE;
    HidEventUserDefine *pEvt = (HidEventUserDefine *)wiced_hidd_event_queue_get_current_element(eventQ);

    if (pEvt != NULL)
    {
        switch (pEvt->eventInfo.eventType)
        {
        case HID_EVENT_RC_MIC_START_REQ:
        case HID_EVENT_RC_MIC_STOP_REQ:
        case HID_EVENT_MIC_START:
        case HID_EVENT_MIC_STOP:
            handled = TRUE;
            audio_procEvtVoiceStartStop(eventQ, pEvt->eventInfo.eventType);
            break;

        case HID_EVENT_VOICE_DATA_AVAILABLE: //audio data
#ifdef HID_AUDIO
 #ifdef SUPPORT_AUDIO_REG_RD_WR
        case HID_EVENT_AUDIO_CODEC_RD:
        case HID_EVENT_AUDIO_CODEC_WT:
 #endif
        case HID_EVENT_AUDIO_MODE:
#endif
            handled = TRUE;
            wiced_hidd_event_queue_remove_current_element(eventQ);
            audio_procEvent(pEvt->eventInfo.eventType);
            break;

        default:
            break;
        }
    }

    return handled;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t audio_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize)
{
    if ((reportType == WICED_HID_REPORT_TYPE_FEATURE) && (reportId == WICED_HIDD_VOICE_CTL_REPORT_ID))
    {
        hidd_voice_control_report_t* voiceCtrlRpt = (hidd_voice_control_report_t *)((uint8_t *)payload - 1);

        //save gatt attribute value here when receiving the report
        memset(app_voice_ctrl_feature_rpt, 0, sizeof(hidd_voice_control_report_t)-1);
        memcpy(app_voice_ctrl_feature_rpt, &(voiceCtrlRpt->format), MIN(payloadSize, (sizeof(hidd_voice_control_report_t)-1)));

        audio_handleVoiceCtrlMsg(voiceCtrlRpt);
        return TRUE;
    }
    return FALSE;
}
#endif // HID_AUDIO
#endif // SUPPORT_AUDIO
