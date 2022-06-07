/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
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
 * BLE Remote audio functions
 *
 * This file provides definitions and function prototypes for BLE remote control
 * device
 *
 */
#include "hidd_audio_atv.h"

#ifndef __SUPPORT_AUDIO_H__
#define __SUPPORT_AUDIO_H__

#ifdef SUPPORT_AUDIO
//#define AUDIO_QUEUE_EVENT                   // We don't use event queue for audio to save time and space
//#define SUPPORT_AUDIO_REG_RD_WR             // Most of hosts don't care codec registers. We disable codec read/write to save code space

 #define audio_is_active()  hidd_mic_audio_is_active()
 #define DECODE_BUFF_SIZE (HIDD_MIC_AUDIO_BUFFER_SIZE*2+1)
 #define AUDIO_DATA_SIZE HIDD_MIC_AUDIO_BUFFER_SIZE
 #ifdef ATT_MTU_SIZE_180
  #define AUDIO_MTU_SIZE 180
 #else
  #ifdef ANDROID_AUDIO_1_0
   #define AUDIO_MTU_SIZE 160
   #define AUDIO_MTU_BASIC_SIZE 20
  #else
   #define AUDIO_MTU_SIZE 20
  #endif
 #endif

 extern uint8_t voice_rpt[];
 #ifdef HID_AUDIO
 extern uint8_t app_voice_ctrl_input_rpt[];
 extern uint8_t app_voice_ctrl_feature_rpt[];
 #endif

 typedef void (queueUserEvent)(HidEventUserDefine * event);
 void audio_init(void (*activityDetectedPtr)());
 void audio_shutdown();
 void audio_queue_stop_event();
 void audio_start();
 void audio_stop();
 void audio_pollActivityVoice(void);
 void audio_start_request();
 void audio_stop_request();
 void audio_procEvent(uint8_t eventType);
 void audio_voiceCtlSend(uint8_t eventType);
 wiced_bool_t audio_button(uint8_t key, wiced_bool_t down);

 #ifdef HID_AUDIO
  wiced_bool_t audio_event(wiced_hidd_app_event_queue_t * eventQ);
  wiced_bool_t audio_setReport(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
 #else
  #define audio_event(q) FALSE
  #define audio_setReport(reportType, reportId, payload, payloadSize) FALSE
 #endif
 #ifdef AUDIO_QUEUE_EVENT
  void audio_flush_data(void);
  #define audio_DATA()      {(audio.evtQueueFuncPtr)(&audio.voiceEvent); if (++audio.packetInQueue >= hidd_mic_audio_FIFO_count()) { hidd_mic_audio_overflow();}
  #define audio_STOP()      audio_queue_event(HID_EVENT_MIC_STOP)
  #define audio_START()     audio_queue_event(HID_EVENT_MIC_START)
  #define audio_STOP_REQ()  audio_queue_event(HID_EVENT_RC_MIC_STOP_REQ)
  #define audio_START_REQ() audio_queue_event(HID_EVENT_RC_MIC_START_REQ)
  #define audio_CODEC_RD()  audio_queue_event(HID_EVENT_AUDIO_CODEC_RD)
  #define audio_CODEC_WR()  audio_queue_event(HID_EVENT_AUDIO_CODEC_WT)
  #define audio_MODE_RD()   audio_queue_event(HID_EVENT_AUDIO_MODE)
 #else
  #define audio_flush_data()
  #define audio_DATA()      audio_procEvtVoice();
  #define audio_STOP()      audio_stop()
  #define audio_START()     audio_start()
  #define audio_STOP_REQ()  audio_stop_request()
  #define audio_START_REQ() audio_start_request()
  #define audio_CODEC_RD()  audio_voiceReadCodecSetting()
  #define audio_CODEC_WR()  audio_voiceWriteCodecSetting()
  #define audio_MODE_RD()   audio_voiceCtlSend(HIDD_RC_VOICEMODE_RD_ACK)
  #define audio_flush_data()
 #endif
#else // !SUPPORT_AUDIO
 #define audio_is_active() FALSE
 #define audio_init(ptr)
 #define audio_shutdown()
 #define audio_flush_data()
 #define audio_queue_stop_event()
 #define audio_start()
 #define audio_stop()
 #define audio_setReport(reportType, reportId, payload, payloadSize) FALSE
 #define audio_pollActivityVoice()
 #define audio_procEvent(eventType)
 #define audio_event(q) FALSE
 #define audio_button(key, down) FALSE
#endif // SUPPORT_AUDIO

#endif // __SUPPORT_AUDIO_H__
