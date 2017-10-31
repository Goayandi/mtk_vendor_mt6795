/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>
#include <cutils/log.h>
#include <string.h>
#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_s5k5e8yxmipiraw.h"
#include "camera_info_s5k5e8yxmipiraw.h"
#include "camera_custom_AEPlinetable.h"
#include "camera_custom_tsf_tbl.h"

#define NVRAM_TUNING_PARAM_NUM  5341001

const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,

    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
      {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      }
    },
    ISPPca: {
#include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
#include INCLUDE_FILENAME_ISP_REGS_PARAM
    },
    ISPMulitCCM:{
      Poly22:{
        69220,      // i4R_AVG
        18277,      // i4R_STD
        113820,      // i4B_AVG
        31458,      // i4B_STD
        0,      // i4R_MAX
        0,      // i4R_MIN
        0,      // i4G_MAX
        0,      // i4G_MIN
        0,      // i4B_MAX
        0,      // i4B_MIN
        {  // i4P00[9]
            9250000, -4504000, 366000, -1382000, 6358000, 144000, -136000, -5016000, 10264000
        },
        {  // i4P10[9]
            2616677, -3696084, 1062755, 189612, -765700, 585495, -261113, -1078554, 1326076
        },
        {  // i4P01[9]
            1770820, -2966261, 1178475, -341001, -967595, 1312896, -844765, -3092076, 3929900
        },
        {  // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },
      AWBGain:{
        // Strobe
        {
          512,    // i4R
          512,    // i4G
          512    // i4B
        },
        // A
        {
          574,    // i4R
          512,    // i4G
          1318    // i4B
        },
        // TL84
        {
          681,    // i4R
          512,    // i4G
          1041    // i4B
        },
        // CWF
        {
          768,    // i4R
          512,    // i4G
          1079    // i4B
        },
        // D65
        {
          956,    // i4R
          512,    // i4G
          708    // i4B
        },
        // Reserved 1
        {
          512,    // i4R
          512,    // i4G
          512    // i4B
        },
        // Reserved 2
        {
          512,    // i4R
          512,    // i4G
          512    // i4B
        },
        // Reserved 3
        {
          512,    // i4R
          512,    // i4G
          512    // i4B
        }
      },
      Weight:{
        1, // Strobe
        1, // A
        1, // TL84
        1, // CWF
        1, // D65
        1, // Reserved 1
        1, // Reserved 2
        1  // Reserved 3
      }
    },
    MDPMulitCCM:{
      Poly22:{
        125225,      // i4R_AVG
        32141,      // i4R_STD
        165675,      // i4B_AVG
        30249,      // i4B_STD
        646,      // i4R_MAX
        527,      // i4R_MIN
        789,      // i4G_MAX
        670,      // i4G_MIN
        856,      // i4B_MAX
        721,      // i4B_MIN
        {  // i4P00[9]
            5995000, -505000, -370000, -1387500, 7165000, -657500, -532500, -2045000, 7697500
        },
        {  // i4P10[9]
            725905, -1081879, 355974, 627096, -389018, -238079, 626246, -86040, -540207
        },
        {  // i4P01[9]
            257857, -576218, 318361, -96902, -138571, 235473, 24959, 193148, -218107
        },
        {  // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        }
      },

      AWBGain:{
        // Strobe
        {
          1016,    // i4R
          512,    // i4G
          1101    // i4B
        },
        // A
        {
          878,    // i4R
          512,    // i4G
          1971    // i4B
        },
        // TL84
        {
          1192,    // i4R
          512,    // i4G
          1628    // i4B
        },
        // CWF
        {
          1280,    // i4R
          512,    // i4G
          1773    // i4B
        },
        // D65
        {
          1659,    // i4R
          512,    // i4G
          1255    // i4B
        },
        // Reserved 1
        {
          512,    // i4R
          512,    // i4G
          512    // i4B
        },
        // Reserved 2
        {
          512,    // i4R
          512,    // i4G
          512    // i4B
        },
        // Reserved 3
        {
          512,    // i4R
          512,    // i4G
          512    // i4B
        }
      },
      Weight:{
        1, // Strobe
        1, // A
        1, // TL84
        1, // CWF
        1, // D65
        1, // Reserved 1
        1, // Reserved 2
        1  // Reserved 3
      }
    },
    isp_ccm_ratio: 1.000000,
    //bInvokeSmoothCCM
    bInvokeSmoothCCM: MFALSE,
    DngMetadata:
    {
        0,  //i4RefereceIlluminant1
        3,  //i4RefereceIlluminant2
      rNoiseProfile:{
        {
          S:{
            0.000004,      // a
            0.000336       // b
          },
          O:{
            0.000000,      // a
            -0.000234       // b
          }
        },
        {
          S:{
            0.000004,      // a
            0.000336       // b
          },
          O:{
            0.000000,      // a
            -0.000234       // b
          }
        },
        {
          S:{
            0.000004,      // a
            0.000336       // b
          },
          O:{
            0.000000,      // a
            -0.000234       // b
          }
        },
        {
          S:{
            0.000004,      // a
            0.000336       // b
          },
          O:{
            0.000000,      // a
            -0.000234       // b
          }
        }
      }
    },
  rGmaParam:
  {
        {   // Normal Mode
            eISP_DYNAMIC_GMA_MODE,  // eGMAMode
          8,
			{
				{   // i4ContrastWeightingTbl
				        0, 0, 0, 10, 20, 40, 60, 80, 80, 80, 80
				},
				{   // i4LVWeightingTbl
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  33,  66, 100, 100, 100, 100, 100, 100, 100, 100
                }
            },
			{
				1,      // i4Enable
				1,      // i4WaitAEStable
				4       // i4Speed
			},
			{
				0,      // i4Enable
				2047,      // i4CenterPt
				50,      // i4LowPercent
				100000,      // i4LowCurve100
				100000,      // i4HighCurve100
				50,      // i4HighPercent
				100,      // i4SlopeH100
				100       // i4SlopeL100
			},
            {
                0       // rGMAFlare.i4Enable
            }
        },
        {   // HDR Mode
            eISP_DYNAMIC_GMA_MODE,  // eGMAMode
          8,
			{
				{   // i4ContrastWeightingTbl
				        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
				},
				{   // i4LVWeightingTbl
				        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 33, 66, 100, 100, 100, 100, 100, 100, 100, 100
                }
            },
			{
				1,      // i4Enable
				1,      // i4WaitAEStable
				4       // i4Speed
			},
			{
				0,      // i4Enable
				2047,      // i4CenterPt
				50,      // i4LowPercent
				100000,      // i4LowCurve100
				100000,      // i4HighCurve100
				50,      // i4HighPercent
				100,      // i4SlopeH100
				100       // i4SlopeL100
			},
            {
                0       // rGMAFlare.i4Enable
            }
        },
        {   // Reserve 0
            eISP_DYNAMIC_GMA_MODE,  // eGMAMode
          8,
			{
				{   // i4ContrastWeightingTbl
				        0, 0, 33, 66, 100, 100, 100, 100, 100, 100, 100
				},
				{   // i4LVWeightingTbl
				        0, 0, 0, 0, 0, 0, 0, 0, 0, 33, 66, 100, 100, 100, 100, 100, 100, 100, 100, 100
				},
			},
			{
				1,      // i4Enable
				1,      // i4WaitAEStable
				4       // i4Speed
			},
			{
				0,      // i4Enable
				2047,      // i4CenterPt
				50,      // i4LowPercent
				100000,      // i4LowCurve100
				100000,      // i4HighCurve100
				50,      // i4HighPercent
				100,      // i4SlopeH100
				100       // i4SlopeL100
			},
			{
				0       // i4Enable
            }
        },
        {   // Reserve 1
            eISP_DYNAMIC_GMA_MODE,  // eGMAMode
            8,                  // i4LowContrastThr
            {
				{   // i4ContrastWeightingTbl
				        0, 0, 33, 66, 100, 100, 100, 100, 100, 100, 100
				},
				{   // i4LVWeightingTbl
				        0, 0, 0, 0, 0, 0, 0, 0, 0, 33, 66, 100, 100, 100, 100, 100, 100, 100, 100, 100
				},
			},
			{
				1,      // i4Enable
				1,      // i4WaitAEStable
				4       // i4Speed
			},
			{
				0,      // i4Enable
				2047,      // i4CenterPt
				50,      // i4LowPercent
				100000,      // i4LowCurve100
				100000,      // i4HighCurve100
				50,      // i4HighPercent
				100,      // i4SlopeH100
				100       // i4SlopeL100
			},
			{
				0       // i4Enable
			},
      }
  },
  rLceParam:
  {
      {   // Normal Mode
			1,      // i4AutoLCEEnable
			{   // rDefLCEParam
				0,      // i4LCEBa
				50,      // i4LCEPa
				110       // i4LCEPb
			},
			{   // rAutoLCEParam
				3,      // i4LCESeg
				800,      // i4LCEContrastRatio
				{
					0,      // i4LCEBa
					{   // i4LCEPa
						{70, 70, 70, 70, 70, 70, 70, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 70, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 60, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 60, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50}
					},
					{   // i4LCEPb
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 90, 90, 80, 80, 80, 80, 80},
						{120, 120, 120, 120, 120, 130, 100, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 100, 100, 100, 100, 100, 100, 100, 100},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 100, 110, 110, 110, 110, 110, 110, 110},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 110, 110, 110, 110, 110, 110, 110, 110},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 110, 110, 110, 110, 110, 110, 110, 110}
					},
				},
				{   // rLCESmooth
					1,      // i4Enable
					0,      // i4WaitAEStable
					4      // i4Speed
				},
                {
                    // rLCEFlare
                    0   // i4Enable
                }
			}
      },
      {   // HDR Mode
			1,      // i4AutoLCEEnable
			{   // rDefLCEParam
				0,      // i4LCEBa
				50,      // i4LCEPa
				110       // i4LCEPb
			},
			{   // rAutoLCEParam
				3,      // i4LCESeg
				800,      // i4LCEContrastRatio
				{
					0,      // i4LCEBa
					{   // i4LCEPa
						{70, 70, 70, 70, 70, 70, 70, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 70, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 60, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 60, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50}
					},
					{   // i4LCEPb
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 90, 90, 80, 80, 80, 80, 80},
						{120, 120, 120, 120, 120, 130, 100, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 100, 100, 100, 100, 100, 100, 100, 100},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 100, 110, 110, 110, 110, 110, 110, 110},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 110, 110, 110, 110, 110, 110, 110, 110},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 110, 110, 110, 110, 110, 110, 110, 110}
					},
				},
				{   // rLCESmooth
					1,      // i4Enable
					0,      // i4WaitAEStable
					4      // i4Speed
				},
				{   // rLCEFlare
                    // rLCEFlare
                    0   // i4Enable
                }
            }
        },
      {   // Reserve 0
			1,      // i4AutoLCEEnable
			{   // rDefLCEParam
				0,      // i4LCEBa
				50,      // i4LCEPa
				110       // i4LCEPb
			},
			{   // rAutoLCEParam
				3,      // i4LCESeg
				800,      // i4LCEContrastRatio
				{
					0,      // i4LCEBa
					{   // i4LCEPa
						{70, 70, 70, 70, 70, 70, 70, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 70, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 60, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 60, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50}
					},
					{   // i4LCEPb
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 90, 90, 80, 80, 80, 80, 80},
						{120, 120, 120, 120, 120, 130, 100, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 100, 100, 100, 100, 100, 100, 100, 100},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 100, 110, 110, 110, 110, 110, 110, 110},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 110, 110, 110, 110, 110, 110, 110, 110},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 110, 110, 110, 110, 110, 110, 110, 110}
					},
				},
				{   // rLCESmooth
					1,      // i4Enable
					0,      // i4WaitAEStable
					4      // i4Speed
				},
				{   // rLCEFlare
					0,      // i4Enable
				}
			}
      },
      {   // Reserve 1
			1,      // i4AutoLCEEnable
			{   // rDefLCEParam
				0,      // i4LCEBa
				50,      // i4LCEPa
				110       // i4LCEPb
			},
			{   // rAutoLCEParam
				3,      // i4LCESeg
				800,      // i4LCEContrastRatio
				{
					0,      // i4LCEBa
					{   // i4LCEPa
						{70, 70, 70, 70, 70, 70, 70, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 70, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 60, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{70, 70, 70, 70, 70, 70, 60, 60, 60, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{60, 60, 60, 60, 60, 60, 60, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
						{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50}
					},
					{   // i4LCEPb
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 90, 80, 80, 80, 80, 80, 80},
						{130, 130, 130, 130, 130, 130, 100, 100, 90, 90, 90, 90, 90, 90, 80, 80, 80, 80, 80},
						{120, 120, 120, 120, 120, 130, 100, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 100, 100, 100, 100, 100, 100, 100, 100},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 100, 110, 110, 110, 110, 110, 110, 110},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 110, 110, 110, 110, 110, 110, 110, 110},
						{120, 120, 120, 120, 120, 110, 100, 90, 90, 90, 90, 110, 110, 110, 110, 110, 110, 110, 110}
					},
				},
				{   // rLCESmooth
					1,      // i4Enable
					0,      // i4WaitAEStable
					4      // i4Speed
				},
				{   // rLCEFlare
					0,      // i4Enable
				}
			}
      }
  },
  ANR_TBL:
  {0}
}};

const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1144,    // u4MinGain, 1024 base = 1x
            16384,    // u4MaxGain, 16x
            99,    // u4MiniISOGain, ISOxx  
            32,    // u4GainStepUnit, 1x/8 
            30953,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            17001,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            17001,    // u4CapExpUnit 
            29,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            16525,    // u4Video1ExpUnit
            119,    // u4Video1MaxFrameRate
            1024,    // u4Video12PreRatio, 1024 base = 1x
            31239,    // u4Video2ExpUnit
            30,    // u4Video2MaxFrameRate
            1024,    // u4Video22PreRatio, 1024 base = 1x
            20256,    // u4Custom1ExpUnit
            24,    // u4Custom1MaxFrameRate
            1024,    // u4Custom12PreRatio, 1024 base = 1x
            20256,    // u4Custom2ExpUnit
            15,    // u4Custom2MaxFrameRate
            1024,    // u4Custom22PreRatio, 1024 base = 1x
            17001,    // u4Custom3ExpUnit
            29,    // u4Custom3MaxFrameRate
            1024,    // u4Custom32PreRatio, 1024 base = 1x
            17001,    // u4Custom4ExpUnit
            29,    // u4Custom4MaxFrameRate
            1024,    // u4Custom42PreRatio, 1024 base = 1x
            17001,    // u4Custom5ExpUnit
            29,    // u4Custom5MaxFrameRate
            1024,    // u4Custom52PreRatio, 1024 base = 1x
            22,    // u4LensFno, Fno = 2.8
            350    // u4FocusLength_100x
        },
        // rHistConfig
        {
            4,    // u4HistHighThres
            40,    // u4HistLowThres
            2,    // u4MostBrightRatio
            1,    // u4MostDarkRatio
            160,    // u4CentralHighBound
            20,    // u4CentralLowBound
            {240, 230, 220, 210, 200},    // u4OverExpThres[AE_CCT_STRENGTH_NUM] 
            {62, 70, 82, 108, 141},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
            {18, 22, 26, 30, 34}    // u4BlackLightThres[AE_CCT_STRENGTH_NUM] 
        },
        // rCCTConfig
        {
            TRUE,    // bEnableBlackLight
            TRUE,    // bEnableHistStretch
            TRUE,    // bEnableAntiOverExposure
            TRUE,    // bEnableTimeLPF
            TRUE,    // bEnableCaptureThres
            TRUE,    // bEnableVideoThres
            TRUE,    // bEnableVideo1Thres
            TRUE,    // bEnableVideo2Thres
            TRUE,    // bEnableCustom1Thres
            TRUE,    // bEnableCustom2Thres
            TRUE,    // bEnableCustom3Thres
            TRUE,    // bEnableCustom4Thres
            TRUE,    // bEnableCustom5Thres
            TRUE,    // bEnableStrobeThres
            58,    // u4AETarget
            47,    // u4StrobeAETarget
            58,    // u4InitIndex
            4,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            2,    // u4BlackLightStrengthIndex
            2,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            -14,    // i4BVOffset delta BV = value/10 
            64,    // u4PreviewFlareOffset
            64,    // u4CaptureFlareOffset
            3,    // u4CaptureFlareThres
            64,    // u4VideoFlareOffset
            3,    // u4VideoFlareThres
            64,    // u4CustomFlareOffset
            3,    // u4CustomFlareThres
            64,    // u4StrobeFlareOffset
            3,    // u4StrobeFlareThres
            160,    // u4PrvMaxFlareThres
            0,    // u4PrvMinFlareThres
            160,    // u4VideoMaxFlareThres
            0,    // u4VideoMinFlareThres
            18,    // u4FlatnessThres    // 10 base for flatness condition.
            75,    // u4FlatnessStrength
            //rMeteringSpec
            {
                //rHS_Spec
                {
                    TRUE,//bEnableHistStretch           // enable histogram stretch
                    1024,//u4HistStretchWeight          // Histogram weighting value
                    40,//u4Pcent                      // 
                    220,//u4Thd                        // 0~255
                    75, //74,//u4FlatThd                    // 0~255
                    120,//u4FlatBrightPcent
                    120,//u4FlatDarkPcent
                    //sFlatRatio
                    {
                        1000,  //i4X1
                        1024,  //i4Y1
                        2400,  //i4X2
                        0  //i4Y2
                    },
                    TRUE, //bEnableGreyTextEnhance
                    1800, //u4GreyTextFlatStart, > sFlatRatio.i4X1, < sFlatRatio.i4X2
                    {
                        10,  //i4X1
                        1024,  //i4Y1
                        80,  //i4X2
                        0  //i4Y2
                    }
                },
                //rAOE_Spec
                {
                    TRUE, //bEnableAntiOverExposure
                    1024, //u4AntiOverExpWeight
                    10, //u4Pcent
                    220, //u4Thd
                    TRUE, //bEnableCOEP
                    1, //u4COEPcent
                    106, //u4COEThd
                    0, //u4BVCompRatio
                    //sCOEYRatio;     // the outer y ratio
                    {
                        23,  //i4X1
                        1024,  //i4Y1
                        47,  //i4X2
                        0  //i4Y2
                    },
                    //sCOEDiffRatio;  // inner/outer y difference ratio
                    {
                        1500,  //i4X1
                        0,  //i4Y1
                        2100,  //i4X2
                        1024  //i4Y2
                    }
                },
                //rABL_Spec
                {
                    TRUE,//bEnableBlackLight
                    1024,//u4BackLightWeight
                    400,//u4Pcent
                    22,//u4Thd
                    255, // center luminance
                    246, // final target limitation, 256/128 = 2x
                    //sFgBgEVRatio
                    {
                        2100,  //i4X1
                        0,  //i4Y1
                        4000,  //i4X2
                        1024  //i4Y2
                    },
                    //sBVRatio
                    {
                        3800,  //i4X1
                        0,  //i4Y1
                        5000,  //i4X2
                        1024  //i4Y2
                    }
                },
                //rNS_Spec
                {
                    TRUE, // bEnableNightScene
                    10,//u4Pcent
                    150,//u4Thd
                    72,//u4FlatThd
                    180,//u4BrightTonePcent
                    80,//u4BrightToneThd
                    500,//u4LowBndPcent
                    5,//u4LowBndThd
                    26,//u4LowBndThdLimit
                    50,//u4FlatBrightPcent
                    300,//u4FlatDarkPcent
                    //sFlatRatio
                    {
                        1200,  //i4X1
                        1024,  //i4Y1
                        2800,  //i4X2
                        0  //i4Y2
                    },
                    //sBVRatio
                    {
                        -500,  //i4X1
                        1024,  //i4Y1
                        3000,  //i4X2
                        0  //i4Y2
                    },
                    TRUE, // bEnableNightSkySuppresion
                    //sSkyBVRatio
                    {
                        -4000,  //i4X1
                        1024,  //i4Y1
                        -2000,  //i4X2
                        0  //i4Y2
                    }
                },
                // rTOUCHFD_Spec
                {
                    40, //uMeteringYLowBound;
                    50, //uMeteringYHighBound;
                    40, //uFaceYLowBound;
                    50, //uFaceYHighBound;
                    3, //uFaceCentralWeight;
                    120, //u4MeteringStableMax;
                    80, //u4MeteringStableMin;
                }
            }, //End rMeteringSpec
            // rFlareSpec
            {
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //uPrvFlareWeightArr[16];
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //uVideoFlareWeightArr[16];
                96,                                               //u4FlareStdThrHigh;
                48,                                               //u4FlareStdThrLow;
                0,                                               //u4PrvCapFlareDiff;
                2,                                               //u4FlareMaxStepGap_Fast;
                0,                                               //u4FlareMaxStepGap_Slow;
                1800,                                               //u4FlarMaxStepGapLimitBV;
                2,                                               //u4FlareAEStableCount;
            },
            //rAEMoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                190, //u4Bright2TargetEnd
                10, //u4Dark2TargetStart
                90, //u4B2TEnd
                85, //u4B2TStart
                70, //u4D2TEnd
                85, //u4D2TStart
            },
            //rAEVideoMoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150, //u4Bright2TargetEnd
                20, //u4Dark2TargetStart
                90, //u4B2TEnd
                40, //u4B2TStart
                30, //u4D2TEnd
                90, //u4D2TStart
            },
            //rAEVideo1MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150, //u4Bright2TargetEnd
                20, //u4Dark2TargetStart
                90, //u4B2TEnd
                40, //u4B2TStart
                30, //u4D2TEnd
                90, //u4D2TStart
            },
            //rAEVideo2MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150, //u4Bright2TargetEnd
                20, //u4Dark2TargetStart
                90, //u4B2TEnd
                40, //u4B2TStart
                30, //u4D2TEnd
                90, //u4D2TStart
            },
            //rAECustom1MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150, //u4Bright2TargetEnd
                20, //u4Dark2TargetStart
                90, //u4B2TEnd
                40, //u4B2TStart
                30, //u4D2TEnd
                90, //u4D2TStart
            },
            //rAECustom2MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150, //u4Bright2TargetEnd
                20, //u4Dark2TargetStart
                90, //u4B2TEnd
                40, //u4B2TStart
                30, //u4D2TEnd
                90, //u4D2TStart
            },
            //rAECustom3MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150, //u4Bright2TargetEnd
                20, //u4Dark2TargetStart
                90, //u4B2TEnd
                40, //u4B2TStart
                30, //u4D2TEnd
                90, //u4D2TStart
            },
            //rAECustom4MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150, //u4Bright2TargetEnd
                20, //u4Dark2TargetStart
                90, //u4B2TEnd
                40, //u4B2TStart
                30, //u4D2TEnd
                90, //u4D2TStart
            },
            //rAECustom5MoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150, //u4Bright2TargetEnd
                20, //u4Dark2TargetStart
                90, //u4B2TEnd
                40, //u4B2TStart
                30, //u4D2TEnd
                90, //u4D2TStart
            },
            //rAEFaceMoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                190, //u4Bright2TargetEnd
                10, //u4Dark2TargetStart
                80, //u4B2TEnd
                30, //u4B2TStart
                20, //u4D2TEnd
                60, //u4D2TStart
            },
            //rAETrackingMoveRatio =
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                190, //u4Bright2TargetEnd
                10, //u4Dark2TargetStart
                80, //u4B2TEnd
                30, //u4B2TStart
                20, //u4D2TEnd
                60, //u4D2TStart
            },
            //rAEAOENVRAMParam =
            {
                1,      // i4AOEStrengthIdx: 0 / 1 / 2
                130,      // u4BVCompRatio
                {//rAEAOEAlgParam
                    {//rAEAOEAlgParam[0]
                        47,  //u4Y_Target
                        25,  //u4AOE_OE_percent
                        210,  //u4AOE_OEBound
                        10,  //u4AOE_DarkBound
                        950,  //u4AOE_LowlightPrecent
                        1,  //u4AOE_LowlightBound
                        145,  //u4AOESceneLV_L
                        180,  //u4AOESceneLV_H
                        40,  //u4AOE_SWHdrLE_Bound
                    },
                    {//rAEAOEAlgParam[1]
                        47,  //u4Y_Target
                        25,  //u4AOE_OE_percent
                        210,  //u4AOE_OEBound
                        15,  //u4AOE_DarkBound
                        950,  //u4AOE_LowlightPrecent
                        3,  //u4AOE_LowlightBound
                        145,  //u4AOESceneLV_L
                        180,  //u4AOESceneLV_H
                        40,  //u4AOE_SWHdrLE_Bound
                    },
                    {//rAEAOEAlgParam[2]
                        47,  //u4Y_Target
                        25,  //u4AOE_OE_percent
                        210,  //u4AOE_OEBound
                        25,  //u4AOE_DarkBound
                        950,  //u4AOE_LowlightPrecent
                        8,  //u4AOE_LowlightBound
                        145,  //u4AOESceneLV_L
                        180,  //u4AOESceneLV_H
                        40,  //u4AOE_SWHdrLE_Bound
                    }
                }
            }
        },
        // rAEHDRConfig
        {
            3072,    // i4RMGSeg
            35,    // i4HDRTarget_L
            40,    // i4HDRTarget_H
            100,    // i4HDRTargetLV_L
            120,    // i4HDRTargetLV_H
            20,    // i4OverExpoRatio
            212,    // i4OverExpoTarget
            120,    // i4OverExpoLV_L
            180,    // i4OverExpoLV_H
            4,    // i4UnderExpoContrastThr
            {3, 3, 3, 3, 3, 2, 1, 1, 1, 1, 1            },    // i4UnderExpoTargetTbl[AE_HDR_UNDEREXPO_CONTRAST_TARGET_TBL_NUM]
            950,    // i4UnderExpoRatio
            1000,    // i4AvgExpoRatio
            8,    // i4AvgExpoTarget
            768,    // i4HDRAESpeed
            2,    // i4HDRConvergeThr
            40,    // i4SWHdrLEThr
            20,    // i4SWHdrSERatio
            180,    // i4SWHdrSETarget
            1024        // i4SWHdrBaseGain
        }
    },
    // AWB NVRAM
    AWB_NVRAM_START
    {
        {
          {
            // AWB calibration data
            {
                // rUnitGain (unit gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rGoldenGain (golden sample gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rUnitGain TL84 (unit gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rGoldenGain TL84 (golden sample gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rUnitGain Alight (unit gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rGoldenGain Alight (golden sample gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rD65Gain (D65 WB gain: 1.0 = 512)
                {
                    962,    // D65Gain_R
                    512,    // D65Gain_G
                    727    // D65Gain_B
                }
            },
            // Original XY coordinate of AWB light source
            {
                // Strobe
                {
                    0,    // i4X
                    0    // i4Y
                },
                // Horizon
                {
                    -438,    // OriX_Hor
                    -379    // OriY_Hor
                },
                // A
                {
                    -318,    // OriX_A
                    -393    // OriY_A
                },
                // TL84
                {
                    -105,    // OriX_TL84
                    -369    // OriY_TL84
                },
                // CWF
                {
                    -131,    // OriX_CWF
                    -440    // OriY_CWF
                },
                // DNP
                {
                    4,    // OriX_DNP
                    -401    // OriY_DNP
                },
                // D65
                {
                    103,    // OriX_D65
                    -363    // OriY_D65
                },
                // DF
                {
                    0,    // OriX_DF
                    0    // OriY_DF
                }
            },
            // Rotated XY coordinate of AWB light source
            {
                // Strobe
                {
                    0,    // i4X
                    0    // i4Y
                },
                // Horizon
                {
                    -457,    // RotX_Hor
                    -357    // RotY_Hor
                },
                // A
                {
                    -338,    // RotX_A
                    -377    // RotY_A
                },
                // TL84
                {
                    -124,    // RotX_TL84
                    -364    // RotY_TL84
                },
                // CWF
                {
                    -153,    // RotX_CWF
                    -433    // RotY_CWF
                },
                // DNP
                {
                    -16,    // RotX_DNP
                    -401    // RotY_DNP
                },
                // D65
                {
                    85,    // RotX_D65
                    -368    // RotY_D65
                },
                // DF
                {
                    70,    // RotX_DF
                    -425    // RotY_DF
                }
            },
            // AWB gain of AWB light source
            {
                // Strobe 
                {
                    512,    // i4R
                    512,    // i4G
                    512    // i4B
                },
                // Horizon 
                {
                    512,    // AWBGAIN_HOR_R
                    554,    // AWBGAIN_HOR_G
                    1677    // AWBGAIN_HOR_B
                },
                // A 
                {
                    567,    // AWBGAIN_A_R
                    512,    // AWBGAIN_A_G
                    1340    // AWBGAIN_A_B
                },
                // TL84 
                {
                    733,    // AWBGAIN_TL84_R
                    512,    // AWBGAIN_TL84_G
                    972    // AWBGAIN_TL84_B
                },
                // CWF 
                {
                    779,    // AWBGAIN_CWF_R
                    512,    // AWBGAIN_CWF_G
                    1109    // AWBGAIN_CWF_B
                },
                // DNP 
                {
                    886,    // AWBGAIN_DNP_R
                    512,    // AWBGAIN_DNP_G
                    876    // AWBGAIN_DNP_B
                },
                // D65 
                {
                    962,    // AWBGAIN_D65_R
                    512,    // AWBGAIN_D65_G
                    727    // AWBGAIN_D65_B
                },
                // DF 
                {
                    512,    // AWBGAIN_DF_R
                    512,    // AWBGAIN_DF_G
                    512    // AWBGAIN_DF_B
                }
            },
            // Rotation matrix parameter
            {
                3,    // RotationAngle
                256,    // Cos
                13    // Sin
            },
            // Daylight locus parameter
            {
                -140,    // i4SlopeNumerator
                128    // i4SlopeDenominator
            },
            // Predictor gain
            {
                101, // i4PrefRatio100
                // DaylightLocus_L
                {
                    932,    // i4R
                    530,    // i4G
                    747    // i4B
                },
                // DaylightLocus_H
                {
                    791,    // i4R
                    512,    // i4G
                    903    // i4B
                },
                // Temporal General
                {
                    962,    // i4R
                    512,    // i4G
                    727    // i4B
                },
                // AWB LSC Gain
                {
                    830,        // i4R
                    512,        // i4G
                    857        // i4B
                }
            },
            // AWB light area
            {
                // Strobe:FIXME
                {
                    0,    // i4RightBound
                    0,    // i4LeftBound
                    0,    // i4UpperBound
                    0    // i4LowerBound
                },
                // Tungsten
                {
                    -210,    // TungRightBound
                    -857,    // TungLeftBound
                    -302,    // TungUpperBound
                    -371    // TungLowerBound
                },
                // Warm fluorescent
                {
                    -210,    // WFluoRightBound
                    -857,    // WFluoLeftBound
                    -371,    // WFluoUpperBound
                    -523    // WFluoLowerBound
                },
                // Fluorescent
                {
                    -51,    // FluoRightBound
                    -210,    // FluoLeftBound
                    -308,    // FluoUpperBound
                    -399    // FluoLowerBound
                },
                // CWF
                {
                -55,    // CWFRightBound
                -210,    // CWFLeftBound
                -399,    // CWFUpperBound
                -488    // CWFLowerBound
                },
                // Daylight
                {
                    115,    // DayRightBound
                    -51,    // DayLeftBound
                    -308,    // DayUpperBound
                    -399    // DayLowerBound
                },
                // Shade
                {
                    445,    // ShadeRightBound
                    115,    // ShadeLeftBound
                    -308,    // ShadeUpperBound
                    -394    // ShadeLowerBound
                },
                // Daylight Fluorescent
                {
                    115,    // DFRightBound
                    -55,    // DFLeftBound
                    -399,    // DFUpperBound
                    -488    // DFLowerBound
                }
            },
            // PWB light area
            {
                // Reference area
                {
                    445,    // PRefRightBound
                    -857,    // PRefLeftBound
                    -277,    // PRefUpperBound
                    -523    // PRefLowerBound
                },
                // Daylight
                {
                    140,    // PDayRightBound
                    -51,    // PDayLeftBound
                    -308,    // PDayUpperBound
                    -399    // PDayLowerBound
                },
                // Cloudy daylight
                {
                    240,    // PCloudyRightBound
                    65,    // PCloudyLeftBound
                    -308,    // PCloudyUpperBound
                    -399    // PCloudyLowerBound
                },
                // Shade
                {
                    340,    // PShadeRightBound
                    65,    // PShadeLeftBound
                    -308,    // PShadeUpperBound
                    -399    // PShadeLowerBound
                },
                // Twilight
                {
                    -51,    // PTwiRightBound
                    -211,    // PTwiLeftBound
                    -308,    // PTwiUpperBound
                    -399    // PTwiLowerBound
                },
                // Fluorescent
                {
                    135,    // PFluoRightBound
                    -253,    // PFluoLeftBound
                    -314,    // PFluoUpperBound
                    -483    // PFluoLowerBound
                },
                // Warm fluorescent
                {
                    -238,    // PWFluoRightBound
                    -438,    // PWFluoLeftBound
                    -314,    // PWFluoUpperBound
                    -483    // PWFluoLowerBound
                },
                // Incandescent
                {
                    -238,    // PIncaRightBound
                    -438,    // PIncaLeftBound
                    -308,    // PIncaUpperBound
                    -399    // PIncaLowerBound
                },
                // Gray World
                {
                    5000,    // PGWRightBound
                    -5000,    // PGWLeftBound
                    5000,    // PGWUpperBound
                    -5000    // PGWLowerBound
                }
            },
            // PWB default gain	
            {
                // Daylight
                {
                    895,    // PWB_Day_R
                    512,    // PWB_Day_G
                    756    // PWB_Day_B
                },
                // Cloudy daylight
                {
                    1028,    // PWB_Cloudy_R
                    512,    // PWB_Cloudy_G
                    649    // PWB_Cloudy_B
                },
                // Shade
                {
                    1096,    // PWB_Shade_R
                    512,    // PWB_Shade_G
                    604    // PWB_Shade_B
                },
                // Twilight
                {
                    715,    // PWB_Twi_R
                    512,    // PWB_Twi_G
                    970    // PWB_Twi_B
                },
                // Fluorescent
                {
                    836,    // PWB_Fluo_R
                    512,    // PWB_Fluo_G
                    928    // PWB_Fluo_B
                },
                // Warm fluorescent
                {
                    584,    // PWB_WFluo_R
                    512,    // PWB_WFluo_G
                    1378    // PWB_WFluo_B
                },
                // Incandescent
                {
                    548,    // PWB_Inca_R
                    512,    // PWB_Inca_G
                    1301    // PWB_Inca_B
                },
                // Gray World
                {
                    512,    // PWB_GW_R
                    512,    // PWB_GW_G
                    512    // PWB_GW_B
                }
            },
            // AWB preference color	
            {
                // Tungsten
                {
                    40,    // TUNG_SLIDER
                    4223    // TUNG_OFFS
                },
                // Warm fluorescent	
                {
                    40,    // WFluo_SLIDER
                    4223    // WFluo_OFFS
                },
                // Shade
                {
                    50,    // Shade_SLIDER
                    909    // Shade_OFFS
                },
                // Sunset Area
                {
                    11,   // i4Sunset_BoundXr_Thr
                    -401    // i4Sunset_BoundYr_Thr
                },
                // Shade F Area
                {
                    -210,   // i4BoundXrThr
                    -368    // i4BoundYrThr
                },
                // Shade F Vertex
                {
                    -131,   // i4BoundXrThr
                    -384    // i4BoundYrThr
                },
                // Shade CWF Area
                {
                    -210,   // i4BoundXrThr
                    -437    // i4BoundYrThr
                },
                // Shade CWF Vertex
                {
                    -133,   // i4BoundXrThr
                    -463    // i4BoundYrThr
                },
            },
            // CCT estimation
            {
                // CCT
                {
                    2300,    // i4CCT[0]
                    2850,    // i4CCT[1]
                    3750,    // i4CCT[2]
                    5100,    // i4CCT[3]
                    6500     // i4CCT[4]
                },
                // Rotated X coordinate
                {
                -542,    // i4RotatedXCoordinate[0]
                -423,    // i4RotatedXCoordinate[1]
                -209,    // i4RotatedXCoordinate[2]
                -101,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
                }
            }
        },
        // Algorithm Tuning Parameter
        {
            // AWB Backup Enable
            0,

            // Daylight locus offset LUTs for tungsten
            {
                21, // i4Size: LUT dimension
                {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000}, // i4LUTIn
                {0, 350, 800, 1222, 1444, 1667, 1889, 2111, 2333, 2556, 2778, 3000, 3222, 3444, 3667, 3889, 4111, 4333, 4556, 4778, 5000} // i4LUTOut
            },
            // Daylight locus offset LUTs for WF
            {
                21, // i4Size: LUT dimension
                {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000}, // i4LUTIn
                {0, 350, 700, 1000, 1444, 1667, 1889, 2111, 2333, 2556, 2778, 3000, 3222, 3444, 3667, 3889, 4111, 4333, 4556, 4778, 5000} // i4LUTOut
            },
            // Daylight locus offset LUTs for Shade
            {
                21, // i4Size: LUT dimension
                {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000}, // i4LUTIn
                {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000} // i4LUTOut
            },
            // Preference gain for each light source
            {
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // STROBE
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // TUNGSTEN
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // WARM F
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}
                }, // F
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // CWF
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}
                }, // DAYLIGHT
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // SHADE
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                } // DAYLIGHT F
            },
            // Parent block weight parameter
            {
                1,      // bEnable
                6           // i4ScalingFactor: [6] 1~12, [7] 1~6, [8] 1~3, [9] 1~2, [>=10]: 1
            },
            // AWB LV threshold for predictor
            {
                115,    // i4InitLVThr_L
                155,    // i4InitLVThr_H
                100      // i4EnqueueLVThr
            },
            // AWB number threshold for temporal predictor
            {
                65,     // i4Neutral_ParentBlk_Thr
                //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  25,   2,   2,   2,   2,   2,   2,   2}  // (%) i4CWFDF_LUTThr
                },
                // AWB light neutral noise reduction for outdoor
                {
                    //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                    // Non neutral
                    {   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    // Flurescent
                    {   0,   0,   0,   0,   0,   3,   5,   5,   5,   5,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    // CWF
                    {   0,   0,   0,   0,   0,   3,   5,   5,   5,   5,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    // Daylight
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   2,   2,   2,   2,   2,   2,   2,   2},  // (%)
                    // DF
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                },
                // AWB feature detection
                {
                    // Sunset Prop
                    {
                        1,          // i4Enable
                        120,        // i4LVThr_L
                        130,        // i4LVThr_H
                        10,         // i4SunsetCountThr
                        0,          // i4SunsetCountRatio_L
                        171         // i4SunsetCountRatio_H
                    },
                    // Shade F Detection
                    {
                        1,          // i4Enable
                        50,        // i4LVThr_L
                        90,        // i4LVThr_H
                        128         // i4DaylightProb
                    },
                    // Shade CWF Detection
                    {
                        1,          // i4Enable
                        50,        // i4LVThr_L
                        90,        // i4LVThr_H
                        192         // i4DaylightProb
                    },
                },
                // AWB non-neutral probability for spatial and temporal weighting look-up table (Max: 100; Min: 0)
                {
                    //LV0   1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
                    {   0,  33,  66, 100, 100, 100, 100, 100, 100, 100, 100,  70,  30,  20,  10,   0,   0,   0,   0}
                },
                // AWB daylight locus probability look-up table (Max: 100; Min: 0)
                {   //LV0    1     2     3      4     5     6     7     8      9      10     11    12   13     14    15   16    17    18
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  25,   0,   0,   0,   0}, // Strobe
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  75,  50,  25,   0,   0,   0}, // Tungsten
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  75,  50,  25,  25,  25,   0,   0,   0}, // Warm fluorescent
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  95,  75,  50,  25,  25,  25,   0,   0,   0}, // Fluorescent
                    {  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  80,  55,  30,  30,  30,  30,   0,   0,   0}, // CWF
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  75,  50,  40,  30,  20}, // Daylight
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  75,  50,  25,   0,   0,   0,   0}, // Shade
                    {  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  80,  55,  30,  30,  30,  30,   0,   0,   0} // Daylight fluorescent
                },
                // AWB tuning information
                {
                    0,		// project code
                    0,		// model
                    0,		// date
                    0,		// reserved 0
                    0,		// reserved 1
                    0,		// reserved 2
                    0,		// reserved 3
                    0,		// reserved 4
                }
            }
        },
        {
          {
            // AWB calibration data
            {
                // rUnitGain (unit gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rGoldenGain (golden sample gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rUnitGain TL84 (unit gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rGoldenGain TL84 (golden sample gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rUnitGain Alight (unit gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rGoldenGain Alight (golden sample gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // rD65Gain (D65 WB gain: 1.0 = 512)
                {
                    1095,    // D65Gain_R
                    512,    // D65Gain_G
                    736    // D65Gain_B
                }
            },
            // Original XY coordinate of AWB light source
            {
                // Strobe
                {
                    0,    // i4X
                    0    // i4Y
                },
                // Horizon
                {
                    -425,    // OriX_Hor
                    -449    // OriY_Hor
                },
                // A
                {
                    -278,    // OriX_A
                    -462    // OriY_A
                },
                // TL84
                {
                    -78,    // OriX_TL84
                    -493    // OriY_TL84
                },
                // CWF
                {
                    -37,    // OriX_CWF
                    -530    // OriY_CWF
                },
                // DNP
                {
                    23,    // OriX_DNP
                    -487    // OriY_DNP
                },
                // D65
                {
                    188,    // OriX_D65
                    -442    // OriY_D65
                },
                // DF
                {
                    168,    // OriX_DF
                    -527    // OriY_DF
                }
            },
            // Rotated XY coordinate of AWB light source
            {
                // Strobe
                {
                    0,    // i4X
                    0    // i4Y
                },
                // Horizon
                {
                    -441,    // RotX_Hor
                    -434    // RotY_Hor
                },
                // A
                {
                    -294,    // RotX_A
                    -452    // RotY_A
                },
                // TL84
                {
                    -95,    // RotX_TL84
                    -490    // RotY_TL84
                },
                // CWF
                {
                    -56,    // RotX_CWF
                    -529    // RotY_CWF
                },
                // DNP
                {
                    6,    // RotX_DNP
                    -488    // RotY_DNP
                },
                // D65
                {
                    172,    // RotX_D65
                    -449    // RotY_D65
                },
                // DF
                {
                    149,    // RotX_DF
                    -533    // RotY_DF
                }
            },
            // AWB gain of AWB light source
            {
                // Strobe 
                {
                    512,    // i4R
                    512,    // i4G
                    512    // i4B
                },
                // Horizon 
                {
                    529,    // AWBGAIN_HOR_R
                    512,    // AWBGAIN_HOR_G
                    1671    // AWBGAIN_HOR_B
                },
                // A 
                {
                    656,    // AWBGAIN_A_R
                    512,    // AWBGAIN_A_G
                    1394    // AWBGAIN_A_B
                },
                // TL84 
                {
                    898,    // AWBGAIN_TL84_R
                    512,    // AWBGAIN_TL84_G
                    1109    // AWBGAIN_TL84_B
                },
                // CWF 
                {
                    998,    // AWBGAIN_CWF_R
                    512,    // AWBGAIN_CWF_G
                    1103    // AWBGAIN_CWF_B
                },
                // DNP 
                {
                    1022,    // AWBGAIN_DNP_R
                    512,    // AWBGAIN_DNP_G
                    959    // AWBGAIN_DNP_B
                },
                // D65 
                {
                    1202,    // AWBGAIN_D65_R
                    512,    // AWBGAIN_D65_G
                    722    // AWBGAIN_D65_B
                },
                // DF 
                {
                    1312,    // AWBGAIN_DF_R
                    512,    // AWBGAIN_DF_G
                    833    // AWBGAIN_DF_B
                }
            },
            // Rotation matrix parameter
            {
                2,    // RotationAngle
                256,    // Cos
                9    // Sin
            },
            // Daylight locus parameter
            {
                -138,    // i4SlopeNumerator
                128    // i4SlopeDenominator
            },
            // Predictor gain
            {
                101, // i4PrefRatio100
                // DaylightLocus_L
                {
                    1233,    // i4R
                    512,    // i4G
                    703    // i4B
                },
                // DaylightLocus_H
                {
                    899,    // i4R
                    512,    // i4G
                    986    // i4B
                },
                // Temporal General
                {
                    1233,    // i4R
                    512,    // i4G
                    703    // i4B
                },
                // AWB LSC Gain
                {
                    938,        // i4R
                    512,        // i4G
                    943        // i4B
                }
            },
            // AWB light area
            {
                // Strobe:FIXME
                {
                    0,    // i4RightBound
                    0,    // i4LeftBound
                    0,    // i4UpperBound
                    0    // i4LowerBound
                },
                // Tungsten
                {
                    -195,    // TungRightBound
                    -841,    // TungLeftBound
                    -399,    // TungUpperBound
                    -471    // TungLowerBound
                },
                // Warm fluorescent
                {
                    -195,    // WFluoRightBound
                    -841,    // WFluoLeftBound
                    -471,    // WFluoUpperBound
                    -529    // WFluoLowerBound
                },
                // Fluorescent
                {
                    -19,    // FluoRightBound
                    -195,    // FluoLeftBound
                    -417,    // FluoUpperBound
                    -514    // FluoLowerBound
                },
                // CWF
                {
                26,    // CWFRightBound
                -170,    // CWFLeftBound
                -514,    // CWFUpperBound
                -574    // CWFLowerBound
                },
                // Daylight
                {
                    202,    // DayRightBound
                    -19,    // DayLeftBound
                    -417,    // DayUpperBound
                    -514    // DayLowerBound
                },
                // Shade
                {
                    532,    // ShadeRightBound
                    202,    // ShadeLeftBound
                    -417,    // ShadeUpperBound
                    -481    // ShadeLowerBound
                },
                // Daylight Fluorescent
                {
                    202,    // DFRightBound
                    26,    // DFLeftBound
                    -514,    // DFUpperBound
                    -574    // DFLowerBound
                }
            },
            // PWB light area
            {
                // Reference area
                {
                    532,    // PRefRightBound
                    -841,    // PRefLeftBound
                    -374,    // PRefUpperBound
                    -574    // PRefLowerBound
                },
                // Daylight
                {
                    227,    // PDayRightBound
                    -19,    // PDayLeftBound
                    -417,    // PDayUpperBound
                    -514    // PDayLowerBound
                },
                // Cloudy daylight
                {
                    327,    // PCloudyRightBound
                    152,    // PCloudyLeftBound
                    -417,    // PCloudyUpperBound
                    -514    // PCloudyLowerBound
                },
                // Shade
                {
                    427,    // PShadeRightBound
                    152,    // PShadeLeftBound
                    -417,    // PShadeUpperBound
                    -514    // PShadeLowerBound
                },
                // Twilight
                {
                    -19,    // PTwiRightBound
                    -179,    // PTwiLeftBound
                    -417,    // PTwiUpperBound
                    -514    // PTwiLowerBound
                },
                // Fluorescent
                {
                    222,    // PFluoRightBound
                    -195,    // PFluoLeftBound
                    -399,    // PFluoUpperBound
                    -579    // PFluoLowerBound
                },
                // Warm fluorescent
                {
                    -194,    // PWFluoRightBound
                    -394,    // PWFluoLeftBound
                    -399,    // PWFluoUpperBound
                    -579    // PWFluoLowerBound
                },
                // Incandescent
                {
                    -194,    // PIncaRightBound
                    -394,    // PIncaLeftBound
                    -417,    // PIncaUpperBound
                    -514    // PIncaLowerBound
                },
                // Gray World
                {
                    5000,    // PGWRightBound
                    -5000,    // PGWLeftBound
                    5000,    // PGWUpperBound
                    -5000    // PGWLowerBound
                }
            },
            // PWB default gain	
            {
                // Daylight
                {
                    1125,    // PWB_Day_R
                    512,    // PWB_Day_G
                    812    // PWB_Day_B
                },
                // Cloudy daylight
                {
                    1343,    // PWB_Cloudy_R
                    512,    // PWB_Cloudy_G
                    672    // PWB_Cloudy_B
                },
                // Shade
                {
                    1433,    // PWB_Shade_R
                    512,    // PWB_Shade_G
                    627    // PWB_Shade_B
                },
                // Twilight
                {
                    863,    // PWB_Twi_R
                    512,    // PWB_Twi_G
                    1079    // PWB_Twi_B
                },
                // Fluorescent
                {
                    1033,    // PWB_Fluo_R
                    512,    // PWB_Fluo_G
                    951    // PWB_Fluo_B
                },
                // Warm fluorescent
                {
                    692,    // PWB_WFluo_R
                    512,    // PWB_WFluo_G
                    1462    // PWB_WFluo_B
                },
                // Incandescent
                {
                    669,    // PWB_Inca_R
                    512,    // PWB_Inca_G
                    1418    // PWB_Inca_B
                },
                // Gray World
                {
                    512,    // PWB_GW_R
                    512,    // PWB_GW_G
                    512    // PWB_GW_B
                }
            },
            // AWB preference color	
            {
                // Tungsten
                {
                    100,    // TUNG_SLIDER
                    4004    // TUNG_OFFS
                },
                // Warm fluorescent	
                {
                    100,    // WFluo_SLIDER
                    4004    // WFluo_OFFS
                },
                // Shade
                {
                    50,    // Shade_SLIDER
                    909    // Shade_OFFS
                },
                // Sunset Area
                {
                    19,   // i4Sunset_BoundXr_Thr
                    -488    // i4Sunset_BoundYr_Thr
                },
                // Shade F Area
                {
                    -70,   // i4BoundXrThr
                    -449    // i4BoundYrThr
                },
                // Shade F Vertex
                {
                    -70,   // i4BoundXrThr
                    -449    // i4BoundYrThr
                },
                // Shade CWF Area
                {
                    -56,   // i4BoundXrThr
                    -529    // i4BoundYrThr
                },
                // Shade CWF Vertex
                {
                    -56,   // i4BoundXrThr
                    -529    // i4BoundYrThr
                },
            },
            // CCT estimation
            {
                // CCT
                {
                    2300,    // i4CCT[0]
                    2850,    // i4CCT[1]
                    3750,    // i4CCT[2]
                    5100,    // i4CCT[3]
                    6500     // i4CCT[4]
                },
                // Rotated X coordinate
                {
                -547,    // i4RotatedXCoordinate[0]
                -423,    // i4RotatedXCoordinate[1]
                -271,    // i4RotatedXCoordinate[2]
                -104,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
                }
            }
        },
        // Algorithm Tuning Parameter
        {
            // AWB Backup Enable
            0,

            // Daylight locus offset LUTs for tungsten
            {
                21, // i4Size: LUT dimension
                {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000}, // i4LUTIn
                {0, 350, 800, 1222, 1444, 1667, 1889, 2111, 2333, 2556, 2778, 3000, 3222, 3444, 3667, 3889, 4111, 4333, 4556, 4778, 5000} // i4LUTOut
            },
            // Daylight locus offset LUTs for WF
            {
                21, // i4Size: LUT dimension
                {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000}, // i4LUTIn
                {0, 350, 700, 850, 1200, 1667, 1889, 2111, 2333, 2556, 2778, 3000, 3222, 3444, 3667, 3889, 4111, 4333, 4556, 4778, 5000} // i4LUTOut
            },
            // Daylight locus offset LUTs for Shade
            {
                21, // i4Size: LUT dimension
                {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000}, // i4LUTIn
                {0, 500, 1000, 1500, 2500, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8000, 8000, 8500, 9000, 9500, 10000} // i4LUTOut
            },
            // Preference gain for each light source
            {
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // STROBE
                {
                    {530, 530, 480}, {530, 530, 480}, {530, 530, 480}, {530, 530, 480}, {530, 530, 480}, {520, 520, 496}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // TUNGSTEN
                {
                    {512, 512, 508}, {512, 512, 508}, {512, 512, 508}, {512, 512, 508}, {512, 512, 508}, {512, 512, 508}, {512, 512, 508}, {512, 512, 508}, {512, 512, 508}, {512, 512, 508}, 
                    {512, 512, 508}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // WARM F
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}
                }, // F
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // CWF
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}, {502, 512, 512}
                }, // DAYLIGHT
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                }, // SHADE
                {
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 
                    {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}
                } // DAYLIGHT F
            },
            // Parent block weight parameter
            {
                1,      // bEnable
                6           // i4ScalingFactor: [6] 1~12, [7] 1~6, [8] 1~3, [9] 1~2, [>=10]: 1
            },
            // AWB LV threshold for predictor
            {
                115,    // i4InitLVThr_L
                155,    // i4InitLVThr_H
                100      // i4EnqueueLVThr
            },
            // AWB number threshold for temporal predictor
            {
                65,     // i4Neutral_ParentBlk_Thr
                //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  25,   2,   2,   2,   2,   2,   2,   2}  // (%) i4CWFDF_LUTThr
                },
                // AWB light neutral noise reduction for outdoor
                {
                    //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                    // Non neutral
                    {   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    // Flurescent
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    // CWF
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    // Daylight
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   2,   2,   2,   2,   2,   2,   2,   2},  // (%)
                    // DF
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                },
                // AWB feature detection
                {
                    // Sunset Prop
                    {
                        1,          // i4Enable
                        120,        // i4LVThr_L
                        130,        // i4LVThr_H
                        10,         // i4SunsetCountThr
                        0,          // i4SunsetCountRatio_L
                        171         // i4SunsetCountRatio_H
                    },
                    // Shade F Detection
                    {
                        1,          // i4Enable
                        90,        // i4LVThr_L
                        105,        // i4LVThr_H
                        96         // i4DaylightProb
                    },
                    // Shade CWF Detection
                    {
                        1,          // i4Enable
                        90,        // i4LVThr_L
                        105,        // i4LVThr_H
                        208         // i4DaylightProb
                    },
                },
                // AWB non-neutral probability for spatial and temporal weighting look-up table (Max: 100; Min: 0)
                {
                    //LV0   1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
                    {  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  50,  30,  20,  10,   0,   0,   0,   0}
                },
                // AWB daylight locus probability look-up table (Max: 100; Min: 0)
                {   //LV0    1     2     3      4     5     6     7     8      9      10     11    12   13     14    15   16    17    18
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  25,   0,   0,   0,   0}, // Strobe
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  75,  50,  25,  25,  25,   0,   0,   0}, // Tungsten
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  75,  50,  25,  25,  25,   0,   0,   0}, // Warm fluorescent
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100,  95,  95,  70,  50,  25,  25,  25,   0,   0,   0}, // Fluorescent
                    {  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  80,  55,  30,  30,  30,  30,   0,   0,   0}, // CWF
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  75,  50,  40,  30,  20}, // Daylight
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  75,  50,  25,   0,   0,   0,   0}, // Shade
                    {  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  80,  55,  30,  30,  30,  30,   0,   0,   0} // Daylight fluorescent
                },
                // AWB tuning information
                {
                    6735,		// project code
                    5588,		// model
                    20150624,		// date
                    0,		// reserved 0
                    1,		// reserved 1
                    2,		// reserved 2
                    3,		// reserved 3
                    4,		// reserved 4
                }
            }
        }
    },
    // Flash AWB NVRAM
    {
#include INCLUDE_FILENAME_FLASH_AWB_PARA
    },
    {0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace

const CAMERA_TSF_TBL_STRUCT CAMERA_TSF_DEFAULT_VALUE =
{
    {
        1, // isTsfEn
        2, // tsfCtIdx
        {20, 2000, -110, -110, 512, 512, 512, 0}    // rAWBInput[8]
    },
#include INCLUDE_FILENAME_TSF_PARA
#include INCLUDE_FILENAME_TSF_DATA
};


const NVRAM_CAMERA_FEATURE_STRUCT CAMERA_FEATURE_DEFAULT_VALUE =
{
#include INCLUDE_FILENAME_FEATURE_PARA
};
typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),
                                             sizeof(NVRAM_LENS_PARA_STRUCT),
                                             sizeof(AE_PLINETABLE_T),
                                             0,
                                             sizeof(CAMERA_TSF_TBL_STRUCT),
                                             0,
                                             sizeof(NVRAM_CAMERA_FEATURE_STRUCT)
    };

    if (CameraDataType > CAMERA_NVRAM_DATA_FEATURE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
        return 1;
    }

    switch(CameraDataType)
    {
        case CAMERA_NVRAM_DATA_ISP:
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_3A:
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_SHADING:
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
            break;
        case CAMERA_DATA_AE_PLINETABLE:
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
            break;
        case CAMERA_DATA_TSF_TABLE:
            memcpy(pDataBuf,&CAMERA_TSF_DEFAULT_VALUE,sizeof(CAMERA_TSF_TBL_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_FEATURE:
            memcpy(pDataBuf,&CAMERA_FEATURE_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_FEATURE_STRUCT));
            break;
        default:
            break;
    }
    return 0;
}}; // NSFeature

