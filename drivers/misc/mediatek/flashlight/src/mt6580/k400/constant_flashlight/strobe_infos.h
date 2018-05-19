/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _STROBE_INFOS_H
#define _STROBE_INFOS_H
#define DEBUG_LEDS_STROBE
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[AW3643_strobe]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#ifdef DEBUG_LEDS_STROBE
    #define PK_DBG PK_DBG_FUNC
    #define PK_VER PK_TRC_VERBOSE
    #define PK_ERR PK_ERROR
#else
    #define PK_DBG(a, ...)
    #define PK_VER(a, ...)
    #define PK_ERR(a, ...)
#endif

#define STROBE_DEVICE_ID 			 0x63
#define LM3643_REG_ENABLE       		 0x01
#define LM3643_REG_TIMING            	 0x08
#define LM3643_REG_COMPATIB      	 0x0C
#define LM3643_REG_FLASH_LED1  	 0x03
#define LM3643_REG_FLASH_LED2  	 0x04
#define LM3643_REG_TORCH_LED1  	 0x05
#define LM3643_REG_TORCH_LED2		 0x06
#define LM3643_REG_BOOST_CONFIG 	 0x07
#endif
