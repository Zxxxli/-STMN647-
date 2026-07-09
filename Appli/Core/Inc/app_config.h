/**
 ****************************************************************************************************
 * @file        app_config.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app_config.h文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 N647开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "stm32n6xx_hal.h"
#include "postprocess_conf.h"

#define LCD_BG_WIDTH                            800
#define LCD_BG_HEIGHT                           480
#define LCD_FG_WIDTH                            LCD_BG_WIDTH
#define LCD_FG_HEIGHT                           LCD_BG_HEIGHT

#define DISPLAY_DELAY                           1
#define DISPLAY_BUFFER_NB                       (DISPLAY_DELAY + 2)

#define CAMERA_MIRROR_FLIP                      CMW_MIRRORFLIP_MIRROR

#define CPU_LOAD_HISTORY_DEPTH                  8

#define BQUEUE_MAX_BUFFERS                      3

#define NN_WIDTH                                256
#define NN_HEIGHT                               256
#define NN_FORMAT                               DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1
#define NN_BPP                                  3
#define NN_BUFFER_OUT_SIZE                      301056
#define NN_CLASSES                              2
#define NN_CLASSES_TABLE                        {"person", "not_person"}

#define POSTPROCESS_TYPE                        POSTPROCESS_MPE_YOLO_V8_UF
#define AI_MPE_YOLOV8_PP_TOTAL_BOXES            1344
#define AI_MPE_YOLOV8_PP_NB_CLASSES             1
#define AI_POSE_PP_POSE_KEYPOINTS_NB            17
#define AI_MPE_YOLOV8_PP_CONF_THRESHOLD         0.7500000000f
#define AI_MPE_YOLOV8_PP_IOU_THRESHOLD          0.5000000000f
#define AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT        10

#define CIRCLE_RADIUS                           5
#define BINDING_WIDTH                           3
#define COLOR_HEAD                              UTIL_LCD_COLOR_GREEN
#define COLOR_ARMS                              UTIL_LCD_COLOR_BLUE
#define COLOR_TRUNK                             UTIL_LCD_COLOR_MAGENTA
#define COLOR_LEGS                              UTIL_LCD_COLOR_ORANGE
#define COLOR_BOX                               UTIL_LCD_COLOR_RED

#endif
