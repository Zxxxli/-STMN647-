/**
 ****************************************************************************************************
 * @file        app.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app.c文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 N647开发板
 * 在线视频:www.yuanzige.com
 * 技术论�?www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#include "app.h"
#include "app_config.h"
#include "app_utils.h"
#include "app_lcd.h"
#include "app_camera.h"
#include "app_bqueue.h"
#include "app_cpuload.h"
#include "app_postprocess.h"
#include "fibocom.h"
#include "tx_api.h"
#include "cmw_camera.h"
#include "ll_aton_runtime.h"
#include <math.h>
#include <string.h>

typedef enum {
    SQUAT_STATE_UNKNOWN = 0,
    SQUAT_STATE_STAND,
    SQUAT_STATE_DOWN
} squat_state_t;

typedef enum {
    ACTION_NONE = 0,
    ACTION_SQUAT,
    ACTION_JUMPING_JACK,
    ACTION_PLANK
} action_type_t;

typedef enum {
    JACK_STATE_UNKNOWN = 0,
    JACK_STATE_OPEN
} jack_state_t;

typedef struct {
    uint8_t valid;

    uint8_t knee_valid;
    float left_knee_angle;
    float right_knee_angle;
    float knee_angle;
    float knee_angle_filtered;

    uint8_t shoulder_valid;
    float shoulder_width;

    uint8_t ankle_valid;
    float ankle_ratio;
    float ankle_ratio_filtered;

    uint8_t torso_v_valid;
    float torso_v_angle;
    float torso_v_angle_filtered;

    uint8_t leg_angle_valid;
    float leg_angle;
    float leg_angle_filtered;

    uint8_t stretch_valid;
    float stretch_score;
    float stretch_score_filtered;

    uint8_t arm_valid;
    uint8_t arms_up;
    uint8_t left_wrist_up;
    uint8_t right_wrist_up;
    uint8_t left_elbow_up;
    uint8_t right_elbow_up;
    uint8_t arm_evidence;
    uint8_t arms_up_strong;

    uint8_t squat_candidate;
    uint8_t jack_strong_candidate;
    uint8_t jack_mid_candidate;
    uint8_t jack_leg_only_candidate;
    uint8_t jack_candidate;
    uint8_t jack_candidate_strength;

    uint8_t plank_valid;
    uint8_t plank_candidate;
    uint8_t plank_elbow_ground;
    uint8_t plank_hip_drop_bad;
    float plank_body_angle;
    float plank_line_score;
} pose_features_t;

typedef struct {
    int32_t nb_detect;
    mpe_pp_outBuffer_t detects[AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT];
    pose_features_t features;
    action_type_t locked_action;
    action_type_t last_action;
    uint32_t squat_count;
    uint32_t jack_count;
    uint32_t plank_time_s;
    float squat_last_depth_percent;
    uint8_t squat_last_depth_valid;
    uint8_t squat_last_lean_level;
    float squat_last_max_lean_angle;
    uint8_t squat_last_lean_valid;
    float jack_last_stretch_score;
    uint8_t jack_last_stretch_level;
    uint8_t jack_last_stretch_valid;
    uint8_t plank_quality_valid;
    uint8_t plank_quality_level;
    uint8_t plank_hip_drop_bad;
    float plank_body_angle;
    float plank_line_score;
    uint32_t current_count;
    uint32_t action_lost_ms;
    uint8_t action_tracking;
    uint32_t nn_period_ms;
    uint32_t inf_ms;
    uint32_t pp_ms;
    uint32_t disp_ms;
} app_display_info_t;

typedef struct {
    TX_SEMAPHORE update;
    TX_MUTEX lock;
    app_display_info_t info;
} app_display_t;

static TX_SEMAPHORE isp_semaphore;

static void app_camera_display_pipe_vsync_cb(void);
static void app_camera_display_pipe_frame_cb(void);
static void app_camera_nn_pipe_frame_cb(void);

static TX_THREAD nn_thread;
static UCHAR nn_thread_stack[4096];
static TX_THREAD pp_thread;
static UCHAR pp_thread_stack[4096];
static TX_THREAD dp_thread;
static UCHAR dp_thread_stack[4096];
static TX_THREAD isp_thread;
static UCHAR isp_thread_stack[4096];

static VOID nn_thread_entry(ULONG id);
static VOID pp_thread_entry(ULONG id);
static VOID dp_thread_entry(ULONG id);
static VOID isp_thread_entry(ULONG id);

#define KP_LEFT_SHOULDER   5
#define KP_RIGHT_SHOULDER  6
#define KP_LEFT_ELBOW      7
#define KP_RIGHT_ELBOW     8
#define KP_LEFT_WRIST      9
#define KP_RIGHT_WRIST     10
#define KP_LEFT_HIP        11
#define KP_RIGHT_HIP       12
#define KP_LEFT_KNEE       13
#define KP_RIGHT_KNEE      14
#define KP_LEFT_ANKLE      15
#define KP_RIGHT_ANKLE     16

#define KP_CONF_TH_BODY              0.40f
#define KP_CONF_TH_ARM               0.35f

#define SQUAT_DOWN_TH                110.0f
#define SQUAT_STAND_TH               145.0f
#define SQUAT_LOCK_ANKLE_MAX_RATIO   1.60f
#define SQUAT_DEPTH_STAND_KNEE_REF   170.0f
#define SQUAT_DEPTH_DEEP_KNEE_REF     50.0f
#define SQUAT_LEAN_LV1_TH             16.0f
#define SQUAT_LEAN_LV2_TH             22.0f
#define SQUAT_LEAN_LV3_TH             32.0f
#define SQUAT_LEAN_LV4_TH             42.0f

#define JACK_OPEN_RATIO_STRONG_TH       1.65f
#define JACK_OPEN_RATIO_MID_TH          1.50f
#define JACK_OPEN_RATIO_LEG_ONLY_TH     1.75f
#define JACK_CLOSE_RATIO_TH             1.35f
#define JACK_STRETCH_LEGANG_CLOSE_REF    0.0f
#define JACK_STRETCH_LEGANG_OPEN_REF    28.0f
#define JACK_STRETCH_LV1_TH             20.0f
#define JACK_STRETCH_LV2_TH             40.0f
#define JACK_STRETCH_LV3_TH             60.0f
#define JACK_STRETCH_LV4_TH             80.0f

#define JACK_WRIST_UP_MARGIN            0.02f
#define JACK_ELBOW_UP_MARGIN            0.12f

#define PLANK_BODY_MAX_ANGLE            28.0f
#define PLANK_MIN_HORIZONTAL_SPAN        0.22f
#define PLANK_MIN_LINE_SCORE            55.0f
#define PLANK_HIP_DROP_CLOSE_RATIO      0.25f
#define PLANK_ELBOW_WRIST_Y_MARGIN       0.08f
#define PLANK_SHOULDER_ARM_Y_MARGIN      0.04f
#define PLANK_CONFIRM_FRAMES             3

#define JACK_STRONG_CONFIRM_FRAMES      1
#define JACK_MID_CONFIRM_FRAMES         2
#define JACK_LEG_ONLY_CONFIRM_FRAMES    2

#define ACTION_CANDIDATE_FRAMES      2
#define ACTION_CONFIRM_FRAMES        2
#define ACTION_LOCK_TIMEOUT_MS       5000U
#define ACTION_LOST_TIMEOUT_MS       1000U

#define ANGLE_EMA_ALPHA              0.45f
#define RATIO_EMA_ALPHA              0.45f
#define LEG_ANGLE_EMA_ALPHA          0.45f
#define STRETCH_EMA_ALPHA            0.45f

#define SQUAT_TARGET_COUNT           20U
#define JACK_TARGET_COUNT            20U
#define PLANK_ENCOURAGE_TIME_S       15U
#define PLANK_TARGET_TIME_S          30U
#define SQUAT_DEPTH_GOOD_TH          70.0f
#define SQUAT_LEAN_GOOD_MIN_TH       20.0f
#define SQUAT_LEAN_GOOD_MAX_TH       45.0f
#define JACK_STRETCH_GOOD_TH         80.0f
#define PLANK_QUALITY_GOOD_LEVEL     4U

static const uint32_t bindings[][3] = {
    {15, 13, COLOR_LEGS},
    {13, 11, COLOR_LEGS},
    {16, 14, COLOR_LEGS},
    {14, 12, COLOR_LEGS},
    {11, 12, COLOR_TRUNK},
    {5, 11, COLOR_TRUNK},
    {6, 12, COLOR_TRUNK},
    {5, 6, COLOR_ARMS},
    {5, 7, COLOR_ARMS},
    {6, 8, COLOR_ARMS},
    {7, 9, COLOR_ARMS},
    {8, 10, COLOR_ARMS},
    {1, 2, COLOR_HEAD},
    {0, 1, COLOR_HEAD},
    {0, 2, COLOR_HEAD},
    {1, 3, COLOR_HEAD},
    {2, 4, COLOR_HEAD},
    {3, 5, COLOR_HEAD},
    {4, 6, COLOR_HEAD},
};

static const uint32_t kp_color[AI_POSE_PP_POSE_KEYPOINTS_NB] = {
    COLOR_HEAD,
    COLOR_HEAD,
    COLOR_HEAD,
    COLOR_HEAD,
    COLOR_HEAD,
    COLOR_ARMS,
    COLOR_ARMS,
    COLOR_ARMS,
    COLOR_ARMS,
    COLOR_ARMS,
    COLOR_ARMS,
    COLOR_TRUNK,
    COLOR_TRUNK,
    COLOR_LEGS,
    COLOR_LEGS,
    COLOR_LEGS,
    COLOR_LEGS,
};

static app_display_t display;
static action_type_t g_locked_action = ACTION_NONE;
static action_type_t g_last_action = ACTION_NONE;
static uint32_t g_action_lock_start_ms = 0;

static action_type_t g_candidate_action = ACTION_NONE;
static uint8_t g_candidate_confirm = 0;

static uint32_t g_squat_count = 0;
static uint32_t g_jack_count = 0;
static uint32_t g_plank_time_s = 0;
static uint32_t g_plank_start_ms = 0;
static uint32_t g_plank_last_seen_ms = 0;
static uint8_t g_plank_quality_valid = 0;
static uint8_t g_plank_quality_level = 0;
static uint8_t g_plank_hip_drop_bad = 0;
static float g_plank_body_angle = -1.0f;
static float g_plank_line_score = -1.0f;

static squat_state_t g_squat_state = SQUAT_STATE_UNKNOWN;
static uint8_t g_squat_armed = 0;
static uint8_t g_squat_stand_confirm = 0;

static jack_state_t g_jack_state = JACK_STATE_UNKNOWN;
static uint8_t g_jack_armed = 0;
static uint8_t g_jack_close_confirm = 0;

static uint32_t g_lost_start_ms = 0;
static uint8_t g_lost_active = 0;

static float g_knee_angle_filtered = -1.0f;
static uint8_t g_knee_filter_valid = 0;

static float g_ankle_ratio_filtered = -1.0f;
static uint8_t g_ankle_filter_valid = 0;

static float g_torso_v_angle_filtered = -1.0f;
static uint8_t g_torso_v_filter_valid = 0;
static float g_leg_angle_filtered = -1.0f;
static uint8_t g_leg_angle_filter_valid = 0;
static float g_stretch_score_filtered = -1.0f;
static uint8_t g_stretch_filter_valid = 0;

static float g_squat_rep_max_depth_percent = 0.0f;
static float g_squat_rep_max_lean_angle = 0.0f;
static uint8_t g_squat_rep_depth_valid = 0;
static uint8_t g_squat_rep_lean_valid = 0;
static float g_squat_last_depth_percent = 0.0f;
static uint8_t g_squat_last_depth_valid = 0;
static uint8_t g_squat_last_lean_level = 0;
static float g_squat_last_max_lean_angle = 0.0f;
static uint8_t g_squat_last_lean_valid = 0;

static float g_jack_rep_max_stretch_score = 0.0f;
static uint8_t g_jack_rep_stretch_valid = 0;
static float g_jack_last_stretch_score = 0.0f;
static uint8_t g_jack_last_stretch_level = 0;
static uint8_t g_jack_last_stretch_valid = 0;

static uint8_t g_squat_target_voice_done = 0;
static uint8_t g_jack_target_voice_done = 0;
static uint8_t g_plank_encourage_voice_done = 0;
static uint8_t g_plank_target_voice_done = 0;
static uint8_t g_all_target_voice_done = 0;
static uint8_t g_squat_depth_voice_done = 0;
static uint8_t g_squat_lean_low_voice_done = 0;
static uint8_t g_squat_lean_high_voice_done = 0;
static uint8_t g_jack_stretch_voice_done = 0;
static uint8_t g_plank_quality_voice_done = 0;

LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default);
static uint8_t nn_input_buffers[3][NN_WIDTH * NN_HEIGHT * NN_BPP] __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static app_bqueue_t nn_input_queue;
static uint8_t nn_output_buffers[2][ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32)] __attribute__((aligned(32)));
static app_bqueue_t nn_output_queue;
static const char *nn_classes_table[NN_CLASSES] = NN_CLASSES_TABLE;

static app_cpuload_t cpuload;

static void app_display_network_output(app_display_info_t *display_info);
static const char *app_action_str(action_type_t action);
static int32_t app_select_main_person(mpe_pp_outBuffer_t *detects, int32_t nb_detect);
static uint8_t app_key_valid(mpe_pp_keyPoints_t *key, float th);
static float app_calc_angle(mpe_pp_keyPoints_t *a, mpe_pp_keyPoints_t *b, mpe_pp_keyPoints_t *c);
static uint8_t app_calc_torso_v_angle(mpe_pp_outBuffer_t *p, float *angle);
static uint8_t app_calc_leg_spread_angle(mpe_pp_outBuffer_t *p, float *angle);
static float app_clampf(float v, float lo, float hi);
static float app_calc_squat_depth_percent(float knee_angle);
static uint8_t app_calc_squat_lean_level(float lean_angle);
static void app_reset_squat_rep_stats(void);
static void app_update_squat_rep_stats(pose_features_t *features);
static void app_commit_squat_rep_stats(void);
static float app_calc_jack_stretch_from_leg_angle(float leg_angle);
static uint8_t app_calc_jack_stretch_level(float stretch_score);
static void app_reset_jack_rep_stats(void);
static void app_update_jack_rep_stats(pose_features_t *features);
static void app_check_all_targets_voice(void);
static void app_handle_squat_voice(void);
static void app_handle_jack_voice(void);
static void app_handle_plank_voice(void);
static void app_commit_jack_rep_stats(void);
static uint8_t app_calc_plank_pose(mpe_pp_outBuffer_t *p, pose_features_t *features);
static void app_reset_plank_state(void);
static uint8_t app_calc_plank_quality_level(uint8_t hip_drop_bad);
static void app_extract_pose_features(mpe_pp_outBuffer_t *detects,
                                      int32_t nb_detect,
                                      pose_features_t *features);
static action_type_t app_select_action_candidate(pose_features_t *features);
static uint8_t app_get_jack_required_confirm(pose_features_t *features);
static void app_update_action_logic(pose_features_t *features, uint32_t now_ms);
static void app_update_display_action_info(app_display_info_t *info);

void app_run(void)
{
    app_lcd_init();
    app_bqueue_init(&nn_input_queue, 3, (uint8_t *[3]){nn_input_buffers[0], nn_input_buffers[1], nn_input_buffers[2]});
    app_bqueue_init(&nn_output_queue, 2, (uint8_t *[2]){nn_output_buffers[0], nn_output_buffers[1]});
    app_cpuload_init(&cpuload);
    app_camera_init(app_camera_display_pipe_vsync_cb, app_camera_display_pipe_frame_cb, NULL, app_camera_nn_pipe_frame_cb);

    tx_semaphore_create(&isp_semaphore, NULL, 0);
    tx_semaphore_create(&display.update, NULL, 0);
    tx_mutex_create(&display.lock, NULL, TX_INHERIT);

    app_camera_display_pipe_start(app_lcd_get_bg_buffer(), CMW_MODE_CONTINUOUS);

    fibocom_startup_test();

    tx_thread_create(&nn_thread, "NN Thread", nn_thread_entry, 0, nn_thread_stack, sizeof(nn_thread_stack), TX_MAX_PRIORITIES - 3, TX_MAX_PRIORITIES - 3, 10, TX_AUTO_START);
    tx_thread_create(&pp_thread, "PP Thread", pp_thread_entry, 0, pp_thread_stack, sizeof(pp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&dp_thread, "DP Thread", dp_thread_entry, 0, dp_thread_stack, sizeof(dp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&isp_thread, "ISP Thread", isp_thread_entry, 0, isp_thread_stack, sizeof(isp_thread_stack), TX_MAX_PRIORITIES - 4, TX_MAX_PRIORITIES - 4, 10, TX_AUTO_START);
}

static float app_clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static uint8_t app_calc_torso_v_angle(mpe_pp_outBuffer_t *p, float *angle)
{
    float sx;
    float sy;
    float hx;
    float hy;
    float dx;
    float dy;

    if ((p == NULL) || (angle == NULL))
    {
        return 0;
    }

    if (!(app_key_valid(&p->pKeyPoints[KP_LEFT_SHOULDER], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_RIGHT_SHOULDER], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_LEFT_HIP], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_RIGHT_HIP], KP_CONF_TH_BODY)))
    {
        return 0;
    }

    sx = 0.5f * (p->pKeyPoints[KP_LEFT_SHOULDER].x + p->pKeyPoints[KP_RIGHT_SHOULDER].x);
    sy = 0.5f * (p->pKeyPoints[KP_LEFT_SHOULDER].y + p->pKeyPoints[KP_RIGHT_SHOULDER].y);
    hx = 0.5f * (p->pKeyPoints[KP_LEFT_HIP].x + p->pKeyPoints[KP_RIGHT_HIP].x);
    hy = 0.5f * (p->pKeyPoints[KP_LEFT_HIP].y + p->pKeyPoints[KP_RIGHT_HIP].y);

    dx = (sx - hx) * LCD_BG_WIDTH;
    dy = (sy - hy) * LCD_BG_HEIGHT;
    if ((fabsf(dx) < 1e-3f) && (fabsf(dy) < 1e-3f))
    {
        return 0;
    }

    *angle = atan2f(fabsf(dx), fabsf(dy)) * 57.2957795f;
    return 1;
}

static uint8_t app_calc_leg_spread_angle(mpe_pp_outBuffer_t *p, float *angle)
{
    float hx;
    float hy;
    float lax;
    float lay;
    float rax;
    float ray;
    float v1x;
    float v1y;
    float v2x;
    float v2y;
    float dot;
    float n1;
    float n2;
    float c;

    if ((p == NULL) || (angle == NULL))
    {
        return 0;
    }

    if (!(app_key_valid(&p->pKeyPoints[KP_LEFT_HIP], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_RIGHT_HIP], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_LEFT_ANKLE], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_RIGHT_ANKLE], KP_CONF_TH_BODY)))
    {
        return 0;
    }

    hx = 0.5f * (p->pKeyPoints[KP_LEFT_HIP].x + p->pKeyPoints[KP_RIGHT_HIP].x) * LCD_BG_WIDTH;
    hy = 0.5f * (p->pKeyPoints[KP_LEFT_HIP].y + p->pKeyPoints[KP_RIGHT_HIP].y) * LCD_BG_HEIGHT;
    lax = p->pKeyPoints[KP_LEFT_ANKLE].x * LCD_BG_WIDTH;
    lay = p->pKeyPoints[KP_LEFT_ANKLE].y * LCD_BG_HEIGHT;
    rax = p->pKeyPoints[KP_RIGHT_ANKLE].x * LCD_BG_WIDTH;
    ray = p->pKeyPoints[KP_RIGHT_ANKLE].y * LCD_BG_HEIGHT;

    v1x = lax - hx;
    v1y = lay - hy;
    v2x = rax - hx;
    v2y = ray - hy;
    n1 = sqrtf(v1x * v1x + v1y * v1y);
    n2 = sqrtf(v2x * v2x + v2y * v2y);
    if ((n1 < 1.0f) || (n2 < 1.0f))
    {
        return 0;
    }

    dot = v1x * v2x + v1y * v2y;
    c = dot / (n1 * n2);
    c = app_clampf(c, -1.0f, 1.0f);
    *angle = acosf(c) * 57.2957795f;
    return 1;
}

static float app_calc_squat_depth_percent(float knee_angle)
{
    return app_clampf((SQUAT_DEPTH_STAND_KNEE_REF - knee_angle) /
                      (SQUAT_DEPTH_STAND_KNEE_REF - SQUAT_DEPTH_DEEP_KNEE_REF) *
                      100.0f, 0.0f, 100.0f);
}

static uint8_t app_calc_squat_lean_level(float lean_angle)
{
    if (lean_angle < SQUAT_LEAN_LV1_TH)
    {
        return 1;
    }
    else if (lean_angle < SQUAT_LEAN_LV2_TH)
    {
        return 2;
    }
    else if (lean_angle < SQUAT_LEAN_LV3_TH)
    {
        return 3;
    }
    else if (lean_angle < SQUAT_LEAN_LV4_TH)
    {
        return 4;
    }
    return 5;
}

static void app_reset_squat_rep_stats(void)
{
    g_squat_rep_max_depth_percent = 0.0f;
    g_squat_rep_max_lean_angle = 0.0f;
    g_squat_rep_depth_valid = 0;
    g_squat_rep_lean_valid = 0;
}

static void app_update_squat_rep_stats(pose_features_t *features)
{
    float depth;

    if (features == NULL)
    {
        return;
    }

    if (features->knee_valid && (features->knee_angle_filtered > 0.0f))
    {
        depth = app_calc_squat_depth_percent(features->knee_angle_filtered);
        if (depth > g_squat_rep_max_depth_percent)
        {
            g_squat_rep_max_depth_percent = depth;
        }
        g_squat_rep_depth_valid = 1;
    }

    if (features->torso_v_valid && (features->torso_v_angle_filtered >= 0.0f))
    {
        if (features->torso_v_angle_filtered > g_squat_rep_max_lean_angle)
        {
            g_squat_rep_max_lean_angle = features->torso_v_angle_filtered;
        }
        g_squat_rep_lean_valid = 1;
    }
}

static void app_commit_squat_rep_stats(void)
{
    if (g_squat_rep_depth_valid)
    {
        g_squat_last_depth_percent = g_squat_rep_max_depth_percent;
        g_squat_last_depth_valid = 1;
    }

    if (g_squat_rep_lean_valid)
    {
        g_squat_last_max_lean_angle = g_squat_rep_max_lean_angle;
        g_squat_last_lean_level = app_calc_squat_lean_level(g_squat_rep_max_lean_angle);
        g_squat_last_lean_valid = 1;
    }
}

static float app_calc_jack_stretch_from_leg_angle(float leg_angle)
{
    return app_clampf((leg_angle - JACK_STRETCH_LEGANG_CLOSE_REF) /
                      (JACK_STRETCH_LEGANG_OPEN_REF - JACK_STRETCH_LEGANG_CLOSE_REF) *
                      100.0f, 0.0f, 100.0f);
}

static uint8_t app_calc_jack_stretch_level(float stretch_score)
{
    if (stretch_score < JACK_STRETCH_LV1_TH)
    {
        return 1;
    }
    else if (stretch_score < JACK_STRETCH_LV2_TH)
    {
        return 2;
    }
    else if (stretch_score < JACK_STRETCH_LV3_TH)
    {
        return 3;
    }
    else if (stretch_score < JACK_STRETCH_LV4_TH)
    {
        return 4;
    }
    return 5;
}

static void app_reset_jack_rep_stats(void)
{
    g_jack_rep_max_stretch_score = 0.0f;
    g_jack_rep_stretch_valid = 0;
}

static void app_update_jack_rep_stats(pose_features_t *features)
{
    if ((features == NULL) || (features->stretch_valid == 0) ||
        (features->stretch_score_filtered < 0.0f))
    {
        return;
    }

    if (features->stretch_score_filtered > g_jack_rep_max_stretch_score)
    {
        g_jack_rep_max_stretch_score = features->stretch_score_filtered;
    }
    g_jack_rep_stretch_valid = 1;
}

static uint16_t app_float_percent_to_u16(float value)
{
    value = app_clampf(value, 0.0f, 100.0f);
    return (uint16_t)(value + 0.5f);
}

static uint16_t app_u32_to_u16(uint32_t value)
{
    if (value > 65535U)
    {
        return 65535U;
    }
    return (uint16_t)value;
}


static void app_check_all_targets_voice(void)
{
    if ((g_all_target_voice_done == 0U) &&
        (g_squat_count >= SQUAT_TARGET_COUNT) &&
        (g_jack_count >= JACK_TARGET_COUNT) &&
        (g_plank_target_voice_done != 0U))
    {
        fibocom_voice_prompt("VOICE_ALL_DONE");
        g_all_target_voice_done = 1U;
    }
}

static void app_handle_squat_voice(void)
{
    if ((g_squat_target_voice_done == 0U) && (g_squat_count >= SQUAT_TARGET_COUNT))
    {
        fibocom_voice_prompt("VOICE_SQUAT_TARGET");
        g_squat_target_voice_done = 1U;
    }

    if ((g_squat_last_depth_valid != 0U) &&
        (g_squat_depth_voice_done == 0U) &&
        (g_squat_last_depth_percent < SQUAT_DEPTH_GOOD_TH))
    {
        fibocom_voice_prompt("VOICE_SQUAT_DEPTH");
        g_squat_depth_voice_done = 1U;
    }

    if (g_squat_last_lean_valid != 0U)
    {
        if ((g_squat_lean_low_voice_done == 0U) &&
            (g_squat_last_max_lean_angle < SQUAT_LEAN_GOOD_MIN_TH))
        {
            fibocom_voice_prompt("VOICE_LEAN_LOW");
            g_squat_lean_low_voice_done = 1U;
        }
        else if ((g_squat_lean_high_voice_done == 0U) &&
                 (g_squat_last_max_lean_angle > SQUAT_LEAN_GOOD_MAX_TH))
        {
            fibocom_voice_prompt("VOICE_LEAN_HIGH");
            g_squat_lean_high_voice_done = 1U;
        }
    }

    app_check_all_targets_voice();
}

static void app_handle_jack_voice(void)
{
    if ((g_jack_target_voice_done == 0U) && (g_jack_count >= JACK_TARGET_COUNT))
    {
        fibocom_voice_prompt("VOICE_JACK_TARGET");
        g_jack_target_voice_done = 1U;
    }

    if ((g_jack_last_stretch_valid != 0U) &&
        (g_jack_stretch_voice_done == 0U) &&
        (g_jack_last_stretch_score < JACK_STRETCH_GOOD_TH))
    {
        fibocom_voice_prompt("VOICE_JACK_STRETCH");
        g_jack_stretch_voice_done = 1U;
    }

    app_check_all_targets_voice();
}

static void app_handle_plank_voice(void)
{
    if ((g_plank_encourage_voice_done == 0U) && (g_plank_time_s >= PLANK_ENCOURAGE_TIME_S))
    {
        fibocom_voice_prompt("VOICE_PLANK_KEEP");
        g_plank_encourage_voice_done = 1U;
    }

    if ((g_plank_target_voice_done == 0U) && (g_plank_time_s >= PLANK_TARGET_TIME_S))
    {
        fibocom_voice_prompt("VOICE_PLANK_TARGET");
        g_plank_target_voice_done = 1U;
    }

    if ((g_plank_quality_valid != 0U) &&
        (g_plank_quality_voice_done == 0U) &&
        (g_plank_hip_drop_bad != 0U))
    {
        fibocom_voice_prompt("VOICE_PLANK_BAD");
        g_plank_quality_voice_done = 1U;
    }

    app_check_all_targets_voice();
}
static void app_report_squat_params(void)
{
    fibocom_report_property(1U, app_u32_to_u16(g_squat_count));

    if (g_squat_last_depth_valid != 0U)
    {
        fibocom_report_property(3U, app_float_percent_to_u16(g_squat_last_depth_percent));
    }

    if (g_squat_last_lean_valid != 0U)
    {
        fibocom_report_property(4U, app_float_percent_to_u16(g_squat_last_max_lean_angle));
    }
}

static void app_report_jack_params(void)
{
    fibocom_report_property(2U, app_u32_to_u16(g_jack_count));

    if (g_jack_last_stretch_valid != 0U)
    {
        fibocom_report_property(5U, app_float_percent_to_u16(g_jack_last_stretch_score));
    }
}

static void app_report_plank_params(void)
{
    fibocom_report_property(6U, app_u32_to_u16(g_plank_time_s));

    if (g_plank_quality_valid != 0U)
    {
        fibocom_report_property(7U, g_plank_quality_level);
    }
}

static void app_commit_jack_rep_stats(void)
{
    if (g_jack_rep_stretch_valid)
    {
        g_jack_last_stretch_score = g_jack_rep_max_stretch_score;
        g_jack_last_stretch_level = app_calc_jack_stretch_level(g_jack_rep_max_stretch_score);
        g_jack_last_stretch_valid = 1;
    }
}


static void app_camera_display_pipe_vsync_cb(void)
{
    tx_semaphore_put(&isp_semaphore);
}

static void app_camera_display_pipe_frame_cb(void)
{
    app_lcd_switch_bg_buffer();
    app_camera_display_pipe_set_address(app_lcd_get_bg_buffer());
}

static void app_camera_nn_pipe_frame_cb(void)
{
    uint8_t *buffer;

    buffer = app_bqueue_get_free(&nn_input_queue, 0);
    if (buffer != NULL)
    {
        app_camera_nn_pipe_set_address(buffer);
        app_bqueue_put_ready(&nn_input_queue);
    }
}

static VOID nn_thread_entry(ULONG id)
{
    uint32_t nn_out_len;
    uint32_t nn_in_len;
    uint8_t *nn_pipe_dst;
    uint8_t *capture_buffer;
    uint8_t *output_buffer;
    uint32_t nn_period[2];
    uint32_t nn_period_ms;
    uint32_t time_stamp;
    uint32_t inf_ms;

    nn_in_len = LL_Buffer_len(LL_ATON_Input_Buffers_Info_Default());
    nn_out_len = LL_Buffer_len(LL_ATON_Output_Buffers_Info_Default());

    nn_period[1] = HAL_GetTick();

    nn_pipe_dst = app_bqueue_get_free(&nn_input_queue, 0);

    app_camera_nn_pipe_start(nn_pipe_dst, CMW_MODE_CONTINUOUS);

    while (1)
    {
        nn_period[0] = nn_period[1];
        nn_period[1] = HAL_GetTick();
        nn_period_ms = nn_period[1] - nn_period[0];

        capture_buffer = app_bqueue_get_ready(&nn_input_queue);
        output_buffer = app_bqueue_get_free(&nn_output_queue, 1);

        time_stamp = HAL_GetTick();
        LL_ATON_Set_User_Input_Buffer_Default(0, capture_buffer, nn_in_len);
        SCB_InvalidateDCache_by_Addr(output_buffer, nn_out_len);
        LL_ATON_Set_User_Output_Buffer_Default(0, output_buffer, nn_out_len);
        LL_ATON_RT_Main(&NN_Instance_Default);
        inf_ms = HAL_GetTick() - time_stamp;

        app_bqueue_put_free(&nn_input_queue);
        app_bqueue_put_ready(&nn_output_queue);

        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display.info.inf_ms = inf_ms;
        display.info.nn_period_ms = nn_period_ms;
        tx_mutex_put(&display.lock);
    }
}

static VOID pp_thread_entry(ULONG id)
{
    mpe_yolov8_pp_static_param_t pp_params;
    uint8_t *output_buffer;
    mpe_pp_out_t pp_output;
    uint32_t nn_pp[2];
    int32_t i;

    app_postprocess_init(&pp_params);

    while (1)
    {
        output_buffer = app_bqueue_get_ready(&nn_output_queue);
        pp_output.pOutBuff = NULL;

        nn_pp[0] = HAL_GetTick();
        app_postprocess_run((void *[]){(void *)output_buffer}, 1, &pp_output, &pp_params);
        nn_pp[1] = HAL_GetTick();

        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display.info.nb_detect = pp_output.nb_detect;
        for (i = 0; i < pp_output.nb_detect; i++)
        {
            display.info.detects[i] = pp_output.pOutBuff[i];
        }
        app_extract_pose_features(display.info.detects,
                                  display.info.nb_detect,
                                  &display.info.features);
        app_update_action_logic(&display.info.features, HAL_GetTick());
        app_update_display_action_info(&display.info);
        display.info.pp_ms = nn_pp[1] - nn_pp[0];
        tx_mutex_put(&display.lock);
        app_bqueue_put_free(&nn_output_queue);
        tx_semaphore_ceiling_put(&display.update, 1);
    }
}

static VOID dp_thread_entry(ULONG id)
{
    uint32_t disp_ms = 0;
    app_display_info_t display_info;
    uint32_t time_stamp;

    while (1)
    {
        tx_semaphore_get(&display.update, TX_WAIT_FOREVER);
        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display_info = display.info;
        tx_mutex_put(&display.lock);
        display_info.disp_ms = disp_ms;

        time_stamp = HAL_GetTick();
        app_display_network_output(&display_info);
        disp_ms = HAL_GetTick() - time_stamp;
    }
}

static VOID isp_thread_entry(ULONG id)
{
    while (1)
    {
        tx_semaphore_get(&isp_semaphore, TX_WAIT_FOREVER);

        app_camera_isp_update();
    }
}

static const char *app_action_str(action_type_t action)
{
    switch (action)
    {
        case ACTION_SQUAT:
            return "Squat";

        case ACTION_JUMPING_JACK:
            return "Jack";

        case ACTION_PLANK:
            return "Plank";

        case ACTION_NONE:
        default:
            return "None";
    }
}

static int32_t app_select_main_person(mpe_pp_outBuffer_t *detects, int32_t nb_detect)
{
    int32_t best = -1;
    float best_area = 0.0f;

    if ((detects == NULL) || (nb_detect <= 0))
    {
        return -1;
    }

    for (int32_t i = 0; i < nb_detect; i++)
    {
        float area = detects[i].width * detects[i].height;

        if (area > best_area)
        {
            best_area = area;
            best = i;
        }
    }

    return best;
}

static uint8_t app_key_valid(mpe_pp_keyPoints_t *key, float th)
{
    if (key == NULL)
    {
        return 0;
    }

    return key->conf >= th;
}

static float app_calc_angle(mpe_pp_keyPoints_t *a, mpe_pp_keyPoints_t *b, mpe_pp_keyPoints_t *c)
{
    float ax, ay, bx, by, cx, cy;
    float bax, bay, bcx, bcy;
    float dot;
    float norm1;
    float norm2;
    float cosv;

    if ((a == NULL) || (b == NULL) || (c == NULL))
    {
        return -1.0f;
    }

    ax = a->x * LCD_BG_WIDTH;
    ay = a->y * LCD_BG_HEIGHT;
    bx = b->x * LCD_BG_WIDTH;
    by = b->y * LCD_BG_HEIGHT;
    cx = c->x * LCD_BG_WIDTH;
    cy = c->y * LCD_BG_HEIGHT;

    bax = ax - bx;
    bay = ay - by;
    bcx = cx - bx;
    bcy = cy - by;

    dot = bax * bcx + bay * bcy;
    norm1 = sqrtf(bax * bax + bay * bay);
    norm2 = sqrtf(bcx * bcx + bcy * bcy);

    if ((norm1 < 1e-3f) || (norm2 < 1e-3f))
    {
        return -1.0f;
    }

    cosv = dot / (norm1 * norm2);

    if (cosv > 1.0f)
    {
        cosv = 1.0f;
    }
    else if (cosv < -1.0f)
    {
        cosv = -1.0f;
    }

    return acosf(cosv) * 57.2957795f;
}

static uint8_t app_calc_plank_quality_level(uint8_t hip_drop_bad)
{
    return hip_drop_bad ? 1U : 5U;
}

static void app_reset_plank_state(void)
{
    g_plank_start_ms = 0;
    g_plank_last_seen_ms = 0;
}

static uint8_t app_calc_plank_pose(mpe_pp_outBuffer_t *p, pose_features_t *features)
{
    float sx;
    float sy;
    float hx;
    float hy;
    float ky;
    float ax;
    float ay;
    float body_dx;
    float body_dy;
    float body_len;
    float hip_to_line;
    float lower_y;
    float hip_lower_gap;
    float shoulder_lower_gap;
    float elbow_y = 0.0f;
    float wrist_y = 0.0f;
    float shoulder_y = 0.0f;
    uint8_t arm_pairs = 0;

    if ((p == NULL) || (features == NULL))
    {
        return 0;
    }

    if (!(app_key_valid(&p->pKeyPoints[KP_LEFT_SHOULDER], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_RIGHT_SHOULDER], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_LEFT_HIP], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_RIGHT_HIP], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_LEFT_KNEE], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_RIGHT_KNEE], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_LEFT_ANKLE], KP_CONF_TH_BODY) &&
          app_key_valid(&p->pKeyPoints[KP_RIGHT_ANKLE], KP_CONF_TH_BODY)))
    {
        return 0;
    }

    sx = 0.5f * (p->pKeyPoints[KP_LEFT_SHOULDER].x + p->pKeyPoints[KP_RIGHT_SHOULDER].x) * LCD_BG_WIDTH;
    sy = 0.5f * (p->pKeyPoints[KP_LEFT_SHOULDER].y + p->pKeyPoints[KP_RIGHT_SHOULDER].y) * LCD_BG_HEIGHT;
    hx = 0.5f * (p->pKeyPoints[KP_LEFT_HIP].x + p->pKeyPoints[KP_RIGHT_HIP].x) * LCD_BG_WIDTH;
    hy = 0.5f * (p->pKeyPoints[KP_LEFT_HIP].y + p->pKeyPoints[KP_RIGHT_HIP].y) * LCD_BG_HEIGHT;
    ky = 0.5f * (p->pKeyPoints[KP_LEFT_KNEE].y + p->pKeyPoints[KP_RIGHT_KNEE].y) * LCD_BG_HEIGHT;
    ax = 0.5f * (p->pKeyPoints[KP_LEFT_ANKLE].x + p->pKeyPoints[KP_RIGHT_ANKLE].x) * LCD_BG_WIDTH;
    ay = 0.5f * (p->pKeyPoints[KP_LEFT_ANKLE].y + p->pKeyPoints[KP_RIGHT_ANKLE].y) * LCD_BG_HEIGHT;

    body_dx = fabsf(ax - sx);
    body_dy = fabsf(ay - sy);
    body_len = sqrtf(body_dx * body_dx + body_dy * body_dy);
    if ((body_len < 20.0f) || (body_dx < (PLANK_MIN_HORIZONTAL_SPAN * LCD_BG_WIDTH)))
    {
        return 0;
    }

    features->plank_body_angle = atan2f(body_dy, body_dx) * 57.2957795f;
    hip_to_line = fabsf((ay - sy) * hx - (ax - sx) * hy + ax * sy - ay * sx) / body_len;
    features->plank_line_score = 100.0f - (hip_to_line / body_len) * 300.0f;
    if (features->plank_line_score < 0.0f)
    {
        features->plank_line_score = 0.0f;
    }
    else if (features->plank_line_score > 100.0f)
    {
        features->plank_line_score = 100.0f;
    }

    lower_y = 0.5f * (ky + ay);
    hip_lower_gap = fabsf(hy - lower_y);
    shoulder_lower_gap = fabsf(sy - lower_y);
    features->plank_hip_drop_bad =
        (shoulder_lower_gap > 1.0f) &&
        (hip_lower_gap <= (shoulder_lower_gap * PLANK_HIP_DROP_CLOSE_RATIO));

    if (app_key_valid(&p->pKeyPoints[KP_LEFT_SHOULDER], KP_CONF_TH_BODY) &&
        app_key_valid(&p->pKeyPoints[KP_LEFT_ELBOW], KP_CONF_TH_ARM) &&
        app_key_valid(&p->pKeyPoints[KP_LEFT_WRIST], KP_CONF_TH_ARM))
    {
        shoulder_y += p->pKeyPoints[KP_LEFT_SHOULDER].y;
        elbow_y += p->pKeyPoints[KP_LEFT_ELBOW].y;
        wrist_y += p->pKeyPoints[KP_LEFT_WRIST].y;
        arm_pairs++;
    }

    if (app_key_valid(&p->pKeyPoints[KP_RIGHT_SHOULDER], KP_CONF_TH_BODY) &&
        app_key_valid(&p->pKeyPoints[KP_RIGHT_ELBOW], KP_CONF_TH_ARM) &&
        app_key_valid(&p->pKeyPoints[KP_RIGHT_WRIST], KP_CONF_TH_ARM))
    {
        shoulder_y += p->pKeyPoints[KP_RIGHT_SHOULDER].y;
        elbow_y += p->pKeyPoints[KP_RIGHT_ELBOW].y;
        wrist_y += p->pKeyPoints[KP_RIGHT_WRIST].y;
        arm_pairs++;
    }

    if (arm_pairs == 0U)
    {
        return 0;
    }

    shoulder_y /= arm_pairs;
    elbow_y /= arm_pairs;
    wrist_y /= arm_pairs;
    features->plank_elbow_ground =
        ((elbow_y - shoulder_y) > PLANK_SHOULDER_ARM_Y_MARGIN) &&
        ((wrist_y - shoulder_y) > PLANK_SHOULDER_ARM_Y_MARGIN) &&
        (fabsf(elbow_y - wrist_y) < PLANK_ELBOW_WRIST_Y_MARGIN);

    features->plank_valid = 1;
    if ((features->plank_body_angle <= PLANK_BODY_MAX_ANGLE) &&
        features->plank_elbow_ground)
    {
        features->plank_candidate = 1;
    }

    return 1;
}


static void app_extract_pose_features(mpe_pp_outBuffer_t *detects,
                                      int32_t nb_detect,
                                      pose_features_t *features)
{
    int32_t idx;
    mpe_pp_outBuffer_t *person;
    float left_angle = -1.0f;
    float right_angle = -1.0f;
    float torso_v_angle = -1.0f;
    float leg_angle = -1.0f;

    if (features == NULL)
    {
        return;
    }

    memset(features, 0, sizeof(*features));
    features->left_knee_angle = -1.0f;
    features->right_knee_angle = -1.0f;
    features->knee_angle = -1.0f;
    features->knee_angle_filtered = -1.0f;
    features->ankle_ratio = -1.0f;
    features->ankle_ratio_filtered = -1.0f;
    features->torso_v_angle = -1.0f;
    features->torso_v_angle_filtered = -1.0f;
    features->leg_angle = -1.0f;
    features->leg_angle_filtered = -1.0f;
    features->stretch_score = -1.0f;
    features->stretch_score_filtered = -1.0f;
    features->plank_body_angle = -1.0f;
    features->plank_line_score = -1.0f;

    idx = app_select_main_person(detects, nb_detect);
    if (idx < 0)
    {
        return;
    }

    person = &detects[idx];

    if (app_calc_torso_v_angle(person, &torso_v_angle))
    {
        features->torso_v_valid = 1;
        features->torso_v_angle = torso_v_angle;
        if (g_torso_v_filter_valid == 0)
        {
            g_torso_v_angle_filtered = torso_v_angle;
            g_torso_v_filter_valid = 1;
        }
        else
        {
            g_torso_v_angle_filtered = ANGLE_EMA_ALPHA * torso_v_angle +
                                       (1.0f - ANGLE_EMA_ALPHA) * g_torso_v_angle_filtered;
        }
        features->torso_v_angle_filtered = g_torso_v_angle_filtered;
    }

    if (app_calc_leg_spread_angle(person, &leg_angle))
    {
        features->leg_angle_valid = 1;
        features->leg_angle = leg_angle;
        if (g_leg_angle_filter_valid == 0)
        {
            g_leg_angle_filtered = leg_angle;
            g_leg_angle_filter_valid = 1;
        }
        else
        {
            g_leg_angle_filtered = LEG_ANGLE_EMA_ALPHA * leg_angle +
                                   (1.0f - LEG_ANGLE_EMA_ALPHA) * g_leg_angle_filtered;
        }
        features->leg_angle_filtered = g_leg_angle_filtered;
        features->stretch_score = app_calc_jack_stretch_from_leg_angle(g_leg_angle_filtered);
        features->stretch_valid = 1;
        if (g_stretch_filter_valid == 0)
        {
            g_stretch_score_filtered = features->stretch_score;
            g_stretch_filter_valid = 1;
        }
        else
        {
            g_stretch_score_filtered = STRETCH_EMA_ALPHA * features->stretch_score +
                                       (1.0f - STRETCH_EMA_ALPHA) * g_stretch_score_filtered;
        }
        features->stretch_score_filtered = g_stretch_score_filtered;
    }

    if (app_key_valid(&person->pKeyPoints[KP_LEFT_HIP], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_LEFT_KNEE], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_LEFT_ANKLE], KP_CONF_TH_BODY))
    {
        left_angle = app_calc_angle(&person->pKeyPoints[KP_LEFT_HIP],
                                    &person->pKeyPoints[KP_LEFT_KNEE],
                                    &person->pKeyPoints[KP_LEFT_ANKLE]);
    }

    if (app_key_valid(&person->pKeyPoints[KP_RIGHT_HIP], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_RIGHT_KNEE], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_RIGHT_ANKLE], KP_CONF_TH_BODY))
    {
        right_angle = app_calc_angle(&person->pKeyPoints[KP_RIGHT_HIP],
                                     &person->pKeyPoints[KP_RIGHT_KNEE],
                                     &person->pKeyPoints[KP_RIGHT_ANKLE]);
    }

    features->left_knee_angle = left_angle;
    features->right_knee_angle = right_angle;

    if ((left_angle > 0.0f) && (right_angle > 0.0f))
    {
        features->knee_angle = 0.5f * (left_angle + right_angle);
        features->knee_valid = 1;
    }
    else if (left_angle > 0.0f)
    {
        features->knee_angle = left_angle;
        features->knee_valid = 1;
    }
    else if (right_angle > 0.0f)
    {
        features->knee_angle = right_angle;
        features->knee_valid = 1;
    }

    if (features->knee_valid)
    {
        if (g_knee_filter_valid == 0)
        {
            g_knee_angle_filtered = features->knee_angle;
            g_knee_filter_valid = 1;
        }
        else
        {
            g_knee_angle_filtered =
                ANGLE_EMA_ALPHA * features->knee_angle +
                (1.0f - ANGLE_EMA_ALPHA) * g_knee_angle_filtered;
        }

        features->knee_angle_filtered = g_knee_angle_filtered;
    }

    if (app_key_valid(&person->pKeyPoints[KP_LEFT_SHOULDER], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_RIGHT_SHOULDER], KP_CONF_TH_BODY))
    {
        features->shoulder_width = fabsf(person->pKeyPoints[KP_LEFT_SHOULDER].x -
                                         person->pKeyPoints[KP_RIGHT_SHOULDER].x);
        if (features->shoulder_width > 0.03f)
        {
            features->shoulder_valid = 1;
        }
    }

    if (features->shoulder_valid &&
        app_key_valid(&person->pKeyPoints[KP_LEFT_ANKLE], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_RIGHT_ANKLE], KP_CONF_TH_BODY))
    {
        float ankle_width = fabsf(person->pKeyPoints[KP_LEFT_ANKLE].x -
                                  person->pKeyPoints[KP_RIGHT_ANKLE].x);
        features->ankle_ratio = ankle_width / features->shoulder_width;
        features->ankle_valid = 1;

        if (g_ankle_filter_valid == 0)
        {
            g_ankle_ratio_filtered = features->ankle_ratio;
            g_ankle_filter_valid = 1;
        }
        else
        {
            g_ankle_ratio_filtered =
                RATIO_EMA_ALPHA * features->ankle_ratio +
                (1.0f - RATIO_EMA_ALPHA) * g_ankle_ratio_filtered;
        }

        features->ankle_ratio_filtered = g_ankle_ratio_filtered;
    }

    if (app_key_valid(&person->pKeyPoints[KP_LEFT_SHOULDER], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_LEFT_WRIST], KP_CONF_TH_ARM))
    {
        features->left_wrist_up =
            person->pKeyPoints[KP_LEFT_WRIST].y <
            (person->pKeyPoints[KP_LEFT_SHOULDER].y + JACK_WRIST_UP_MARGIN);
    }

    if (app_key_valid(&person->pKeyPoints[KP_RIGHT_SHOULDER], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_RIGHT_WRIST], KP_CONF_TH_ARM))
    {
        features->right_wrist_up =
            person->pKeyPoints[KP_RIGHT_WRIST].y <
            (person->pKeyPoints[KP_RIGHT_SHOULDER].y + JACK_WRIST_UP_MARGIN);
    }

    if (app_key_valid(&person->pKeyPoints[KP_LEFT_SHOULDER], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_LEFT_ELBOW], KP_CONF_TH_ARM))
    {
        features->left_elbow_up =
            person->pKeyPoints[KP_LEFT_ELBOW].y <
            (person->pKeyPoints[KP_LEFT_SHOULDER].y + JACK_ELBOW_UP_MARGIN);
    }

    if (app_key_valid(&person->pKeyPoints[KP_RIGHT_SHOULDER], KP_CONF_TH_BODY) &&
        app_key_valid(&person->pKeyPoints[KP_RIGHT_ELBOW], KP_CONF_TH_ARM))
    {
        features->right_elbow_up =
            person->pKeyPoints[KP_RIGHT_ELBOW].y <
            (person->pKeyPoints[KP_RIGHT_SHOULDER].y + JACK_ELBOW_UP_MARGIN);
    }

    features->arms_up_strong = features->left_wrist_up && features->right_wrist_up;
    features->arm_evidence = features->left_wrist_up ||
                             features->right_wrist_up ||
                             features->left_elbow_up ||
                             features->right_elbow_up;
    features->arm_valid = features->arm_evidence;
    features->arms_up = features->arm_evidence;

    if (features->ankle_valid)
    {
        if ((features->ankle_ratio_filtered > JACK_OPEN_RATIO_STRONG_TH) &&
            features->arms_up_strong)
        {
            features->jack_strong_candidate = 1;
            features->jack_candidate = 1;
            features->jack_candidate_strength = 3;
        }
        else if (features->ankle_ratio_filtered > JACK_OPEN_RATIO_LEG_ONLY_TH)
        {
            features->jack_leg_only_candidate = 1;
            features->jack_candidate = 1;
            features->jack_candidate_strength = 2;
        }
        else if ((features->ankle_ratio_filtered > JACK_OPEN_RATIO_MID_TH) &&
                 features->arm_evidence)
        {
            features->jack_mid_candidate = 1;
            features->jack_candidate = 1;
            features->jack_candidate_strength = 1;
        }
    }

    if (features->knee_valid &&
        (features->knee_angle_filtered < SQUAT_DOWN_TH) &&
        ((features->ankle_valid == 0) ||
         (features->ankle_ratio_filtered < SQUAT_LOCK_ANKLE_MAX_RATIO)))
    {
        features->squat_candidate = 1;
    }

    (void)app_calc_plank_pose(person, features);

    features->valid = features->knee_valid || features->ankle_valid ||
                      features->arm_valid || features->torso_v_valid ||
                      features->leg_angle_valid || features->stretch_valid ||
                      features->plank_valid;
}

static action_type_t app_select_action_candidate(pose_features_t *features)
{
    if (features == NULL)
    {
        return ACTION_NONE;
    }

    if (features->jack_candidate)
    {
        return ACTION_JUMPING_JACK;
    }

    if (features->squat_candidate)
    {
        return ACTION_SQUAT;
    }

    if (features->plank_candidate)
    {
        return ACTION_PLANK;
    }

    return ACTION_NONE;
}
static uint8_t app_get_jack_required_confirm(pose_features_t *features)
{
    if (features == NULL)
    {
        return 255U;
    }

    if (features->jack_strong_candidate)
    {
        return JACK_STRONG_CONFIRM_FRAMES;
    }

    if (features->jack_leg_only_candidate)
    {
        return JACK_LEG_ONLY_CONFIRM_FRAMES;
    }

    if (features->jack_mid_candidate)
    {
        return JACK_MID_CONFIRM_FRAMES;
    }

    return 255U;
}

static void app_update_action_logic(pose_features_t *features, uint32_t now_ms)
{
    action_type_t cand;
    uint8_t required_confirm;

    if (features == NULL)
    {
        return;
    }

    if ((g_locked_action != ACTION_NONE) && (features->valid == 0))
    {
        uint32_t lost_ms;

        if (g_lost_active == 0)
        {
            g_lost_active = 1;
            g_lost_start_ms = now_ms;
        }

        lost_ms = now_ms - g_lost_start_ms;
        if (lost_ms <= ACTION_LOST_TIMEOUT_MS)
        {
            return;
        }

        g_locked_action = ACTION_NONE;
        g_candidate_action = ACTION_NONE;
        g_candidate_confirm = 0;
        g_squat_state = SQUAT_STATE_UNKNOWN;
        g_squat_armed = 0;
        g_squat_stand_confirm = 0;
        g_jack_state = JACK_STATE_UNKNOWN;
        g_jack_armed = 0;
        g_jack_close_confirm = 0;
        app_reset_squat_rep_stats();
        app_reset_jack_rep_stats();
        app_reset_plank_state();
        g_lost_active = 0;
        g_lost_start_ms = 0;
        return;
    }

    if (features->valid)
    {
        g_lost_active = 0;
        g_lost_start_ms = 0;
    }

    if (g_locked_action == ACTION_NONE)
    {
        cand = app_select_action_candidate(features);
        if (cand == ACTION_NONE)
        {
            g_candidate_action = ACTION_NONE;
            g_candidate_confirm = 0;
            return;
        }

        if (cand == g_candidate_action)
        {
            if (g_candidate_confirm < 255U)
            {
                g_candidate_confirm++;
            }
        }
        else
        {
            g_candidate_action = cand;
            g_candidate_confirm = 1;
        }
        if (cand == ACTION_JUMPING_JACK)
        {
            required_confirm = app_get_jack_required_confirm(features);
        }
        else if (cand == ACTION_PLANK)
        {
            required_confirm = PLANK_CONFIRM_FRAMES;
        }
        else
        {
            required_confirm = ACTION_CANDIDATE_FRAMES;
        }

        if (g_candidate_confirm >= required_confirm)
        {
            g_locked_action = cand;
            g_last_action = cand;
            g_action_lock_start_ms = now_ms;
            g_candidate_action = ACTION_NONE;
            g_candidate_confirm = 0;

            if (cand == ACTION_SQUAT)
            {
                g_squat_state = SQUAT_STATE_DOWN;
                g_squat_armed = 1;
                g_squat_stand_confirm = 0;
                app_reset_squat_rep_stats();
                app_update_squat_rep_stats(features);
            }
            else if (cand == ACTION_JUMPING_JACK)
            {
                g_jack_state = JACK_STATE_OPEN;
                g_jack_armed = 1;
                g_jack_close_confirm = 0;
                app_reset_jack_rep_stats();
                app_update_jack_rep_stats(features);
            }
            else if (cand == ACTION_PLANK)
            {
                g_plank_start_ms = now_ms;
                g_plank_last_seen_ms = now_ms;
                g_plank_time_s = 0;
                g_plank_body_angle = features->plank_body_angle;
                g_plank_line_score = features->plank_line_score;
                g_plank_hip_drop_bad = features->plank_hip_drop_bad;
                g_plank_quality_level = app_calc_plank_quality_level(g_plank_hip_drop_bad);
                g_plank_quality_valid = 1;
            }
        }

        return;
    }

    if ((g_locked_action != ACTION_PLANK) &&
        ((now_ms - g_action_lock_start_ms) > ACTION_LOCK_TIMEOUT_MS))
    {
        g_locked_action = ACTION_NONE;
        g_squat_state = SQUAT_STATE_UNKNOWN;
        g_squat_armed = 0;
        g_squat_stand_confirm = 0;
        g_jack_state = JACK_STATE_UNKNOWN;
        g_jack_armed = 0;
        g_jack_close_confirm = 0;
        app_reset_squat_rep_stats();
        app_reset_jack_rep_stats();
        app_reset_plank_state();
        return;
    }

    if (g_locked_action == ACTION_SQUAT)
    {
        app_update_squat_rep_stats(features);

        if (features->knee_valid && (features->knee_angle_filtered > SQUAT_STAND_TH))
        {
            if (g_squat_stand_confirm < 255U)
            {
                g_squat_stand_confirm++;
            }

            if (g_squat_stand_confirm >= ACTION_CONFIRM_FRAMES)
            {
                if (g_squat_armed)
                {
                    app_commit_squat_rep_stats();
                    g_squat_count++;
                    app_report_squat_params();
                    app_handle_squat_voice();
                }

                g_squat_state = SQUAT_STATE_STAND;
                g_squat_armed = 0;
                g_squat_stand_confirm = 0;
                g_last_action = ACTION_SQUAT;
                g_locked_action = ACTION_NONE;
                app_reset_squat_rep_stats();
            }
        }
        else
        {
            g_squat_stand_confirm = 0;
        }
    }
    else if (g_locked_action == ACTION_JUMPING_JACK)
    {
        app_update_jack_rep_stats(features);

        if (features->ankle_valid && (features->ankle_ratio_filtered < JACK_CLOSE_RATIO_TH))
        {
            if (g_jack_close_confirm < 255U)
            {
                g_jack_close_confirm++;
            }

            if (g_jack_close_confirm >= ACTION_CONFIRM_FRAMES)
            {
                if (g_jack_armed)
                {
                    app_commit_jack_rep_stats();
                    g_jack_count++;
                    app_report_jack_params();
                    app_handle_jack_voice();
                }

                g_jack_state = JACK_STATE_UNKNOWN;
                g_jack_armed = 0;
                g_jack_close_confirm = 0;
                g_last_action = ACTION_JUMPING_JACK;
                g_locked_action = ACTION_NONE;
                app_reset_jack_rep_stats();
            }
        }
        else
        {
            g_jack_close_confirm = 0;
        }
    }
    else if (g_locked_action == ACTION_PLANK)
    {
        if (features->plank_candidate)
        {
            g_plank_last_seen_ms = now_ms;
            g_plank_time_s = (now_ms - g_plank_start_ms) / 1000U;
            g_plank_body_angle = features->plank_body_angle;
            g_plank_line_score = features->plank_line_score;
            g_plank_hip_drop_bad = features->plank_hip_drop_bad;
            g_plank_quality_level = app_calc_plank_quality_level(g_plank_hip_drop_bad);
            g_plank_quality_valid = 1;
            app_handle_plank_voice();
            g_last_action = ACTION_PLANK;
        }
        else if ((now_ms - g_plank_last_seen_ms) > ACTION_LOST_TIMEOUT_MS)
        {
            app_report_plank_params();
            g_last_action = ACTION_PLANK;
            g_locked_action = ACTION_NONE;
            app_reset_plank_state();
        }
    }
}

static void app_update_display_action_info(app_display_info_t *info)
{
    action_type_t active_action;

    if (info == NULL)
    {
        return;
    }

    info->locked_action = g_locked_action;
    info->last_action = g_last_action;
    info->squat_count = g_squat_count;
    info->jack_count = g_jack_count;
    info->plank_time_s = g_plank_time_s;
    info->squat_last_depth_percent = g_squat_last_depth_percent;
    info->squat_last_depth_valid = g_squat_last_depth_valid;
    info->squat_last_lean_level = g_squat_last_lean_level;
    info->squat_last_max_lean_angle = g_squat_last_max_lean_angle;
    info->squat_last_lean_valid = g_squat_last_lean_valid;
    info->jack_last_stretch_score = g_jack_last_stretch_score;
    info->jack_last_stretch_level = g_jack_last_stretch_level;
    info->jack_last_stretch_valid = g_jack_last_stretch_valid;
    info->plank_quality_valid = g_plank_quality_valid;
    info->plank_quality_level = g_plank_quality_level;
    info->plank_hip_drop_bad = g_plank_hip_drop_bad;
    info->plank_body_angle = g_plank_body_angle;
    info->plank_line_score = g_plank_line_score;
    info->action_tracking = (g_locked_action != ACTION_NONE) ? 1U : 0U;
    info->action_lost_ms = g_lost_active ? (HAL_GetTick() - g_lost_start_ms) : 0U;

    active_action = (g_locked_action != ACTION_NONE) ? g_locked_action : g_last_action;
    if (active_action == ACTION_SQUAT)
    {
        info->current_count = g_squat_count;
    }
    else if (active_action == ACTION_JUMPING_JACK)
    {
        info->current_count = g_jack_count;
    }
    else if (active_action == ACTION_PLANK)
    {
        info->current_count = g_plank_time_s;
    }
    else
    {
        info->current_count = 0;
    }
}
static void app_convert_length(float32_t wi, float32_t hi, int32_t *wo, int32_t *ho)
{
    *wo = (int32_t)(LCD_BG_WIDTH * wi);
    *ho = (int32_t)(LCD_BG_HEIGHT * hi);
}

static void app_convert_point(float32_t xi, float32_t yi, int32_t *xo, int32_t *yo)
{
    *xo = (int32_t)(LCD_BG_WIDTH * xi);
    *yo = (int32_t)(LCD_BG_HEIGHT * yi);
}

static uint8_t app_clamp_point(int32_t *x, int32_t *y)
{
    int32_t xi;
    int32_t yi;

    xi = *x;
    yi = *y;

    if (*x < 0)
    {
        *x = 0;
    }

    if (*y < 0)
    {
        *y = 0;
    }

    if (*x >= LCD_BG_WIDTH)
    {
        *x = LCD_BG_WIDTH - 1;
    }

    if (*y >= LCD_BG_HEIGHT)
    {
        *y = LCD_BG_HEIGHT - 1;
    }

    return (xi != *x) || (yi != *y);
}

static void app_display_binding_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color)
{
    app_clamp_point(&x0, &y0);
    app_clamp_point(&x1, &y1);

    UTIL_LCD_DrawLine(x0, y0, x1, y1, color);
}

static void app_display_binding(mpe_pp_keyPoints_t *from, mpe_pp_keyPoints_t *to, uint32_t color)
{
    int32_t x0, y0;
    int32_t x1, y1;
    int i;

    if ((from->conf < AI_MPE_YOLOV8_PP_CONF_THRESHOLD) || (to->conf < AI_MPE_YOLOV8_PP_CONF_THRESHOLD))
    {
        return;
    }

    app_convert_point(from->x, from->y, &x0, &y0);
    if (app_clamp_point(&x0, &y0) != 0)
    {
        return;
    }

    app_convert_point(to->x, to->y, &x1, &y1);
    if (app_clamp_point(&x1, &y1) != 0)
    {
        return;
    }

    UTIL_LCD_DrawLine(x0, y0, x1, y1, color);
    for (i = 1; i <= (BINDING_WIDTH - 1) / 2; i++)
    {
        if (abs(y1 - y0) > abs(x1 - x0))
        {
            app_display_binding_line(x0 + i, y0, x1 + i , y1, color);
            app_display_binding_line(x0 - i, y0, x1 - i , y1, color);
        }
        else
        {
            app_display_binding_line(x0, y0 + i, x1 , y1 + i, color);
            app_display_binding_line(x0, y0 - i, x1 , y1 - i, color);
        }
    }
}

static void app_display_keypoint(mpe_pp_keyPoints_t *key, uint32_t color)
{
    int32_t x;
    int32_t y;
    int32_t xc;
    int32_t yc;

    if (key->conf < AI_MPE_YOLOV8_PP_CONF_THRESHOLD)
    {
        return;
    }

    app_convert_point(key->x, key->y, &x, &y);

    xc = x - CIRCLE_RADIUS / 2;
    yc = y - CIRCLE_RADIUS / 2;
    if (app_clamp_point(&xc, &yc) != 0)
    {
        return;
    }

    xc = x + CIRCLE_RADIUS / 2;
    yc = y + CIRCLE_RADIUS / 2;
    if (app_clamp_point(&xc, &yc) != 0)
    {
        return;
    }

    UTIL_LCD_FillCircle(x, y, CIRCLE_RADIUS, color);
}

static void app_display_detection(mpe_pp_outBuffer_t *detect)
{
    int32_t xc;
    int32_t yc;
    int32_t x0;
    int32_t y0;
    int32_t x1;
    int32_t y1;
    int32_t w;
    int32_t h;
    uint8_t i;

    app_convert_point(detect->x_center, detect->y_center, &xc, &yc);
    app_convert_length(detect->width, detect->height, &w, &h);

    x0 = xc - (w + 1) / 2;
    y0 = yc - (h + 1) / 2;
    x1 = xc + (w + 1) / 2;
    y1 = yc + (h + 1) / 2;

    app_clamp_point(&x0, &y0);
    app_clamp_point(&x1, &y1);

    UTIL_LCD_DrawRect(x0, y0, x1 - x0, y1 - y0, UTIL_LCD_COLOR_GREEN);
    UTIL_LCDEx_PrintfAt(x0, y0, LEFT_MODE, nn_classes_table[detect->class_index]);

    for (i = 0; i < ARRAY_NB(bindings); i++)
    {
        app_display_binding(&detect->pKeyPoints[bindings[i][0]], &detect->pKeyPoints[bindings[i][1]], bindings[i][2]);
    }

    for (i = 0; i < AI_POSE_PP_POSE_KEYPOINTS_NB; i++)
    {
        app_display_keypoint(&detect->pKeyPoints[i], kp_color[i]);
    }
}

static void app_display_network_output(app_display_info_t *display_info)
{
    uint8_t line_nb = 0;
    int32_t i;
    action_type_t show_action;
    const uint32_t panel_x = 8U;

    app_lcd_draw_area_update();

    UTIL_LCD_FillRect(0, 0, LCD_FG_WIDTH, LCD_FG_HEIGHT, 0x00000000);

    show_action = (display_info->locked_action != ACTION_NONE) ?
                  display_info->locked_action : display_info->last_action;

    UTIL_LCD_SetFont(&Font24);
    UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Action");
    UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%s", app_action_str(show_action));

    if (show_action == ACTION_SQUAT)
    {
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Count");
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%lu",
                            (unsigned long)display_info->squat_count);
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Knee");
        if (display_info->features.knee_valid &&
            (display_info->features.knee_angle_filtered > 0.0f))
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%.1f",
                                display_info->features.knee_angle_filtered);
        }
        else
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "--");
        }
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Depth");
        if (display_info->squat_last_depth_valid)
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%.0f%%",
                                display_info->squat_last_depth_percent);
        }
        else
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "--");
        }
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Lean");
        if (display_info->squat_last_lean_valid)
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%u/%.0f",
                                display_info->squat_last_lean_level,
                                display_info->squat_last_max_lean_angle);
        }
        else
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "--");
        }
    }
    else if (show_action == ACTION_JUMPING_JACK)
    {
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Count");
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%lu",
                            (unsigned long)display_info->jack_count);
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Leg");
        if (display_info->features.ankle_valid &&
            (display_info->features.ankle_ratio_filtered > 0.0f))
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%.2f",
                                display_info->features.ankle_ratio_filtered);
        }
        else
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "--");
        }
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Stretch");
        if (display_info->jack_last_stretch_valid)
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%.0f/%u",
                                display_info->jack_last_stretch_score,
                                display_info->jack_last_stretch_level);
        }
        else if (display_info->features.stretch_valid)
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%.0f%%",
                                display_info->features.stretch_score_filtered);
        }
        else
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "--");
        }
    }
    else if (show_action == ACTION_PLANK)
    {
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Time");
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%lus",
                            (unsigned long)display_info->plank_time_s);
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Body");
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%.0f/%.0f",
                            display_info->plank_body_angle,
                            display_info->plank_line_score);
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Quality");
        if (display_info->plank_quality_valid)
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "%u/%s",
                                display_info->plank_quality_level,
                                display_info->plank_hip_drop_bad ? "Sag" : "OK");
        }
        else
        {
            UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "--");
        }
    }
    else
    {
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "Ready");
        UTIL_LCDEx_PrintfAt(panel_x, LINE(line_nb++), LEFT_MODE, "--");
    }

    for (i = 0; i < display_info->nb_detect; i++)
    {
        app_display_detection(&display_info->detects[i]);
    }

    app_lcd_draw_area_commit();
}
