#include "fibocom.h"

#include "main.h"
#include "tx_api.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

#define FIBOCOM_WAIT_STEP_MS              20U
#define FIBOCOM_CONNECT_RETRY_MAX         5U
#define FIBOCOM_COMMAND_GAP_TICKS         100U

extern UART_HandleTypeDef huart1;

static uint8_t g_fibocom_connected = 0;
static uint8_t g_fibocom_startup_done = 0;

static void fibocom_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

static void fibocom_clear_rx(void)
{
#if UART_EN_RX
    memset(g_uart_rx_buf, 0, sizeof(g_uart_rx_buf));
    g_uart_rx_sta = 0;
#endif
}

static void fibocom_send_raw(const char *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)strlen(cmd), 0xFFFF);
}

static uint8_t fibocom_wait_for(const char *target, uint32_t timeout_ms)
{
#if UART_EN_RX
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (g_uart_rx_sta & 0x8000U)
        {
            char line[UART_REC_LEN + 1];
            uint16_t len = g_uart_rx_sta & 0x3FFFU;

            if (len > UART_REC_LEN)
            {
                len = UART_REC_LEN;
            }

            memcpy(line, g_uart_rx_buf, len);
            line[len] = '\0';
            fibocom_clear_rx();

            if ((target == NULL) || (strstr(line, target) != NULL))
            {
                return 1;
            }
        }

        fibocom_delay_ms(FIBOCOM_WAIT_STEP_MS);
    }
#else
    (void)target;
    (void)timeout_ms;
#endif

    return 0;
}

static uint8_t fibocom_send_wait(const char *cmd, const char *target, uint32_t timeout_ms)
{
    fibocom_clear_rx();
    fibocom_send_raw(cmd);
    return fibocom_wait_for(target, timeout_ms);
}

static uint8_t fibocom_prepare_ip(void)
{
    uint8_t retry;

    if (fibocom_send_wait("AT+MIPCALL?\r\n", "+MIPCALL: 1", 1200U))
    {
        return 1;
    }

    for (retry = 0; retry < FIBOCOM_CONNECT_RETRY_MAX; retry++)
    {
        if (fibocom_send_wait("AT+MIPCALL=1\r\n", "+MIPCALL: 1", 2000U))
        {
            return 1;
        }
    }

    return 0;
}

static uint8_t fibocom_connect_mqtt(void)
{
    uint8_t retry;

    for (retry = 0; retry < FIBOCOM_CONNECT_RETRY_MAX; retry++)
    {
        fibocom_send_raw("AT+HMDIS\r\n");
        fibocom_delay_ms(100U);

        if (fibocom_send_wait("AT+HMCON=0,60,"
                              "\"91d39d416d.st1.iotda-device.cn-north-4.myhuaweicloud.com\","
                              "\"1883\","
                              "\"6a4a24327f2e6c302f8224a8_stmn6_test\","
                              "\"11111111\","
                              "0\r\n",
                              "+HMCON OK",
                              2500U))
        {
            return 1;
        }
    }

    return 0;
}

static uint16_t fibocom_payload_len(const char *property, uint16_t num)
{
    char json[128];

    snprintf(json, sizeof(json),
             "{\"services\":[{\"service_id\":\"monitor\","
             "\"properties\":{\"%s\":%u}}]}",
             property,
             num);

    return (uint16_t)strlen(json);
}

static const char *fibocom_property_name(uint16_t type)
{
    /* Match the hub() type mapping in ght.c reference code. */
    switch (type)
    {
        case 1U:
            return "squat";
        case 2U:
            return "jumpe";
        case 3U:
            return "sqdee";
        case 4U:
            return "sqbac";
        case 5U:
            return "jopen";
        case 6U:
            return "pbtim";
        case 7U:
            return "pbbod";
        default:
            return "jopen";
    }
}

void hub(uint16_t type, uint16_t num)
{
    char cmd[220];
    const char *property = fibocom_property_name(type);
    uint16_t payload_len = fibocom_payload_len(property, num);

    snprintf(cmd,
             sizeof(cmd),
             "AT+HMPUB=1,"
             "\"$oc/devices/6a4a24327f2e6c302f8224a8_stmn6_test/sys/properties/report\","
             "%u,"
             "\"{\\\"services\\\":[{\\\"service_id\\\":\\\"monitor\\\","
             "\\\"properties\\\":{\\\"%s\\\":%u}}]}\"\r\n",
             payload_len,
             property,
             num);

    fibocom_send_wait(cmd, "+HMPUB OK", 1500U);
}

static void fibocom_first_report(void)
{
    uint16_t type;

    for (type = 1U; type <= 7U; type++)
    {
        hub(type, 0U);
        tx_thread_sleep(FIBOCOM_COMMAND_GAP_TICKS);
    }
}

static const uint8_t g_voice_startup[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xCF, 0xB5, 0xCD, 0xB3, 0xB3, 0xF5, 0xCA, 0xBC, 0xBB, 0xAF, 0xCD, 0xEA, 0xB3, 0xC9, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_all_done[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xC8, 0xAB, 0xB2, 0xBF, 0xC8, 0xCE, 0xCE, 0xF1, 0xD2, 0xD1, 0xCD, 0xEA, 0xB3, 0xC9, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_squat_target[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xC9, 0xEE, 0xB6, 0xD7, 0xCA, 0xFD, 0xC1, 0xBF, 0xD2, 0xD1, 0xB4, 0xEF, 0xB1, 0xEA, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_squat_depth[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xC9, 0xEE, 0xB6, 0xD7, 0xC9, 0xEE, 0xB6, 0xC8, 0xB2, 0xBB, 0xD7, 0xE3, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_lean_low[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xC9, 0xED, 0xCC, 0xE5, 0xC7, 0xB0, 0xC7, 0xE3, 0xB2, 0xBB, 0xD7, 0xE3, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_lean_high[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xC9, 0xED, 0xCC, 0xE5, 0xC7, 0xB0, 0xC7, 0xE3, 0xB9, 0xFD, 0xB4, 0xF3, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_jack_target[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xBF, 0xAA, 0xBA, 0xCF, 0xCC, 0xF8, 0xCA, 0xFD, 0xC1, 0xBF, 0xD2, 0xD1, 0xB4, 0xEF, 0xB1, 0xEA, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_jack_stretch[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xBF, 0xAA, 0xBA, 0xCF, 0xCC, 0xF8, 0xCA, 0xE6, 0xD5, 0xB9, 0xB6, 0xC8, 0xB2, 0xBB, 0xD7, 0xE3, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_plank_encourage[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xBC, 0xE1, 0xB3, 0xD6, 0xD7, 0xA1, 0xBC, 0xCC, 0xD0, 0xF8, 0xB1, 0xA3, 0xB3, 0xD6, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_plank_target[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xC6, 0xBD, 0xB0, 0xE5, 0xD6, 0xA7, 0xB3, 0xC5, 0xD2, 0xD1, 0xB4, 0xEF, 0xB1, 0xEA, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_plank_bad[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xCD, 0xCE, 0xB2, 0xBF, 0xCF, 0xC2, 0xCB, 0xFA, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};
static const uint8_t g_voice_fibocom[] = {0x41, 0x54, 0x2B, 0x47, 0x54, 0x54, 0x53, 0x3D, 0x31, 0x2C, 0x22, 0xB9, 0xE3, 0xBA, 0xCD, 0xCD, 0xA8, 0x22, 0x2C, 0x31, 0x0D, 0x0A, 0x00};

static void fibocom_voice_send_cmd(const uint8_t *cmd)
{
    fibocom_send_wait((const char *)cmd, "OK", 1500U);
    tx_thread_sleep(FIBOCOM_COMMAND_GAP_TICKS);
}

static void fibocom_voice_test(void)
{
    fibocom_voice_send_cmd(g_voice_startup);
}

void fibocom_voice_prompt(const char *text)
{
    if ((g_fibocom_connected == 0) || (text == NULL))
    {
        return;
    }

    if (strcmp(text, "VOICE_ALL_DONE") == 0)
    {
        fibocom_voice_send_cmd(g_voice_all_done);
    }
    else if (strcmp(text, "VOICE_SQUAT_TARGET") == 0)
    {
        fibocom_voice_send_cmd(g_voice_squat_target);
    }
    else if (strcmp(text, "VOICE_SQUAT_DEPTH") == 0)
    {
        fibocom_voice_send_cmd(g_voice_squat_depth);
    }
    else if (strcmp(text, "VOICE_LEAN_LOW") == 0)
    {
        fibocom_voice_send_cmd(g_voice_lean_low);
    }
    else if (strcmp(text, "VOICE_LEAN_HIGH") == 0)
    {
        fibocom_voice_send_cmd(g_voice_lean_high);
    }
    else if (strcmp(text, "VOICE_JACK_TARGET") == 0)
    {
        fibocom_voice_send_cmd(g_voice_jack_target);
    }
    else if (strcmp(text, "VOICE_JACK_STRETCH") == 0)
    {
        fibocom_voice_send_cmd(g_voice_jack_stretch);
    }
    else if (strcmp(text, "VOICE_PLANK_KEEP") == 0)
    {
        fibocom_voice_send_cmd(g_voice_plank_encourage);
    }
    else if (strcmp(text, "VOICE_PLANK_TARGET") == 0)
    {
        fibocom_voice_send_cmd(g_voice_plank_target);
    }
    else if (strcmp(text, "VOICE_PLANK_BAD") == 0)
    {
        fibocom_voice_send_cmd(g_voice_plank_bad);
    }
    else if (strcmp(text, "VOICE_FIBOCOM") == 0)
    {
        fibocom_voice_send_cmd(g_voice_fibocom);
    }
}
void fibocom_startup_test(void)
{
    if (g_fibocom_startup_done)
    {
        return;
    }

    g_fibocom_startup_done = 1;
    fibocom_clear_rx();

    if (fibocom_prepare_ip() == 0)
    {
        return;
    }

    if (fibocom_connect_mqtt() == 0)
    {
        return;
    }

    g_fibocom_connected = 1;
    fibocom_first_report();
    fibocom_voice_test();
}

void fibocom_report_property(uint16_t type, uint16_t num)
{
    if (g_fibocom_connected == 0)
    {
        return;
    }

    hub(type, num);
    tx_thread_sleep(FIBOCOM_COMMAND_GAP_TICKS);
}

void fibocom_report_action_done(uint16_t type, uint16_t num)
{
    fibocom_report_property(type, num);
}
