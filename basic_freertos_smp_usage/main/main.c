#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include "esp_hidd_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"
#include <string.h>
#include <inttypes.h>

#include "freertos/semphr.h"

// BLUETOOTH AND HID CONFIGURATION!

/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#define REPORT_PROTOCOL_MOUSE_REPORT_SIZE      (4)
#define REPORT_BUFFER_SIZE                     REPORT_PROTOCOL_MOUSE_REPORT_SIZE

static const char local_device_name[] = "controller_test1";

typedef struct {
    esp_hidd_app_param_t app_param;
    esp_hidd_qos_param_t both_qos;
    uint8_t protocol_mode;
    SemaphoreHandle_t mouse_mutex;
    TaskHandle_t mouse_task_hdl;
    uint8_t buffer[REPORT_BUFFER_SIZE];
    int8_t x_dir;
} local_param_t;

static local_param_t s_local_param = {0};

// HID report descriptor for a generic mouse. The contents of the report are:
// 3 buttons, moving information for X and Y cursors, information for a wheel.
uint8_t hid_mouse_descriptor[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)

    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)

    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)

    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)

    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

const int hid_mouse_descriptor_len = sizeof(hid_mouse_descriptor);

/**
 * @brief Integrity check of the report ID and report type for GET_REPORT request from HID host.
 *        Boot Protocol Mode requires report ID. For Report Protocol Mode, when the report descriptor
 *        does not declare report ID Global ITEMS, the report ID does not exist in the GET_REPORT request,
 *        and a value of 0 for report_id will occur in ESP_HIDD_GET_REPORT_EVT callback parameter.
 */
bool check_report_id_type(uint8_t report_id, uint8_t report_type)
{
    bool ret = false;
    xSemaphoreTake(s_local_param.mouse_mutex, portMAX_DELAY);
    do {
        if (report_type != ESP_HIDD_REPORT_TYPE_INPUT) {
            break;
        }
        if (s_local_param.protocol_mode == ESP_HIDD_BOOT_MODE) {
            if (report_id == ESP_HIDD_BOOT_REPORT_ID_MOUSE) {
                ret = true;
                break;
            }
        } else {
            if (report_id == 0) {
                ret = true;
                break;
            }
        }
    } while (0);

    if (!ret) {
        if (s_local_param.protocol_mode == ESP_HIDD_BOOT_MODE) {
            esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID);
        } else {
            esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID);
        }
    }
    xSemaphoreGive(s_local_param.mouse_mutex);
    return ret;
}

// send the buttons, change in x, and change in y
void send_mouse_report(uint8_t buttons, char dx, char dy, char wheel)
{
    uint8_t report_id;
    uint16_t report_size;
    xSemaphoreTake(s_local_param.mouse_mutex, portMAX_DELAY);
    if (s_local_param.protocol_mode == ESP_HIDD_REPORT_MODE) {
        report_id = 0;
        report_size = REPORT_PROTOCOL_MOUSE_REPORT_SIZE;
        s_local_param.buffer[0] = buttons;
        s_local_param.buffer[1] = dx;
        s_local_param.buffer[2] = dy;
        s_local_param.buffer[3] = wheel;
    } else {
        // Boot Mode
        report_id = ESP_HIDD_BOOT_REPORT_ID_MOUSE;
        report_size = ESP_HIDD_BOOT_REPORT_SIZE_MOUSE - 1;
        s_local_param.buffer[0] = buttons;
        s_local_param.buffer[1] = dx;
        s_local_param.buffer[2] = dy;
    }
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, report_id, report_size, s_local_param.buffer);
    xSemaphoreGive(s_local_param.mouse_mutex);
}

// move the mouse left and right
void mouse_move_task(void *pvParameters)
{
    const char *TAG = "mouse_move_task";

    ESP_LOGI(TAG, "starting");
    for (;;) {
        s_local_param.x_dir = 1;
        int8_t step = 10;
        for (int i = 0; i < 2; i++) {
            xSemaphoreTake(s_local_param.mouse_mutex, portMAX_DELAY);
            s_local_param.x_dir *= -1;
            xSemaphoreGive(s_local_param.mouse_mutex);
            for (int j = 0; j < 100; j++) {
                send_mouse_report(0, s_local_param.x_dir * step, 0, 0);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "esp_bt_gap_cb";
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            ESP_LOG_BUFFER_HEX(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %06"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%06"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;
    default:
        ESP_LOGI(TAG, "event: %d", event);
        break;
    }
    return;
}

void bt_app_task_start_up(void)
{
    s_local_param.mouse_mutex = xSemaphoreCreateMutex();
    memset(s_local_param.buffer, 0, REPORT_BUFFER_SIZE);
    xTaskCreate(mouse_move_task, "mouse_move_task", 4 * 1024, NULL, configMAX_PRIORITIES - 3, &s_local_param.mouse_task_hdl);
    return;
}

void bt_app_task_shut_down(void)
{
    if (s_local_param.mouse_task_hdl) {
        vTaskDelete(s_local_param.mouse_task_hdl);
        s_local_param.mouse_task_hdl = NULL;
    }

    if (s_local_param.mouse_mutex) {
        vSemaphoreDelete(s_local_param.mouse_mutex);
        s_local_param.mouse_mutex = NULL;
    }
    return;
}

void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    static const char *TAG = "esp_bt_hidd_cb";
    switch (event) {
    case ESP_HIDD_INIT_EVT:
        if (param->init.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters");
            esp_bt_hid_device_register_app(&s_local_param.app_param, &s_local_param.both_qos, &s_local_param.both_qos);
        } else {
            ESP_LOGE(TAG, "init hidd failed!");
        }
        break;
    case ESP_HIDD_DEINIT_EVT:
        break;
    case ESP_HIDD_REGISTER_APP_EVT:
        if (param->register_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters success!");
            ESP_LOGI(TAG, "setting to connectable, discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            if (param->register_app.in_use) {
                ESP_LOGI(TAG, "start virtual cable plug!");
                esp_bt_hid_device_connect(param->register_app.bd_addr);
            }
        } else {
            ESP_LOGE(TAG, "setting hid parameters failed!");
        }
        break;
    case ESP_HIDD_UNREGISTER_APP_EVT:
        if (param->unregister_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "unregister app success!");
        } else {
            ESP_LOGE(TAG, "unregister app failed!");
        }
        break;
    case ESP_HIDD_OPEN_EVT:
        if (param->open.status == ESP_HIDD_SUCCESS) {
            if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
                ESP_LOGI(TAG, "connecting...");
            } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
                ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                         param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                         param->open.bd_addr[5]);
                bt_app_task_start_up();
                ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "open failed!");
        }
        break;
    case ESP_HIDD_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_CLOSE_EVT");
        if (param->close.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                ESP_LOGI(TAG, "disconnecting...");
            } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    case ESP_HIDD_SEND_REPORT_EVT:
        if (param->send_report.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d", param->send_report.report_id,
                     param->send_report.report_type);
        } else {
            ESP_LOGE(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d, status:%d, reason:%d",
                     param->send_report.report_id, param->send_report.report_type, param->send_report.status,
                     param->send_report.reason);
        }
        break;
    case ESP_HIDD_REPORT_ERR_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
        break;
    case ESP_HIDD_GET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d", param->get_report.report_id,
                 param->get_report.report_type, param->get_report.buffer_size);
        if (check_report_id_type(param->get_report.report_id, param->get_report.report_type)) {
            uint8_t report_id;
            uint16_t report_len;
            if (s_local_param.protocol_mode == ESP_HIDD_REPORT_MODE) {
                report_id = 0;
                report_len = REPORT_PROTOCOL_MOUSE_REPORT_SIZE;
            } else {
                // Boot Mode
                report_id = ESP_HIDD_BOOT_REPORT_ID_MOUSE;
                report_len = ESP_HIDD_BOOT_REPORT_SIZE_MOUSE - 1;
            }
            xSemaphoreTake(s_local_param.mouse_mutex, portMAX_DELAY);
            esp_bt_hid_device_send_report(param->get_report.report_type, report_id, report_len, s_local_param.buffer);
            xSemaphoreGive(s_local_param.mouse_mutex);
        } else {
            ESP_LOGE(TAG, "check_report_id failed!");
        }
        break;
    case ESP_HIDD_SET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT");
        break;
    case ESP_HIDD_SET_PROTOCOL_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT");
        if (param->set_protocol.protocol_mode == ESP_HIDD_BOOT_MODE) {
            ESP_LOGI(TAG, "  - boot protocol");
            xSemaphoreTake(s_local_param.mouse_mutex, portMAX_DELAY);
            s_local_param.x_dir = -1;
            xSemaphoreGive(s_local_param.mouse_mutex);
        } else if (param->set_protocol.protocol_mode == ESP_HIDD_REPORT_MODE) {
            ESP_LOGI(TAG, "  - report protocol");
        }
        xSemaphoreTake(s_local_param.mouse_mutex, portMAX_DELAY);
        s_local_param.protocol_mode = param->set_protocol.protocol_mode;
        xSemaphoreGive(s_local_param.mouse_mutex);
        break;
    case ESP_HIDD_INTR_DATA_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
        break;
    case ESP_HIDD_VC_UNPLUG_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
        if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    default:
        break;
    }
}

// end of BLUETOOTH AND HID CONFIGURATION!


#define NUM_BOTOES 5

// ---------------- Botões digitais ----------------
static const gpio_num_t botoes[NUM_BOTOES] = {
    GPIO_NUM_22, // botao1
    GPIO_NUM_23, // botao2
    GPIO_NUM_25, // botao3
    GPIO_NUM_26, // botao4
    GPIO_NUM_27, // botao5
};

static const char *nomes_botoes[NUM_BOTOES] = {
    "botao1",
    "botao2",
    "botao3",
    "botao4",
    "botao5",
};

// ---------------- Joystick analógico ----------------
// ATENÇÃO: GPIO16 e GPIO17 NÃO têm ADC na ESP32 clássica.
// Por isso X e Y foram movidos para pinos ADC1 (só entrada):
#define JOY_X_ADC_CHANNEL ADC_CHANNEL_6 // GPIO34
#define JOY_Y_ADC_CHANNEL ADC_CHANNEL_7 // GPIO35
#define JOY_MS_GPIO       GPIO_NUM_17   // botao/switch do joystick (digital)

static const char *TAG = "TESTE_CONTROLE";

static adc_oneshot_unit_handle_t adc1_handle;

static void configurar_botoes(void)
{
    uint64_t mascara_pinos = 0;
    for (int i = 0; i < NUM_BOTOES; i++) {
        mascara_pinos |= (1ULL << botoes[i]);
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = mascara_pinos,
        .mode = GPIO_MODE_INPUT,
        
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

static void configurar_joystick_switch(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOY_MS_GPIO),
        .mode = GPIO_MODE_INPUT,
        
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

static void configurar_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12, 
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_X_ADC_CHANNEL, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_Y_ADC_CHANNEL, &chan_config));
}

// ---------------- Configurações da Pressão Simulada ----------------
#define BOTAO_PRESSIONADO_NIVEL   0      // 1 se ativo-alto (pull-down físico), 0 se ativo-baixo (pull-up físico)
#define PRESSAO_INICIAL           10.0f  // Pressão inicial ao apertar o botão
#define PRESSAO_MAXIMA            100.0f // Pressão máxima simulada
#define PRESSAO_INCREMENTO_SEG    15.0f  // Quantidade de pressão que aumenta por segundo pressionado
#define PRINT_INTERVAL_MS         200    // Intervalo de print no serial enquanto segura (em milissegundos)

typedef struct{
    int *niveis_anteriores;
    float *pressao_atual;
    int *contadores_print;
    int delay;

} updater_pointer_pack;

static void task_atualizador_botao(void *pack){
    
    updater_pointer_pack *ourpack = (updater_pointer_pack *) pack;

    int   *niveis_anteriores = ourpack->niveis_anteriores;
    float *pressao_atual     = ourpack->pressao_atual    ;
    int   *contadores_print  = ourpack->contadores_print ;

    while(1){
        for (int i = 0; i < NUM_BOTOES; i++) {
            int nivel_atual = gpio_get_level(botoes[i]);
            bool pressionado = (nivel_atual == BOTAO_PRESSIONADO_NIVEL);
            // Detecta transição de estado do botão
            if (nivel_atual != niveis_anteriores[i]) {
                if (pressionado) {
                    // Botão acabou de ser pressionado
                    pressao_atual[i] = PRESSAO_INICIAL;
                    contadores_print[i] = 0;
                    ESP_LOGI(TAG, "%s (GPIO%d) -> PRESSIONADO | Pressão Inicial: %.2f",
                                nomes_botoes[i], botoes[i], pressao_atual[i]);
                } else {
                    // Botão acabou de ser solto
                    ESP_LOGI(TAG, "%s (GPIO%d) -> SOLTO | Pressão Final: %.2f",
                                nomes_botoes[i], botoes[i], pressao_atual[i]);
                    pressao_atual[i] = 0.0f;
                    contadores_print[i] = 0;
                }
                niveis_anteriores[i] = nivel_atual;
            } else if (pressionado) {
                // Botão continua pressionado (apertar e segurar)
                // Aumenta a pressão proporcionalmente ao tempo decorrido (20ms por ciclo)
                pressao_atual[i] += PRESSAO_INCREMENTO_SEG * 0.02f;
                if (pressao_atual[i] > PRESSAO_MAXIMA) {
                    pressao_atual[i] = PRESSAO_MAXIMA;
                }
                // Controla a frequência de print no serial
                contadores_print[i]++;
                if (contadores_print[i] >= (PRINT_INTERVAL_MS / 20)) {
                    ESP_LOGI(TAG, "%s (GPIO%d) -> SEGURADO | Pressão: %.2f",
                                nomes_botoes[i], botoes[i], pressao_atual[i]);
                    contadores_print[i] = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(ourpack->delay));
    }

    vTaskDelete(NULL);
}

typedef struct{
    int *val_x;
    int *val_y;
    int delay;
} joy_pointer_pack;

static void task_atualizador_joystick(void *pack){
    joy_pointer_pack* ourpack = (joy_pointer_pack*) pack;
    while(1){
        adc_oneshot_read(adc1_handle, JOY_X_ADC_CHANNEL, ourpack->val_x);
        adc_oneshot_read(adc1_handle, JOY_Y_ADC_CHANNEL, ourpack->val_y);
        vTaskDelay(pdMS_TO_TICKS(ourpack->delay));
    }  
    vTaskDelete(NULL);

};


void app_main(void)
{

    //local device usage so compiler stops -Walling bullshit

    char noWall[strlen(local_device_name)];
    strcpy(noWall, local_device_name);

    configurar_botoes();
    configurar_joystick_switch();
    configurar_adc();

    // Estados e variáveis para simulação de pressão nos botões
    int niveis_anteriores  [NUM_BOTOES];
    float pressao_atual    [NUM_BOTOES];
    int contadores_print   [NUM_BOTOES];
    
    for (int i = 0; i < NUM_BOTOES; i++) {
        niveis_anteriores[i] = gpio_get_level(botoes[i]);
        pressao_atual[i] = 0.0f;
        contadores_print[i] = 0;
    }

    updater_pointer_pack pack;
    pack.niveis_anteriores = niveis_anteriores;
    pack.pressao_atual     = pressao_atual;
    pack.contadores_print  = contadores_print;
    pack.delay = 10;

    ESP_LOGI(TAG, "Iniciando teste: %d botoes + joystick (X/Y/MS)", NUM_BOTOES);
    ESP_LOGI(TAG, "Modo de simulação de pressão ativado (Botão pressionado nível: %d)", BOTAO_PRESSIONADO_NIVEL);

    int ms_anterior = gpio_get_level(JOY_MS_GPIO);
    int contador = 0, valor_x = 0, valor_y = 0; // joystick vars 

    joy_pointer_pack pack2;
    pack2.delay = 10;
    pack2.val_x = &valor_x;
    pack2.val_y = &valor_y;

    xTaskCreate(task_atualizador_botao   , "buttonUpdater", 4096, (void *) &pack  , 3, NULL);  // update button values
    xTaskCreate(task_atualizador_joystick, "joyUpdater"   , 4096, (void *) &pack2 , 3, NULL);  // update joystick values
    
    //then we leave app main to do the heavy lifting of sending ble hid packets inside this while!
    while (1) {

        // ---- Switch do joystick ---- never once seen this activate. Prolly dead code!
        int ms_atual = gpio_get_level(JOY_MS_GPIO);
        if (ms_atual != ms_anterior) {
            ESP_LOGI(TAG, "joystick_MS (GPIO%d) -> %s",
                     JOY_MS_GPIO, ms_atual ? "SOLTO (1)" : "PRESSIONADO (0)");
            ms_anterior = ms_atual;
        }

        // ---- Eixos analógicos (loga a cada ~500ms pra não spammar) ----
        if (contador % 25 == 0) {
            ESP_LOGI(TAG, "Joystick X=%d Y=%d", valor_x, valor_y);
            contador -= contador;
        }

        contador++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}