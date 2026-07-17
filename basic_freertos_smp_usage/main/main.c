#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
//#include "esp_hidd_api.h"
#include "esp_gap_ble_api.h"

#include "esp_hidd_prf_api.h" //may be conflicting with esp_hidd_api, even if protected through ifndef
#include "esp_wifi.h"
#include "esp_event.h"

#include "esp_system.h"
#include "esp_bt_defs.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_device.h"
#include "hid_dev.h"

#include "esp_bt_defs.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_device.h"


// BLUETOOTH FUNCTIONS

#define HID_DEMO_TAG "CONTROLLER_TEST"

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool send_volum_up = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "Controller_test"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};


static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = ESP_BLE_GAP_CONN_ITVL_MS(7.5), //slave connection min interval
    .max_interval = ESP_BLE_GAP_CONN_ITVL_MS(20), //slave connection max interval
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = ESP_BLE_GAP_ADV_ITVL_MS(20),
    .adv_int_max        = ESP_BLE_GAP_ADV_ITVL_MS(30),
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);

            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->led_write.data, param->led_write.length);
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
                ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
            }
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            esp_bd_addr_t bd_addr;
            memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
            ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                    (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                    (bd_addr[4] << 8) + bd_addr[5]);
            ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
            ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
            if (param->ble_security.auth_cmpl.success) {
                sec_conn = true;
                ESP_LOGI(HID_DEMO_TAG, "secure connection established.");
            } else {
                ESP_LOGE(HID_DEMO_TAG, "pairing failed, reason = 0x%x",
                        param->ble_security.auth_cmpl.fail_reason);
            }
            break;
        default:
            break;
    }
}




// END OF BTFUNCTIONS




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


#define JOY_X_ADC_CHANNEL ADC_CHANNEL_6 // GPIO34
#define JOY_Y_ADC_CHANNEL ADC_CHANNEL_7 // GPIO35
#define JOY_MS_GPIO       GPIO_NUM_17   // botao/switch do joystick (digital)

static const char *TAG = "TESTE_CONTROLE";
static const char *ERRORTAG = "ERRO";

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

typedef struct{
    int *val_x;
    int *val_y;
    float *pressures;
}sender_pack;


#define JOY_ADC_MAX      4095
#define JOY_ADC_CENTER   (JOY_ADC_MAX / 2)

static int8_t mapear_eixo_para_int8(int valor_adc)
{
    int delta = valor_adc - JOY_ADC_CENTER;
    int escalado = (delta * 127) / JOY_ADC_CENTER;

    if (escalado > 127) {
        escalado = 127;
    } else if (escalado < -127) {
        escalado = -127;
    }

    return (int8_t) escalado;
}


static uint8_t mapear_botoes_para_bitmask(const float *pressoes)
{
    uint8_t mascara = 0;

    for (int i = 0; i < NUM_BOTOES; i++) {
        if (pressoes[i] > 0.0f) {
            mascara |= (1 << i);
        }
    }

    return mascara;
}

// Envia periodicamente o estado atual (botões + eixos) via HID sobre BLE.
static void task_sender(void *pack)
{
    sender_pack *ourpack = (sender_pack*) pack;

    uint8_t buffer_botoes; // uint8_t, pronto para ir no relatório
    int8_t  buffer_x;
    int8_t  buffer_y;

    while (1) {
        // leitura já feita pelas outras tasks; aqui só convertemos para uint8_t/int8_t
        buffer_botoes = mapear_botoes_para_bitmask(ourpack->pressures);
        buffer_x      = mapear_eixo_para_int8(*ourpack->val_x);
        buffer_y      = mapear_eixo_para_int8(*ourpack->val_y);

        if (sec_conn) {
            // chamada da função da API que efetivamente manda o relatório pelo GATT
            esp_hidd_send_mouse_value(hid_conn_id, buffer_botoes, buffer_x, buffer_y);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void app_main(void)
{
    // pack setup and configurationnn
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

    sender_pack pack3;
    pack3.pressures = pressao_atual;
    pack3.val_x = &valor_x;
    pack3.val_y = &valor_y;

    //bluetooth configuration and init
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);

    if(ret){
        ESP_LOGI(ERRORTAG, "erro na inicialização do controlador bluetooth");
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);

    if(ret){
        ESP_LOGI(ERRORTAG, "erro na habilitação do controlador bluetooth");
    }

    esp_bluedroid_config_t bd_cfg;
    bd_cfg.sc_en  = 0;
    bd_cfg.ssp_en = 0;
    
    ret = esp_bluedroid_init_with_cfg(&bd_cfg);
    
    if(ret){
        ESP_LOGI(ERRORTAG, "erro na inicialização do bluedroid!");
    }
    
    ret = esp_bluedroid_enable();
    
    if(ret){
        ESP_LOGI(ERRORTAG, "error ao habilitar bluedroid!");
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    // Parâmetros de segurança/pareamento BLE — sem isso o ESP_GAP_BLE_SEC_REQ_EVT
    // e o ESP_GAP_BLE_AUTH_CMPL_EVT tratados em gap_event_handler não têm com o
    // que negociar, e o pareamento fica inconsistente.
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     // bonding, sem MITM
    esp_ble_io_cap_t   iocap    = ESP_IO_CAP_NONE;      // sem display/teclado no "controle"
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));

    //end of bt part

    xTaskCreate(task_atualizador_botao   , "buttonUpdater", 4096, (void *) &pack  , 3, NULL);  // update button values
    xTaskCreate(task_atualizador_joystick, "joyUpdater"   , 4096, (void *) &pack2 , 3, NULL);  // update joystick values
    xTaskCreate(task_sender              , "hidSender"    , 4096, (void *) &pack3 , 3, NULL);  // convert & send HID reports
    
    //then we leave app main to do the heavy lifting of sending ble hid packets inside this while!
    while (1) {

        // ---- Switch do joystick ---- never once seen this activate. Prolly dead code!
        int ms_atual = gpio_get_level(JOY_MS_GPIO);
        if (ms_atual != ms_anterior) {
            ESP_LOGI(TAG, "joystick_MS (GPIO%d) -> %s",
                     JOY_MS_GPIO, ms_atual ? "SOLTO (1)" : "PRESSIONADO (0)");
            ms_anterior = ms_atual;
        }

        // ---- Eixos analógicos 
        if (contador % 25 == 0) {
            ESP_LOGI(TAG, "Joystick X=%d Y=%d", valor_x, valor_y);
            contador -= contador;
        }

        contador++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}