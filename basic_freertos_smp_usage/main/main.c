#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

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

void app_main(void)
{
    configurar_botoes();
    configurar_joystick_switch();
    configurar_adc();

    ESP_LOGI(TAG, "Iniciando teste: %d botoes + joystick (X/Y/MS)", NUM_BOTOES);
    ESP_LOGI(TAG, "Modo de simulação de pressão ativado (Botão pressionado nível: %d)", BOTAO_PRESSIONADO_NIVEL);

    // Estados e variáveis para simulação de pressão nos botões
    int niveis_anteriores[NUM_BOTOES];
    float pressao_atual[NUM_BOTOES];
    int contadores_print[NUM_BOTOES];

    for (int i = 0; i < NUM_BOTOES; i++) {
        niveis_anteriores[i] = gpio_get_level(botoes[i]);
        pressao_atual[i] = 0.0f;
        contadores_print[i] = 0;
    }
    int ms_anterior = gpio_get_level(JOY_MS_GPIO);

    int contador = 0;

    while (1) {
        // ---- Botões digitais com simulação de pressão ----
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

        // ---- Switch do joystick ----
        int ms_atual = gpio_get_level(JOY_MS_GPIO);
        if (ms_atual != ms_anterior) {
            ESP_LOGI(TAG, "joystick_MS (GPIO%d) -> %s",
                     JOY_MS_GPIO, ms_atual ? "SOLTO (1)" : "PRESSIONADO (0)");
            ms_anterior = ms_atual;
        }

        // ---- Eixos analógicos (loga a cada ~500ms pra não spammar) ----
        if (contador % 25 == 0) {
            int valor_x = 0, valor_y = 0;
            adc_oneshot_read(adc1_handle, JOY_X_ADC_CHANNEL, &valor_x);
            adc_oneshot_read(adc1_handle, JOY_Y_ADC_CHANNEL, &valor_y);
            ESP_LOGI(TAG, "Joystick X=%d Y=%d", valor_x, valor_y);
        }

        contador++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}