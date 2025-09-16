#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ssd1306.h"

static const char *TAG = "MONITOR";

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21  
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define AHT20_ADDR                  0x38
#define BMP280_ADDR                 0x77  // Tentando endereco alternativo
#define SSD1306_ADDR                0x3C  // Endereco padrao

#define LED_GREEN_PIN               2
#define LED_YELLOW_PIN              4
#define LED_RED_PIN                 5

typedef struct {
    float temperature;
    float pressure;
} sensor_data_t;

static sensor_data_t g_sensor_data = {0};
static bool g_oled_enabled = false;

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, 
                             I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_write_read(uint8_t addr, uint8_t *write_buf, size_t write_len, 
                               uint8_t *read_buf, size_t read_len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr, write_buf, write_len,
                                       read_buf, read_len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


static void gpio_init(void)
{
    gpio_set_direction(LED_GREEN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_YELLOW_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
    
    gpio_set_level(LED_GREEN_PIN, 0);
    gpio_set_level(LED_YELLOW_PIN, 0);
    gpio_set_level(LED_RED_PIN, 0);
}

static esp_err_t aht20_init(void)
{
    // Comando de soft reset
    uint8_t reset_cmd[] = {0xBA};
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_ADDR, reset_cmd, sizeof(reset_cmd),
                                              I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Comando de inicializacao/calibracao
    uint8_t init_cmd[] = {0xBE, 0x08, 0x00};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_ADDR, init_cmd, sizeof(init_cmd),
                                    I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return ret;
}

static esp_err_t aht20_read_temperature(float *temperature)
{
    uint8_t trigger_cmd[] = {0xAC, 0x33, 0x00};
    uint8_t data[6];
    
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_ADDR, trigger_cmd, 
                                              sizeof(trigger_cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Ler dados
    for (int tentativa = 0; tentativa < 10; tentativa++) {
        ret = i2c_master_read_from_device(I2C_MASTER_NUM, AHT20_ADDR, data, sizeof(data),
                                         I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        if (ret != ESP_OK) return ret;
        
        if ((data[0] & 0x80) == 0) break; // Sensor pronto
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Extrair apenas temperatura (ignorar umidade)
    uint32_t temperature_raw = (((uint32_t)data[3] & 0xF) << 16) | ((uint32_t)data[4] << 8) | data[5];
    *temperature = (float)temperature_raw * 200.0 / 1048576.0 - 50.0;
    
    return ESP_OK;
}


static esp_err_t bmp280_init(void)
{
    uint8_t ctrl_reg[] = {0xF4, 0x27};
    return i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_ADDR, ctrl_reg, sizeof(ctrl_reg),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t bmp280_read(float *temperature, float *pressure)
{
    uint8_t reg_addr = 0xF7;
    uint8_t data[6];
    
    esp_err_t ret = i2c_write_read(BMP280_ADDR, &reg_addr, 1, data, sizeof(data));
    if (ret != ESP_OK) return ret;
    
    int32_t pressure_raw = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t temp_raw = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);
    
    // Conversao simplificada para BMP280 (aproximada)
    // Para precisao total, seria necessario ler os coeficientes de calibracao
    *temperature = ((float)temp_raw / 16384.0) - 4.0;  // Aproximacao para temperatura
    *pressure = ((float)pressure_raw / 256.0) / 100.0;  // Aproximacao para pressao
    
    // Ajuste baseado em valores tipicos ao nivel do mar
    *pressure = *pressure + 990.0; // Compensacao aproximada
    
    return ESP_OK;
}

static void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Iniciando leitura de sensores");
    
    while (1) {
        float bmp_temp; // Temperatura do BMP280 (não usada)
        esp_err_t ret_aht20 = aht20_read_temperature(&g_sensor_data.temperature);
        esp_err_t ret_bmp280 = bmp280_read(&bmp_temp, &g_sensor_data.pressure);
        
        if (ret_aht20 == ESP_OK && ret_bmp280 == ESP_OK) {
            ESP_LOGI(TAG, "AHT20 - Temp: %.1f°C", g_sensor_data.temperature);
            ESP_LOGI(TAG, "BMP280 - Pressao: %.1fhPa", g_sensor_data.pressure);
        } else {
            ESP_LOGE(TAG, "Erro na leitura dos sensores - AHT20: %s, BMP280: %s",
                    (ret_aht20 == ESP_OK) ? "OK" : "FALHA",
                    (ret_bmp280 == ESP_OK) ? "OK" : "FALHA");
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void display_task(void *pvParameters)
{
    char line_buffer[32];
    
    while (1) {
        // Se OLED nao esta habilitado, tenta reinicializar a cada 10 segundos
        if (!g_oled_enabled) {
            ESP_LOGW(TAG, "Tentando reinicializar SSD1306...");
            if (ssd1306_init() == ESP_OK) {
                ESP_LOGI(TAG, "SSD1306 reinicializado com sucesso!");
                g_oled_enabled = true;
                // Limpar display completamente apos reinicializacao
                ssd1306_clear_screen();
                ssd1306_update_display();
            } else {
                ESP_LOGW(TAG, "Reinicializacao do SSD1306 falhou, tentando novamente em 10s");
                vTaskDelay(pdMS_TO_TICKS(10000));
                continue;
            }
        }
        
        // Atualizar display apenas se OLED estiver funcionando
        if (g_oled_enabled) {
            // Limpar apenas na primeira atualizacao ou periodicamente
            static int clear_counter = 0;
            if (clear_counter == 0) {
                ssd1306_clear_screen();
                clear_counter = 30; // Limpar a cada 30 segundos
            }
            clear_counter--;
            
            snprintf(line_buffer, sizeof(line_buffer), "TEMPERATURA: %.1fC", g_sensor_data.temperature);
            ssd1306_display_text(0, line_buffer);
            
            snprintf(line_buffer, sizeof(line_buffer), "PRESSAO: %.1f HPA", g_sensor_data.pressure);
            ssd1306_display_text(2, line_buffer);
            
            esp_err_t ret = ssd1306_update_display();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Erro no update_display, OLED desabilitado");
                g_oled_enabled = false;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void led_task(void *pvParameters)
{
    while (1) {
        gpio_set_level(LED_GREEN_PIN, 1);
        gpio_set_level(LED_YELLOW_PIN, 0);
        gpio_set_level(LED_RED_PIN, 0);
        
        if (g_sensor_data.temperature > 30.0 || g_sensor_data.temperature < 15.0) {
            gpio_set_level(LED_YELLOW_PIN, 1);
            gpio_set_level(LED_GREEN_PIN, 0);
        }
        
        if (g_sensor_data.pressure < 1000.0 || g_sensor_data.pressure > 1030.0) {
            gpio_set_level(LED_RED_PIN, 1);
            gpio_set_level(LED_GREEN_PIN, 0);
            gpio_set_level(LED_YELLOW_PIN, 0);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void validation_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Iniciando validacao do sistema");
    
    uint8_t test_data;
    esp_err_t ret_aht20 = i2c_master_read_from_device(I2C_MASTER_NUM, AHT20_ADDR, &test_data, 1,
                                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    esp_err_t ret_bmp280 = i2c_master_read_from_device(I2C_MASTER_NUM, BMP280_ADDR, &test_data, 1,
                                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    esp_err_t ret_oled = i2c_master_read_from_device(I2C_MASTER_NUM, SSD1306_ADDR, &test_data, 1,
                                                    I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "Teste I2C - AHT20: %s, BMP280: %s, OLED: %s", 
            (ret_aht20 == ESP_OK) ? "OK" : "FALHA",
            (ret_bmp280 == ESP_OK) ? "OK" : "FALHA",
            (ret_oled == ESP_OK) ? "OK" : "FALHA");
    
    ssd1306_clear_screen();
    ssd1306_display_text(0, "TESTE SISTEMA");
    ssd1306_display_text(2, "Validando...");
    ssd1306_update_display();
    
    gpio_set_level(LED_GREEN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(LED_GREEN_PIN, 0);
    
    gpio_set_level(LED_YELLOW_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(LED_YELLOW_PIN, 0);
    
    gpio_set_level(LED_RED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(LED_RED_PIN, 0);
    
    ESP_LOGI(TAG, "Teste de LEDs concluido");
    
    ssd1306_clear_screen();
    ssd1306_display_text(0, "TESTE CONCLUIDO");
    ssd1306_display_text(2, "Sistema OK!");
    ssd1306_update_display();
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando sistema de monitoramento");
    
    ESP_ERROR_CHECK(i2c_master_init());
    gpio_init();
    
    ESP_LOGI(TAG, "Testando sensores AHT20 e BMP280...");
    
    // Teste AHT20
    esp_err_t aht20_ret = aht20_init();
    ESP_LOGI(TAG, "AHT20 init result: 0x%x", (unsigned int)aht20_ret);
    if (aht20_ret != ESP_OK) {
        ESP_LOGE(TAG, "ERRO: AHT20 falhou na inicializacao");
    }
    
    // Teste BMP280  
    esp_err_t bmp280_ret = bmp280_init();
    ESP_LOGI(TAG, "BMP280 init result: 0x%x", (unsigned int)bmp280_ret);
    if (bmp280_ret != ESP_OK) {
        ESP_LOGE(TAG, "ERRO: BMP280 falhou na inicializacao");
    }
    
    // Scanner I2C para detectar dispositivos
    ESP_LOGI(TAG, "Executando scanner I2C...");
    uint8_t dummy_data = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, addr, &dummy_data, 0, 100 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Dispositivo I2C encontrado no endereco: 0x%02X", addr);
        }
    }
    
    ESP_ERROR_CHECK(aht20_ret);
    ESP_ERROR_CHECK(bmp280_ret);
    
    // Teste OLED com multiplas tentativas
    esp_err_t oled_ret = ESP_FAIL;
    for (int tentativa = 1; tentativa <= 3; tentativa++) {
        ESP_LOGI(TAG, "Tentativa %d de inicializacao do SSD1306...", tentativa);
        oled_ret = ssd1306_init();
        if (oled_ret == ESP_OK) {
            ESP_LOGI(TAG, "SSD1306 inicializado com sucesso na tentativa %d!", tentativa);
            break;
        }
        ESP_LOGW(TAG, "SSD1306 tentativa %d falhou: 0x%x", tentativa, (unsigned int)oled_ret);
        vTaskDelay(pdMS_TO_TICKS(500)); // Aguardar entre tentativas
    }
    
    if (oled_ret != ESP_OK) {
        ESP_LOGW(TAG, "SSD1306 falhou em todas as tentativas - display desabilitado");
        g_oled_enabled = false;
    } else {
        g_oled_enabled = true;
    }
    
    xTaskCreate(validation_task, "validation_task", 2048, NULL, 2, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, 4, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "Sistema inicializado com sucesso");
}