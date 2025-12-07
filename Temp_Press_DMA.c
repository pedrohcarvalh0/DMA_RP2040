#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "font.h"
#include <math.h>
#include <string.h>

#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa
// Display na I2C
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C

// Configuração de buffers DMA
#define BUFFER_SIZE 32              // Tamanho do buffer circular para leituras
#define DMA_CHANNEL_TEMP 0          // Canal DMA para temperatura
#define DMA_CHANNEL_UMID 1          // Canal DMA para umidade



// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
   // Fim do trecho para modo BOOTSEL com botão B

   
    stdio_init_all();

    // I2C do Display funcionando em 400Khz.
    i2c_init(I2C_PORT_DISP, 400 * 1000);

    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA_DISP);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL_DISP);                                        // Pull up the clock line
    ssd1306_t ssd;                                                     // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP); // Inicializa o display
    ssd1306_config(&ssd);                                              // Configura o display
    ssd1306_send_data(&ssd);                                           // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Inicializa o I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o BMP280
    bmp280_init(I2C_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C_PORT, &params);

    // Inicializa o AHT20
    aht20_reset(I2C_PORT);
    aht20_init(I2C_PORT);

    // Estrutura para armazenar os dados do sensor
    AHT20_Data data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    // Buffers DMA para armazenar múltiplas leituras (buffer circular)
    float temp_buffer[BUFFER_SIZE];      // Buffer de temperatura
    float umid_buffer[BUFFER_SIZE];      // Buffer de umidade
    uint8_t buffer_index = 0;            // Índice atual do buffer circular
    uint8_t buffer_count = 0;            // Contador de leituras válidas
    
    // Buffers de processamento DMA
    float temp_processed[BUFFER_SIZE];   // Buffer processado de temperatura
    float umid_processed[BUFFER_SIZE];   // Buffer processado de umidade
    
    // Valores médios para exibição
    float temp_media = 0.0f;
    float umid_media = 0.0f;
    
    // Inicializa buffers com zero
    memset(temp_buffer, 0, sizeof(temp_buffer));
    memset(umid_buffer, 0, sizeof(umid_buffer));
    memset(temp_processed, 0, sizeof(temp_processed));
    memset(umid_processed, 0, sizeof(umid_processed));

    // Configuração dos canais DMA
    int dma_chan_temp = dma_claim_unused_channel(true);
    int dma_chan_umid = dma_claim_unused_channel(true);

    // Configuração do canal DMA para temperatura
    dma_channel_config c_temp = dma_channel_get_default_config(dma_chan_temp);
    channel_config_set_transfer_data_size(&c_temp, DMA_SIZE_32);  // float = 32 bits
    channel_config_set_read_increment(&c_temp, true);
    channel_config_set_write_increment(&c_temp, true);
    channel_config_set_dreq(&c_temp, DREQ_FORCE);  // Sempre ativo

    // Configuração do canal DMA para umidade
    dma_channel_config c_umid = dma_channel_get_default_config(dma_chan_umid);
    channel_config_set_transfer_data_size(&c_umid, DMA_SIZE_32);  // float = 32 bits
    channel_config_set_read_increment(&c_umid, true);
    channel_config_set_write_increment(&c_umid, true);
    channel_config_set_dreq(&c_umid, DREQ_FORCE);  // Sempre ativo

    // Configura os canais DMA (ainda não iniciados)
    dma_channel_configure(dma_chan_temp, &c_temp,
        temp_processed,    // Destino
        temp_buffer,       // Origem
        BUFFER_SIZE,       // Número de transferências
        false              // Não inicia ainda
    );

    dma_channel_configure(dma_chan_umid, &c_umid,
        umid_processed,    // Destino
        umid_buffer,       // Origem
        BUFFER_SIZE,       // Número de transferências
        false              // Não inicia ainda
    );

    char str_tmp[10];  // Buffer para temperatura
    char str_umi[10];  // Buffer para umidade
    while (1)
    {
        // Leitura do BMP280 (usado apenas para logs, não exibido no display)
        bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Leitura do AHT20
        if (aht20_read(I2C_PORT, &data))
        {
            // Armazena leituras nos buffers DMA (buffer circular)
            temp_buffer[buffer_index] = data.temperature;
            umid_buffer[buffer_index] = data.humidity;
            
            // Incrementa o índice do buffer circular
            buffer_index = (buffer_index + 1) % BUFFER_SIZE;
            
            // Incrementa contador até encher o buffer
            if (buffer_count < BUFFER_SIZE) {
                buffer_count++;
            }
            
            // Usa DMA para transferir dados do buffer circular para buffer de processamento
            // Isso demonstra o uso de DMA para transferência eficiente de dados em alta velocidade
            // minimizando a carga da CPU durante a cópia de memória
            dma_channel_set_read_addr(dma_chan_temp, temp_buffer, false);
            dma_channel_set_write_addr(dma_chan_temp, temp_processed, false);
            dma_channel_set_trans_count(dma_chan_temp, BUFFER_SIZE, true);
            
            dma_channel_set_read_addr(dma_chan_umid, umid_buffer, false);
            dma_channel_set_write_addr(dma_chan_umid, umid_processed, false);
            dma_channel_set_trans_count(dma_chan_umid, BUFFER_SIZE, true);
            
            // Aguarda a conclusão da transferência DMA (CPU livre durante a transferência)
            dma_channel_wait_for_finish_blocking(dma_chan_temp);
            dma_channel_wait_for_finish_blocking(dma_chan_umid);
            
            // Calcula média dos valores válidos (processamento após DMA)
            temp_media = 0.0f;
            umid_media = 0.0f;
            for (int i = 0; i < buffer_count; i++) {
                temp_media += temp_processed[i];
                umid_media += umid_processed[i];
            }
            temp_media /= buffer_count;
            umid_media /= buffer_count;
            
            printf("Temperatura: %.2f C (DMA)\n", temp_media);
            printf("Umidade: %.2f %% (DMA)\n\n", umid_media);
        }
        else
        {
            printf("Erro na leitura do AHT20!\n\n");
        }

        // Prepara strings para exibição
        sprintf(str_tmp, "Temp: %.1fC", temp_media);
        sprintf(str_umi, "Umid: %.1f%%", umid_media);
    
        // Atualiza o display com informações simplificadas
        ssd1306_fill(&ssd, false);                        // Limpa o display
        ssd1306_draw_string(&ssd, "DMA", 50, 6);          // Título
        ssd1306_draw_string(&ssd, str_tmp, 5, 30);        // Temp: valor (uma linha)
        ssd1306_draw_string(&ssd, str_umi, 5, 45);        // Umid: valor (outra linha)
        ssd1306_send_data(&ssd);                          // Atualiza o display

        sleep_ms(500);
    }

    return 0;
}