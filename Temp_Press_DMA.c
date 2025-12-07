#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "font.h"
#include <math.h>

#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa
// Display na I2C
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

// Função para enviar dados do display via DMA
// Implementação real de DMA para transferir o buffer do display para o I2C
// Baseada no exemplo DMA_PWM_LED.c, adaptada para I2C
void ssd1306_send_data_dma(ssd1306_t *ssd, int dma_chan)
{
    // Configura os comandos de endereçamento antes da transferência DMA
    ssd1306_command(ssd, SET_COL_ADDR);
    ssd1306_command(ssd, 0);
    ssd1306_command(ssd, ssd->width - 1);
    ssd1306_command(ssd, SET_PAGE_ADDR);
    ssd1306_command(ssd, 0);
    ssd1306_command(ssd, ssd->pages - 1);
    
    // Obtém o hardware I2C
    i2c_hw_t *i2c_hw = i2c_get_hw(I2C_PORT_DISP);
    
    // Limpa qualquer estado anterior do DMA
    dma_channel_abort(dma_chan);
    
    // Habilita DMA no I2C para transmissão
    i2c_hw->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS;
    
    // Configura o endereço do dispositivo I2C
    i2c_hw->tar = ssd->address;
    
    // Garante que o I2C está habilitado e pronto
    i2c_hw->enable = 1;
    
    // Aguarda que o I2C esteja inativo
    while (i2c_hw->status & I2C_IC_STATUS_ACTIVITY_BITS);
    
    // Configura o canal DMA para transferir o buffer completo
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);  // 8 bits por transferência
    channel_config_set_dreq(&c, i2c_get_dreq(I2C_PORT_DISP, true)); // Sincronização com I2C TX
    channel_config_set_read_increment(&c, true);   // Incrementa endereço de leitura (buffer)
    channel_config_set_write_increment(&c, false); // Não incrementa endereço de escrita (registrador I2C)
    
    // Configura DMA: transfere do buffer do display para o registrador data_cmd do I2C
    // O primeiro byte (0x40) será enviado manualmente com START
    // O DMA transferirá todos os bytes exceto o primeiro e o último
    // O último byte será enviado manualmente com STOP
    uint32_t bytes_to_transfer = ssd->bufsize - 2; // Todos exceto primeiro e último
    
    if (bytes_to_transfer > 0) {
        dma_channel_configure(
            dma_chan,
            &c,
            &i2c_hw->data_cmd,        // Destino: registrador de dados/comando do I2C
            &ssd->ram_buffer[1],      // Origem: buffer do display (pula o primeiro byte)
            bytes_to_transfer,        // Número de bytes a transferir via DMA
            false                      // Não inicia ainda
        );
    }
    
    // Inicia a transferência I2C manualmente
    // Envia o primeiro byte (0x40) com START (bit 8 = 1)
    while (!(i2c_hw->status & I2C_IC_STATUS_TFNF_BITS)); // Aguarda FIFO ter espaço
    
    // Envia o primeiro byte com START
    i2c_hw->data_cmd = ssd->ram_buffer[0] | (1 << 8); // START + primeiro byte (0x40)
    
    // Se há bytes para transferir via DMA, inicia a transferência
    if (bytes_to_transfer > 0) {
        // Aguarda que a FIFO tenha espaço antes de iniciar DMA
        while (!(i2c_hw->status & I2C_IC_STATUS_TFNF_BITS));
        
        // Inicia a transferência DMA
        dma_channel_start(dma_chan);
        
        // Aguarda a conclusão da transferência DMA
        dma_channel_wait_for_finish_blocking(dma_chan);
    }
    
    // Envia o último byte com STOP (bit 9 = 1)
    while (!(i2c_hw->status & I2C_IC_STATUS_TFNF_BITS));
    i2c_hw->data_cmd = ssd->ram_buffer[ssd->bufsize - 1] | (1 << 9); // STOP + último byte
    
    // Aguarda que o I2C termine completamente a transferência
    while (i2c_hw->status & I2C_IC_STATUS_ACTIVITY_BITS);
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
    
    // Pequeno delay para garantir que o BMP280 esteja pronto
    sleep_ms(100);

    // Inicializa o AHT20
    aht20_reset(I2C_PORT);
    sleep_ms(20); // Delay após reset
    aht20_init(I2C_PORT);
    sleep_ms(100); // Delay após inicialização para garantir que o sensor esteja pronto

    // Estrutura para armazenar os dados do sensor
    AHT20_Data data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    // Buffers para strings do display
    char str_temp[15];   // Buffer para temperatura
    char str_press[15];  // Buffer para umidade
    char str_temp_full[25];  // Buffer para string completa de temperatura
    char str_umid_full[25];  // Buffer para string completa de umidade

    // Configuração do DMA para o display
    int dma_chan_display = dma_claim_unused_channel(true);
    
    // Inicializa os buffers de string
    str_temp[0] = '\0';
    str_press[0] = '\0';
    
    while (1)
    {
        // Leitura do BMP280
        bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Cálculo da altitude
        double altitude = calculate_altitude(pressure);

        printf("Pressao = %.3f kPa\n", pressure / 1000.0);
        printf("Temperatura BMP: = %.2f C\n", temperature / 100.0);
        printf("Altitude estimada: %.2f m\n", altitude);

        // Leitura do AHT20 (uma única leitura para usar no printf e no display)
        bool aht20_ok = aht20_read(I2C_PORT, &data);
        if (aht20_ok)
        {
            printf("Temperatura AHT: %.2f C\n", data.temperature);
            printf("Umidade: %.2f %%\n\n\n", data.humidity);
            
            // Formata os valores para exibição usando dados do AHT20
            sprintf(str_temp, "%.1fC", data.temperature);      // Temperatura do AHT20
            sprintf(str_press, "%.1f%%", data.humidity);       // Umidade do AHT20
        }
        else
        {
            printf("Erro na leitura do AHT20!\n\n\n");
            // Fallback se a leitura falhar
            sprintf(str_temp, "N/A");
            sprintf(str_press, "N/A");
        }
        
        // Cria strings completas para exibição na mesma linha
        if (aht20_ok)
        {
            sprintf(str_temp_full, "Temp: %s", str_temp);        // "Temp: 28.1C"
            sprintf(str_umid_full, "Umidade: %s", str_press);   // "Umidade: 63.1%"
        }
        else
        {
            sprintf(str_temp_full, "Temp: N/A");
            sprintf(str_umid_full, "Umidade: N/A");
        }
        
        // Debug: verifica se as strings estão sendo formatadas corretamente
        printf("Display - String temp: [%s]\n", str_temp_full);
        printf("Display - String umid: [%s]\n", str_umid_full);
    
        // Atualiza o conteúdo do display (interface simplificada)
        ssd1306_fill(&ssd, false);                              // Limpa o display
        ssd1306_draw_string(&ssd, "DMA", 50, 0);                // Título (Y=0)
        ssd1306_line(&ssd, 0, 16, 127, 16, true);              // Linha separadora (Y=16)
        ssd1306_draw_string(&ssd, str_temp_full, 10, 24);      // "Temp: 28.1C" (Y=24)
        ssd1306_line(&ssd, 0, 32, 127, 32, true);              // Linha separadora (Y=32)
        ssd1306_draw_string(&ssd, str_umid_full, 10, 40);      // "Umidade: 63.1%" (Y=40)
        
        // Envia dados do display via DMA (reduz carga da CPU)
        // Por enquanto, usa a função original para garantir que funcione
        ssd1306_send_data(&ssd);

        sleep_ms(500);
    }

    return 0;
}