/*
   Este exemplo demonstra como usar o DMA para controlar um LED com PWM no Raspberry Pi Pico.
   O LED piscará suavemente, utilizando um buffer de fade que é transferido via DMA para o PWM.
   São utilizados dois canais DMA: um para controlar o fluxo de dados e outro para transferir os valores do buffer de fade.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

#define pino_LED_0 11
#define PWM_TOP 2000  // Regula o tempo de ciclo do PWM
#define NUM_VALORES 128 //Regula o brilho final do LED

// Buffer de fade
uint16_t fade[NUM_VALORES];
static uint16_t *address_pointer = fade;
static uint32_t transfer_count = NUM_VALORES;

int main() {

    stdio_init_all();

    // Configuração do PWM para o LED
    gpio_set_function(pino_LED_0, GPIO_FUNC_PWM);
    int led_pwm_slice_num = pwm_gpio_to_slice_num(pino_LED_0);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 255.f);
    pwm_config_set_wrap(&config, PWM_TOP);
    pwm_init(led_pwm_slice_num, &config, true);

    // Gera valores de fade
    for (int i = 0; i < NUM_VALORES/2; i++) {
        fade[i] = (uint16_t)((i * i * PWM_TOP) / (255 * 255));
        fade[NUM_VALORES - 1 - i] = fade[i];
    }

    // Reclamação de canais DMA
    int data_chan = dma_claim_unused_channel(true);
    int ctrl_chan = dma_claim_unused_channel(true);

    // Canal de controle

    dma_channel_config c_ctrl = dma_channel_get_default_config(ctrl_chan);
    channel_config_set_transfer_data_size(&c_ctrl, DMA_SIZE_32);
    channel_config_set_read_increment(&c_ctrl, false);
    channel_config_set_write_increment(&c_ctrl, false);
    channel_config_set_chain_to(&c_ctrl, data_chan); // Controle aciona dados
    dma_channel_configure(ctrl_chan, &c_ctrl,
        &dma_hw->ch[data_chan].al1_read_addr, // Escreve no endereço de leitura do canal de dados
        &address_pointer,  // Lê o endereço inicial do buffer fade
        1,                 // 1 transferência
        false              // Não inicia ainda
    );

    // Canal de dados
    dma_channel_config c_data = dma_channel_get_default_config(data_chan);
    channel_config_set_transfer_data_size(&c_data, DMA_SIZE_16);
    channel_config_set_read_increment(&c_data, true);
    channel_config_set_write_increment(&c_data, false);
    channel_config_set_dreq(&c_data, DREQ_PWM_WRAP0 + led_pwm_slice_num);
    channel_config_set_chain_to(&c_data, ctrl_chan); // Dados aciona controle
    channel_config_set_ring(&c_data, true, 9);       // 256 * 2 bytes = 512, log2(512) = 9
    dma_channel_configure(data_chan, &c_data,
        &pwm_hw->slice[led_pwm_slice_num].cc, // Escreve no registrador de comparação do PWM
        fade,           // Lê do buffer fade
        NUM_VALORES,    // Número de transferências
        false           // Não inicia ainda
    );

    // Inicia encadeamento DMA
    dma_channel_set_trans_count(data_chan, NUM_VALORES, false);
    dma_channel_set_read_addr(ctrl_chan, &address_pointer, true);
    dma_start_channel_mask((1u << ctrl_chan) | (1u << data_chan));

    while (true) {
        tight_loop_contents();
    }
}