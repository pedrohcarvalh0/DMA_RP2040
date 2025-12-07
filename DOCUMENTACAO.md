# Documentação - Sistema de Monitoramento de Temperatura e Umidade com DMA

## Visão Geral

Este projeto implementa um sistema de monitoramento de temperatura e umidade utilizando o microcontrolador Raspberry Pi Pico (RP2040). O sistema utiliza sensores AHT20 e BMP280 conectados via I2C, exibindo os dados em um display OLED SSD1306. A principal característica deste projeto é a **implementação real de DMA (Direct Memory Access)** para otimizar a transferência de dados para o display, reduzindo significativamente a carga de processamento da CPU.

## Funcionalidades

- Leitura de temperatura e umidade do sensor AHT20
- Leitura de temperatura e pressão do sensor BMP280
- Exibição dos dados em display OLED SSD1306 (128x64)
- **Implementação real de DMA** para transferência de dados ao display
- Comunicação serial para debug e monitoramento

## Arquitetura do Sistema

### Componentes Principais

1. **Microcontrolador**: Raspberry Pi Pico (RP2040)
2. **Sensores**:
   - AHT20: Temperatura e Umidade
   - BMP280: Temperatura e Pressão
3. **Display**: SSD1306 OLED 128x64 pixels
4. **Comunicação**: I2C (dois barramentos separados)

### Configuração de Pinos

```
I2C0 (Sensores):
- SDA: GPIO 0
- SCL: GPIO 1

I2C1 (Display):
- SDA: GPIO 14
- SCL: GPIO 15

Botão BOOTSEL:
- GPIO 6
```

## Implementação de DMA

### O que é DMA?

**DMA (Direct Memory Access)** é um recurso de hardware que permite transferir dados entre periféricos e memória sem a intervenção direta da CPU. Isso resulta em:

- **Redução da carga da CPU**: A CPU pode executar outras tarefas enquanto o DMA gerencia a transferência
- **Maior eficiência**: Transferências mais rápidas e eficientes
- **Menor latência**: Menos interrupções e overhead de software

### DMA no RP2040

O RP2040 possui 12 canais DMA independentes que podem ser configurados para transferir dados entre:

- Memória e periféricos (SPI, I2C, UART, PWM, ADC, etc.)
- Memória e memória
- Periféricos e periféricos

### Implementação Real no Projeto

#### 1. Estrutura da Função DMA

```c
void ssd1306_send_data_dma(ssd1306_t *ssd, int dma_chan)
```

Esta função implementa **DMA real** para transferir o buffer do display (1024 bytes) para o I2C, baseada no exemplo `DMA_PWM_LED.c`.

#### 2. Configuração do Canal DMA

```c
// Reclamação de um canal DMA não utilizado
int dma_chan_display = dma_claim_unused_channel(true);
```

O código reserva um canal DMA exclusivo para a transferência de dados do display.

#### 3. Detalhes da Implementação

A função `ssd1306_send_data_dma()` implementa os seguintes passos:

##### Passo 1: Preparação do Hardware I2C

```c
i2c_hw_t *i2c_hw = i2c_get_hw(I2C_PORT_DISP);
i2c_hw->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS;  // Habilita DMA no I2C
i2c_hw->tar = ssd->address;                 // Configura endereço do dispositivo
```

##### Passo 2: Configuração do Canal DMA

```c
dma_channel_config c = dma_channel_get_default_config(dma_chan);
channel_config_set_transfer_data_size(&c, DMA_SIZE_8);  // 8 bits por transferência
channel_config_set_dreq(&c, i2c_get_dreq(I2C_PORT_DISP, true)); // Sincronização com I2C TX
channel_config_set_read_increment(&c, true);   // Incrementa endereço de leitura
channel_config_set_write_increment(&c, false); // Não incrementa endereço de escrita
```

##### Passo 3: Estratégia de Transferência Híbrida

A implementação usa uma estratégia híbrida para garantir controle preciso da transferência I2C:

1. **Primeiro byte (manual)**: Enviado manualmente com bit START

   ```c
   i2c_hw->data_cmd = ssd->ram_buffer[0] | (1 << 8); // START + byte 0x40
   ```

2. **Bytes intermediários (DMA)**: Transferidos automaticamente via DMA

   ```c
   dma_channel_configure(dma_chan, &c, &i2c_hw->data_cmd,
                        &ssd->ram_buffer[1], bytes_to_transfer, false);
   dma_channel_start(dma_chan);
   ```

3. **Último byte (manual)**: Enviado manualmente com bit STOP
   ```c
   i2c_hw->data_cmd = ssd->ram_buffer[last] | (1 << 9); // STOP + último byte
   ```

#### 4. Sincronização via DREQ

O DMA utiliza **DREQ (Data Request)** do I2C para sincronização:

- O DMA aguarda automaticamente que o I2C esteja pronto para receber dados
- Isso garante que a transferência seja feita na velocidade correta
- Evita overflow ou underflow da FIFO do I2C

#### 5. Fluxo Completo de Operação

```
1. CPU configura comandos de endereçamento do display
2. CPU habilita DMA no I2C e configura endereço do dispositivo
3. CPU envia primeiro byte manualmente (START + 0x40)
4. DMA inicia transferência automática dos bytes intermediários
5. DMA aguarda DREQ do I2C para cada byte (sincronização automática)
6. CPU envia último byte manualmente (STOP + último byte)
7. CPU aguarda conclusão da transferência I2C
```

### Comparação: Com vs Sem DMA

#### Sem DMA (Método Original)

```c
i2c_write_blocking(I2C_PORT_DISP, ssd->address, ssd->ram_buffer, ssd->bufsize, false);
```

- CPU bloqueada durante toda a transferência (1024 bytes)
- CPU precisa gerenciar cada byte individualmente
- Não pode executar outras tarefas durante a transferência

#### Com DMA (Implementação Atual)

```c
ssd1306_send_data_dma(&ssd, dma_chan_display);
```

- CPU livre após iniciar a transferência
- DMA gerencia automaticamente os bytes intermediários
- CPU pode processar sensores, cálculos, etc. durante a transferência
- Apenas o primeiro e último byte requerem intervenção da CPU

### Benefícios da Implementação Real

1. **Performance**: CPU livre para outras operações durante ~99% da transferência
2. **Eficiência**: Menor consumo de energia e melhor uso de recursos
3. **Escalabilidade**: Permite adicionar mais funcionalidades sem impactar a atualização do display
4. **Precisão**: Controle total sobre início (START) e fim (STOP) da transferência I2C

## Estrutura do Código

### Funções Principais

#### `main()`

Função principal que:

- Inicializa os periféricos (I2C, display, sensores)
- Configura o canal DMA
- Executa o loop principal de leitura e exibição

#### `ssd1306_send_data_dma()`

Função que implementa **DMA real** para transferir o buffer do display para o I2C.

**Características**:

- Acesso direto ao hardware I2C (`i2c_hw_t`)
- Configuração de DREQ para sincronização
- Estratégia híbrida (manual + DMA)
- Controle preciso de START e STOP

#### `calculate_altitude()`

Calcula a altitude estimada a partir da pressão atmosférica.

### Loop Principal

```c
while (1) {
    // 1. Leitura dos sensores (CPU livre)
    bmp280_read_raw(...);
    aht20_read(...);

    // 2. Processamento dos dados (CPU livre)
    // Conversão e formatação

    // 3. Atualização do display
    ssd1306_fill(...);
    ssd1306_draw_string(...);
    ssd1306_send_data_dma(...);  // DMA gerencia a transferência

    // 4. Delay
    sleep_ms(500);
}
```

## Sensores

### AHT20

- **Função**: Mede temperatura e umidade
- **Interface**: I2C
- **Dados utilizados**: Temperatura e Umidade (exibidos no display)
- **Precisão**:
  - Temperatura: ±0.3°C
  - Umidade: ±2% RH

### BMP280

- **Função**: Mede temperatura e pressão
- **Interface**: I2C
- **Dados utilizados**: Temperatura e Pressão (exibidos no serial)
- **Precisão**:
  - Temperatura: ±1°C
  - Pressão: ±1 hPa

## Display OLED SSD1306

### Características

- **Resolução**: 128x64 pixels
- **Interface**: I2C
- **Buffer**: 1024 bytes (128x64/8)
- **Atualização**: Via DMA real

### Estrutura de Exibição

```
┌─────────────────────────┐
│         DMA             │  ← Título (Y=0)
├─────────────────────────┤
│                         │
│  Temp: 28.1C            │  ← Temperatura (Y=24)
├─────────────────────────┤
│  Umidade: 63.1%         │  ← Umidade (Y=40)
│                         │
└─────────────────────────┘
```

## Compilação e Uso

### Pré-requisitos

- Raspberry Pi Pico SDK
- CMake
- Compilador GCC para ARM

### Compilação

```bash
mkdir build
cd build
cmake ..
make
```

### Upload

1. Pressione o botão BOOTSEL no Pico
2. Conecte via USB
3. Copie o arquivo `.uf2` para o dispositivo

### Modo BOOTSEL via Botão

O código implementa um modo BOOTSEL ativado pelo botão B (GPIO 6):

- Pressionar o botão reinicia o Pico em modo BOOTSEL
- Útil para atualizar o firmware sem desconectar o hardware

## Monitoramento Serial

O sistema envia dados via serial para debug:

```
Pressao = 0.000 kPa
Temperatura BMP: = 0.00 C
Altitude estimada: 44330.00 m
Temperatura AHT: 27.47 C
Umidade: 63.12 %
Display - String temp: [Temp: 27.5C]
Display - String umid: [Umidade: 63.1%]
```

## Análise Técnica da Implementação DMA

### Registradores I2C Utilizados

- `i2c_hw->dma_cr`: Controla o DMA do I2C
- `i2c_hw->tar`: Endereço do dispositivo I2C
- `i2c_hw->data_cmd`: Registrador de dados/comando
  - Bits 0-7: Dados
  - Bit 8: START
  - Bit 9: STOP
  - Bit 10: RESTART

### Registradores DMA Utilizados

- `dma_hw->ch[chan].read_addr`: Endereço de leitura
- `dma_hw->ch[chan].write_addr`: Endereço de escrita
- `dma_hw->ch[chan].transfer_count`: Contador de transferências

### Sincronização DREQ

O DREQ (Data Request) do I2C:

- Sinaliza quando o I2C está pronto para receber dados
- Permite que o DMA transfira dados na velocidade correta
- Evita necessidade de polling pela CPU

## Melhorias Futuras

1. **Encadeamento de canais DMA**: Usar dois canais encadeados (como no exemplo PWM) para transferências contínuas
2. **Buffer duplo**: Alternar entre dois buffers para atualizações mais suaves
3. **Interrupções DMA**: Usar interrupções ao invés de polling para maior eficiência
4. **DMA para leitura de sensores**: Implementar DMA também para leitura de dados dos sensores

## Referências

- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [Pico SDK Documentation](https://raspberrypi.github.io/pico-sdk-doxygen/)
- [DMA Examples](https://github.com/raspberrypi/pico-examples/tree/master/dma)
- [I2C Hardware Documentation](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#page=430)

## Autor

Projeto desenvolvido para demonstração de implementação real de DMA no Raspberry Pi Pico.

## Licença

Este projeto é fornecido como exemplo educacional.
