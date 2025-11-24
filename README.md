# AANG – Alimentador Automático de Ração para Gatos

Firmware para controle de um alimentador automático de ração, baseado em ESP32, com dosagem por rosca transportadora e realimentação de massa via célula de carga.

O sistema permite:
- Agendar até 5 refeições diárias com horário e massa configuráveis;
- Acionar uma porção fixa por botão físico;
- Executar alimentação manual pela interface serial ou pela interface web local;
- Verificar nível de ração no reservatório antes de cada refeição;
- Registrar um histórico das refeições realizadas.

---

## Hardware principal

- **ESP32 DevKit (WROVER/DevKitC)**
- **Célula de carga + módulo HX711**
  - DOUT → GPIO 18  
  - SCK  → GPIO 19
- **Motor de passo NEMA 17 + driver TMC2209 (STEP/DIR/EN)**
  - DIR  → GPIO 27  
  - STEP → GPIO 26  
  - EN   → GPIO 32 (ativo em nível baixo)
- **RTC DS3231 (I²C)**
  - SDA → GPIO 21  
  - SCL → GPIO 22
- **Sensor de distância VL53L0X (I²C)**
  - Compartilha SDA/SCL com o DS3231
- **Botão de porção fixa**
  - Botão → GPIO 25 (entrada com 'INPUT_PULLUP')
- **LEDs**
  - LED status motor → GPIO 13  
  - LED nível baixo  → GPIO 4

---

## Bibliotecas utilizadas

Instalar na Arduino IDE:

- ['HX711_ADC'](https://github.com/olkal/HX711_ADC)
- ['AccelStepper'](https://www.airspayce.com/mikem/arduino/AccelStepper/)
- ['RTClib'](https://github.com/adafruit/RTClib)
- ['VL53L0X'](https://github.com/pololu/vl53l0x-arduino)


Placa selecionada: **ESP32 Dev Module**.

---

## Funcionamento geral

### Controle de massa

1. O ESP32 lê a massa no prato pela célula de carga (HX711 calibrado).
2. Cada passo do motor é associado a uma massa média 'm_pass0' (g/passo).
3. Para uma refeição com massa de referência 'm_ref':
   - **Pré-dosagem** em malha aberta (~80% de 'm_ref');
   - **Acomodação** (tempo fixo para os grãos estabilizarem);
   - **Medição** da massa servida;
   - **Ajuste incremental** em malha fechada, com pequenos blocos de micropassos até que o erro absoluto seja menor ou igual a '2 g'.

O algoritmo é implementado como uma máquina de estados (inicialização, pré-dosagem, acomodação, medição, avaliação do erro, ajuste incremental e término).

### Nível do reservatório

O sensor VL53L0X mede a distância até a superfície da ração no reservatório.  
Se a distância exceder um limite configurado, o sistema considera **nível baixo**, acende o LED no GPIO 4 e **bloqueia novas refeições**.

---

## Interface web

O ESP32 atua como ponto de acesso Wi-Fi e servidor HTTP simples.

- **SSID:** 'AANG'  
- **Senha:** 'aangatinho'  
- **IP padrão do AP:** '192.168.4.1'

Após subir o firmware:

1. Conectar o celular ou notebook à rede 'AANG'.
2. Abrir o navegador em 'http://192.168.4.1'.

### Página “Refeições agendadas” ('/')

- Lista os 5 slots de refeição (horário, porção em gramas e estado habilitado/desabilitado).
- Permite ligar/desligar cada agendamento por um “switch”.
- Botão **“ALIMENTAR AGORA”**: dispara uma refeição imediata com porção padrão de 10 g.
- Formulário **“Novo agendamento”**:
  - Slot (1 a 5),
  - Horário (hh:mm),
  - Massa (g),
  - Flag de habilitado.

### Página “Histórico” ('/historico')

- Lista as refeições realizadas (ordem do mais recente para o mais antigo).
- Colunas: **Data**, **Horário**, **Quantidade**.
- O registro é gravado automaticamente ao final de cada refeição (agendada, via botão, serial ou web).

---

## Interface serial

A comunicação serial é usada para depuração e comandos diretos.  
Baud rate: **115200**.

Principais comandos:

- 'h' – mostra ajuda com os comandos.
- 'f' – inicia refeição manual com a massa de referência atual.
- 'm X' – define 'm_ref = X g' (ex.: 'm 25.5').
- Linha contendo apenas um número (ex.: '30') – também define 'm_ref'.
- 'now' / 'time' – mostra a leitura atual do RTC.
- 'list' – lista as refeições agendadas (1..5).
- 'add i hh:mm M' – adiciona/edita refeição no slot 'i'.  
  Ex.: 'add 1 08:30 25.0'
- 'edit i hh:mm M' – edita refeição existente.
- 'del i' – remove refeição do slot 'i'.
- 'on i' / 'off i' – habilita/desabilita a refeição do slot 'i'.

O botão físico no GPIO 25 aciona uma porção fixa de **10 g**, desde que não haja outra refeição em andamento.

---

## Observações

- O valor do fator de calibração do HX711 ('fatorCalibracao') e da massa média por passo ('massaPorPassoBaseG') foram obtidos por ensaios experimentais.
- O RTC DS3231 é usado como referência de horário para o agendamentoo.
