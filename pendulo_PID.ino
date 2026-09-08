#include "arduino_secrets.h"

#include <Arduino.h>
#include <ESP32Encoder.h>

// Pinos de Controle (DRV8825)
const int PIN_STEP = 4;
const int PIN_DIR = 5;
const int PIN_ENABLE = 6;
const int PIN_FIM_CURSO = 8;           // NF (Normalmente Fechado para GND)

// Pinos do Encoder
const int PIN_ENC_A = 17;
const int PIN_ENC_B = 18;

// Constantes de Engenharia
const float PASSO_MM = 0.025;           // 1/8 microstep + polia 20T
const float RAD_POR_CONT = 0.00261799;  // 2400 PPR
const float ALFA = 0.11;                // Filtro EMA 20 Hz
const float DELTA_V = 0.08;             // Incremento de aÃ§Ã£o em m/s
const float V_MAX = 0.80;               // SaturaÃ§Ã£o
const float X_MAX = 0.35;               // Limite de software em metros

ESP32Encoder encoder;
hw_timer_t *timer_filtro = NULL;
hw_timer_t *timer_motor = NULL;

// VariÃ¡veis de Estado
volatile long contagens_ant = 0;
volatile float theta_dot_f = 0.0;
volatile float v_cmd = 0.0;
volatile long pulsos_enviados = 0;
volatile uint8_t flags_hardware = 0;

// SEGURANÃA - FIM DE CURSO
void IRAM_ATTR isr_fim_de_curso() {
    digitalWrite(PIN_ENABLE, HIGH); // DRV8825 desativa as bobinas
    v_cmd = 0.0;
    flags_hardware |= 0b00000001;   // Sinaliza colisÃ£o no bit 0
}

// FILTRAGEM DE ÃNGULO (1kHz)
void IRAM_ATTR isr_1kHz_filtro() {
    long contagens_atuais = encoder.getCount();
    float delta_theta = (contagens_atuais - contagens_ant) * RAD_POR_CONT;
    contagens_ant = contagens_atuais;
    
    float theta_dot_bruto = delta_theta / 0.001; 
    theta_dot_f = (ALFA * theta_dot_bruto) + ((1.0 - ALFA) * theta_dot_f);
}

// CONTROLE DE PASSOS POR FREQUÃNCIA
void IRAM_ATTR isr_motor() {
    if (v_cmd == 0.0 || (flags_hardware & 0b00000001)) return;

    digitalWrite(PIN_DIR, (v_cmd > 0) ? HIGH : LOW);
    
    // Gera pulso
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_STEP, LOW);
    
    pulsos_enviados += (v_cmd > 0) ? 1 : -1;

    // Calcula o prÃ³ximo atraso: T (segundos) = dist_passo / velocidade
    float v_abs = abs(v_cmd);
    uint32_t delay_us = (uint32_t)((PASSO_MM / 1000.0) / v_abs * 1000000.0);
    timerAlarm(timer_motor, delay_us, true, 0);
}

// -------------------------
// ------- PING-PONG -------
// -------------------------

// EMPACOTAMENTO EM BYTES - BAIXA LATÃNCIA
uint8_t calcular_crc8(uint8_t *dados, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= dados[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

void setup() {
    Serial.begin(921600);
    
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_FIM_CURSO, INPUT_PULLUP);
    
    digitalWrite(PIN_ENABLE, LOW); // Habilita driver
    // attachInterrupt(digitalPinToInterrupt(PIN_FIM_CURSO), isr_fim_de_curso, FALLING);
    
    // PCNT para o encoder
    // encoder.attachHalfQuad(PIN_ENC_A, PIN_ENC_B);
    // encoder.clearCount();
    
    // Timer 0: Filtro 1 kHz
    timer_filtro = timerBegin(1000000);         // 1us = 1MHz
    timerAttachInterrupt(timer_filtro, &isr_1kHz_filtro);
    timerAlarm(timer_filtro, 1000, true, 0);    // alarme a casa 1ms
    
    // Timer 1: Motor
    timer_motor = timerBegin(1000000);
    timerAttachInterrupt(timer_motor, &isr_motor);
    timerAlarm(timer_motor, 10000, true, 0);    // Inicia lento (10000 us)
}

void loop() {
    // 1. Montagem e envio do Estado (50 Hz)
    long snapshot_pulsos = pulsos_enviados;
    long snapshot_contagens = encoder.getCount();
    float x_dot = v_cmd; // Velocidade conhecida no firmware
    float th_dot = theta_dot_f;
    
    uint8_t buffer_saida[21];
    buffer_saida[0] = 0x55;
    buffer_saida[1] = 0xAA;
    buffer_saida[2] = 0; // seq (simplificado)
    memcpy(&buffer_saida[3], &snapshot_pulsos, 4);
    memcpy(&buffer_saida[7], &snapshot_contagens, 4);
    memcpy(&buffer_saida[11], &x_dot, 4);
    memcpy(&buffer_saida[15], &th_dot, 4);
    buffer_saida[19] = flags_hardware;
    buffer_saida[20] = calcular_crc8(buffer_saida, 20);
    
    Serial.write(buffer_saida, 21);
    
    // 2. RecepÃ§Ã£o da AÃ§Ã£o (Timeout bloqueante leve)
    uint32_t t_espera = millis();
    while (Serial.available() < 6) {
        if (millis() - t_espera > 10) break; // Timeout de 10ms
    }
    
    if (Serial.available() >= 6) {
        uint8_t buffer_entrada[6];
        Serial.readBytes(buffer_entrada, 6);
        
        if (buffer_entrada[0] == 0x5A && buffer_entrada[1] == 0xA5) {
            if (calcular_crc8(buffer_entrada, 5) == buffer_entrada[5]) {
                uint8_t acao = buffer_entrada[3];
                
                // Aplica a aceleraÃ§Ã£o
                if (acao == 0) v_cmd -= DELTA_V;
                else if (acao == 1) v_cmd += DELTA_V;
                
                // SaturaÃ§Ã£o de velocidade mecÃ¢nica
                if (v_cmd > V_MAX) v_cmd = V_MAX;
                if (v_cmd < -V_MAX) v_cmd = -V_MAX;
                
                // Limite de software
                if (abs(snapshot_pulsos * PASSO_MM / 1000.0) > X_MAX) v_cmd = 0;
            }
        }
    }
    
    // Garante exatos 20ms por ciclo independentemente do processamento
    delay(20 - (millis() - t_espera));
}
