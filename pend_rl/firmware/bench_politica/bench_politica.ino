/* bench_politica.ino -- quanto custa a inferencia NA PLACA.
 *
 * Isolado de proposito: sem stepper, sem encoder, sem serial no laco. Medido
 * junto com o resto do firmware, o numero nao e interpretavel -- nao se sabe
 * o que se esta medindo. Aqui so a rede roda.
 *
 * Roda no ESP32 Dev Module E no ESP32-S3: nao precisa trocar de placa para
 * obter a resposta. LX6 e LX7 a 240 MHz tem FPU de precisao simples e dao
 * numeros da mesma ordem.
 *
 * O QUE ESTE SKETCH RESPONDE
 *   1. Os pesos sobreviveram a flash e a FPU do Xtensa concorda com o torch?
 *   2. Quantos microssegundos custa uma inferencia? (min / media / p99 / max)
 *   3. O tanh e o gargalo, como suspeitamos?
 *   4. Quanto a ISR de 40 kHz do stepper cobra da inferencia?
 *
 * COMO RESPONDER A PERGUNTA 3. Nao por estimativa: grave duas vezes, uma
 * normal e outra com POLITICA_TANH_LUT definido, e compare o total. A
 * diferenca E o custo do tanh. A medicao isolada de tanhf() abaixo e so
 * conferencia -- o custo da libm varia com o argumento.
 *
 * ARQUIVOS. O Arduino so compila o que esta na pasta do sketch, e os pesos
 * vivem em uma copia so, em firmware/pendulo_rl/. Antes de gravar ESTE
 * sketch, traga os tres arquivos para ca:
 *
 *   cp ../pendulo_rl/politica.c ../pendulo_rl/politica.h ../politica/oraculo.h .
 *
 * e apague-os depois, para nao voltarem a divergir. O Arduino compila .c
 * como C e .ino como C++; politica.h ja tem extern "C".
 *
 * PARA LIGAR A LUT: descomente a linha abaixo E grave de novo.
 */
// #define POLITICA_TANH_LUT   <-- nao serve aqui: tem de ir no politica.c
//
// A LUT e escolhida na compilacao de politica.c, nao deste arquivo. Na IDE
// do Arduino, a forma simples e editar politica.c e descomentar o
// "#define POLITICA_TANH_LUT" que o exportar_c.py deixa preparado no topo,
// ou passar a flag em platform.local.txt. Com arduino-cli:
//   arduino-cli compile --build-property compiler.c.extra_flags=-DPOLITICA_TANH_LUT ...

#include <math.h>
#include <stdlib.h>

#include "politica.h"
#include "oraculo.h"

#define N_MED        1000     /* inferencias cronometradas por fase */
#define ISR_HZ       40000    /* a mesma taxa do gerador de passo    */

static uint32_t ciclos[N_MED];
static volatile float sorvedouro = 0.0f;   /* impede o compilador de sumir
                                              com a inferencia inteira */

/* ------------------------------------------------------------------ *
 * ISR de teste: so conta. Nao usa float de proposito.
 *
 * Float em ISR obriga o ESP-IDF a salvar o contexto da FPU a cada
 * interrupcao, e a 40 kHz isso e caro. Se o gerador de passo do firmware
 * usar float, este benchmark subestima o custo real.
 * ------------------------------------------------------------------ */
static hw_timer_t *tmr = NULL;
static volatile uint32_t isr_n = 0;

void IRAM_ATTR naISR() { isr_n++; }

static void ligarISR() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  tmr = timerBegin(1000000);                 /* 1 MHz */
  timerAttachInterrupt(tmr, &naISR);
  timerAlarm(tmr, 1000000 / ISR_HZ, true, 0);
#else
  tmr = timerBegin(0, 80, true);             /* 80 MHz / 80 = 1 MHz */
  timerAttachInterrupt(tmr, &naISR, true);
  timerAlarmWrite(tmr, 1000000 / ISR_HZ, true);
  timerAlarmEnable(tmr);
#endif
}

static void desligarISR() {
  if (!tmr) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerEnd(tmr);
#else
  timerAlarmDisable(tmr);
  timerEnd(tmr);
#endif
  tmr = NULL;
}

/* ------------------------------------------------------------------ */
static int cmp_u32(const void *a, const void *b) {
  uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
  return (x > y) - (x < y);
}

static void medir(const char *rotulo, float mhz) {
  /* aquecimento: o cache de instrucao da flash tem de estar quente, senao a
     primeira dezena de amostras mede o cache, nao a conta */
  for (int i = 0; i < 64; i++)
    sorvedouro += politica_bruta(ORACULO_OBS[i % ORACULO_N]);

  for (int i = 0; i < N_MED; i++) {
    const float *o = ORACULO_OBS[i % ORACULO_N];
    uint32_t c0 = ESP.getCycleCount();
    float y = politica_bruta(o);
    uint32_t c1 = ESP.getCycleCount();
    ciclos[i] = c1 - c0;            /* unsigned: o wrap se resolve sozinho */
    sorvedouro += y;
  }

  /* total por blocos: media exata, sem o custo de ler o contador */
  uint32_t b0 = ESP.getCycleCount();
  for (int i = 0; i < N_MED; i++)
    sorvedouro += politica_bruta(ORACULO_OBS[i % ORACULO_N]);
  uint32_t b1 = ESP.getCycleCount();
  double media_us = (double)(b1 - b0) / N_MED / mhz;

  qsort(ciclos, N_MED, sizeof(uint32_t), cmp_u32);
  double mn  = ciclos[0] / mhz;
  double p50 = ciclos[N_MED / 2] / mhz;
  double p99 = ciclos[(N_MED * 99) / 100] / mhz;
  double mx  = ciclos[N_MED - 1] / mhz;

  Serial.printf("%-22s min %7.2f  p50 %7.2f  p99 %7.2f  max %7.2f us\n",
                rotulo, mn, p50, p99, mx);
  Serial.printf("%-22s media (por bloco) %7.2f us  = %.2f%% do passo de 20 ms\n",
                "", media_us, 100.0 * media_us / 20000.0);
}

/* ------------------------------------------------------------------ */
static void conferirOraculo() {
  float pior = 0.0f;
  int i_pior = 0;
  for (int i = 0; i < ORACULO_N; i++) {
    float e = fabsf(politica_bruta(ORACULO_OBS[i]) - ORACULO_REF[i]);
    if (e > pior) { pior = e; i_pior = i; }
  }
  Serial.printf("oraculo       %d vetores, erro max %.3e no #%d\n",
                ORACULO_N, pior, i_pior);
  if (pior > 1e-3f)
    Serial.println("  >> FALHOU: os pesos nao sobreviveram, ou a FPU discorda. "
                   "PARE AQUI.");
  else if (pior > 1e-5f)
    Serial.println("  >> acima de 1e-5: esperado SO se a LUT estiver ligada.");
  else
    Serial.println("  >> ok, dentro do arredondamento de float32.");
}

static void medirTanh(float mhz) {
  /* 128 argumentos cobrindo a faixa tipica das pre-ativacoes.
     Aproximado: o custo da libm varia com o argumento, e a faixa real
     depende dos pesos. Serve de conferencia, nao de medida fina. */
  static float arg[128];
  for (int i = 0; i < 128; i++) arg[i] = -3.0f + 6.0f * i / 127.0f;

  volatile float s = 0.0f;
  for (int i = 0; i < 128; i++) s += tanhf(arg[i]);     /* aquece */

  uint32_t c0 = ESP.getCycleCount();
  for (int r = 0; r < 100; r++)
    for (int i = 0; i < 128; i++) s += tanhf(arg[i]);
  uint32_t c1 = ESP.getCycleCount();
  sorvedouro += s;

  double us_128 = (double)(c1 - c0) / 100.0 / mhz;
  Serial.printf("tanhf x128 (libm)      %7.2f us  = %.0f ciclos por chamada\n",
                us_128, (double)(c1 - c0) / 100.0 / 128.0);
}

/* ------------------------------------------------------------------ */
void setup() {
  Serial.begin(115200);
  delay(1500);

  float mhz = (float)getCpuFrequencyMhz();

  Serial.println();
  Serial.println("=== bench_politica ===");
  Serial.printf("chip          %s, %d nucleo(s), %.0f MHz, core atual %d\n",
                ESP.getChipModel(), ESP.getChipCores(), mhz, xPortGetCoreID());
  Serial.printf("flash         %u bytes\n", (unsigned)ESP.getFlashChipSize());
  Serial.printf("PSRAM         %u bytes (nao usada: SRAM interna e mais rapida)\n",
                (unsigned)ESP.getPsramSize());
  Serial.printf("heap livre    %u bytes\n", (unsigned)ESP.getFreeHeap());
  Serial.printf("rede          %d -> %d -> %d -> %d, %d parametros, %d bytes\n",
                POLITICA_DIM_OBS, POLITICA_OCULTA, POLITICA_OCULTA,
                POLITICA_DIM_ACAO, POLITICA_N_PARAM, 4 * POLITICA_N_PARAM);
#ifdef POLITICA_TANH_LUT
  Serial.println("tanh          LUT (politica.c compilado com POLITICA_TANH_LUT)");
#else
  Serial.println("tanh          tanhf() da libm");
#endif
  Serial.println();

  politica_iniciar();
  conferirOraculo();
  Serial.println();

  medir("inferencia isolada", mhz);
  medirTanh(mhz);
  Serial.println();

  Serial.printf("ligando ISR a %d Hz...\n", ISR_HZ);
  isr_n = 0;
  ligarISR();
  delay(200);
  uint32_t n0 = isr_n;
  medir("inferencia + ISR", mhz);
  uint32_t n1 = isr_n;
  desligarISR();
  Serial.printf("%-22s ISR disparou %u vezes durante a medicao\n",
                "", (unsigned)(n1 - n0));

  Serial.println();
  Serial.println("Leitura dos numeros:");
  Serial.println("  < 1000 us  -> confortavel nos 20 ms; nao precisa de LUT.");
  Serial.println("  o p99 e o que importa, nao a media: o laco de controle");
  Serial.println("  tem de fechar TODO passo, nao em media.");
  Serial.println("  para medir o custo do tanh, grave de novo com a LUT");
  Serial.println("  ligada em politica.c e compare o total.");
  Serial.printf("  (sorvedouro = %.3f -- ignorar, existe para o compilador "
                "nao otimizar a conta)\n", sorvedouro);
}

void loop() { delay(1000); }
