// Teste no PC do mesmo código C++ que roda no ESP32 (kan_eval.h + kan_planta_pesos.h).
// Compila e roda (na pasta do projeto):
//   g++ -O2 -std=c++11 -I esp32/kan_planta_esp32 esp32/teste_pc/teste_kan_eval.cpp -o teste_kan && ./teste_kan
// Compara a previsão em float com a do MATLAB (double) gravada no .h. Erro esperado < 1e-4.
#include <cstdio>
#include <cmath>
#include "kan_planta_pesos.h"
#include "kan_eval.h"

int main() {
  const char* nomes[4] = {"x", "dx", "th", "dth"};
  float erro[4] = {0, 0, 0, 0};
  for (int t = 0; t < KAN_N_TESTE; t++) {
    float s1[4];
    kan_prever(KAN_TESTE_S[t], KAN_TESTE_U[t], s1);
    for (int i = 0; i < 4; i++) {
      float e = std::fabs(s1[i] - KAN_TESTE_S1[t][i]);
      if (e > erro[i]) erro[i] = e;
    }
  }
  std::printf("KAN %d-%d-%d, %d vetores de teste\n", KAN_TAM[0], KAN_TAM[1], KAN_TAM[2], KAN_N_TESTE);
  float emax = 0;
  for (int i = 0; i < 4; i++) {
    std::printf("  erro max %-3s = %.3e\n", nomes[i], erro[i]);
    if (erro[i] > emax) emax = erro[i];
  }
  bool ok = emax < 1e-4f;
  std::printf("%s\n", ok ? "OK: C++ (float) bate com o MATLAB" : "FALHOU: C++ difere do MATLAB");
  return ok ? 0 : 1;
}
