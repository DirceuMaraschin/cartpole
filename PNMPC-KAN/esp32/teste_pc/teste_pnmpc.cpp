// Teste no PC do PNMPC-KAN (o MESMO pnmpc_kan.h + kan_eval.h do ESP32) em malha fechada
// com a planta da bancada (mesma de matlab/bancada_planta.m: carrinho imposto pelo motor de
// passo, pêndulo ajustado ao ensaio 154800, encoder de 0,15 grau e estimador de omega do firmware).
//
// Cenário padrão: pêndulo parado em 210 graus do sensor (90 = em pé, 270 = pendurado),
// referência de ângulo 90 graus (em pé) e carrinho em x = 0.
//
// Compilar (na pasta do projeto):
//   g++ -O2 -std=c++11 -I esp32/kan_planta_esp32 esp32/teste_pc/teste_pnmpc.cpp -o teste_pnmpc
// Rodar (sintonia opcional como chave=valor; csv=arquivo grava a série):
//   ./teste_pnmpc q_E=50 u_max=5 th0=210 ref=90 xlim=0.2 Tf=15 csv=saida.csv
#define _USE_MATH_DEFINES
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <chrono>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "kan_planta_pesos.h"
#include "kan_eval.h"
#include "pnmpc_kan.h"

struct Planta {
  double w0q = 44.213, kl = 4.496, c = 0.095;   // bancada_param.mat (ajuste ao ensaio 154800)
  double s[4] = {0, 0, 0, 0};
  double hist[8]; int idx = 0; double th_med = 0, w_med = 0;
  static double quant(double th) { const double q = 2 * M_PI / 2400; return q * std::round(th / q); }
  void reiniciar(double x, double th) {
    s[0] = x; s[1] = 0; s[2] = th; s[3] = 0;
    th_med = quant(th); for (double& h : hist) h = th_med; idx = 0; w_med = 0;
  }
  void f(const double* z, double a, double* d) const {
    d[0] = z[1]; d[1] = a; d[2] = z[3]; d[3] = -w0q * std::sin(z[2]) - kl * a * std::cos(z[2]) - c * z[3];
  }
  void passo(double a, double Ts) {
    const int n = 20; const double h = Ts / n;
    for (int i = 0; i < n; i++) {
      double k1[4], k2[4], k3[4], k4[4], z[4];
      f(s, a, k1); for (int j = 0; j < 4; j++) z[j] = s[j] + h / 2 * k1[j];
      f(z, a, k2); for (int j = 0; j < 4; j++) z[j] = s[j] + h / 2 * k2[j];
      f(z, a, k3); for (int j = 0; j < 4; j++) z[j] = s[j] + h * k3[j];
      f(z, a, k4); for (int j = 0; j < 4; j++) s[j] += h / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
      th_med = quant(s[2]);
      double bruto = (th_med - hist[idx]) / (8 * h);
      w_med = 0.5 * w_med + 0.5 * bruto;
      hist[idx] = th_med; idx = (idx + 1) % 8;
    }
  }
};

static double sensor_para_bancada(double graus) { return (graus + 90.0) * M_PI / 180.0; }   // 90 -> pi, 270 -> 2pi
static double wrapd(double a) { return a - 2 * M_PI * std::floor((a + M_PI) / (2 * M_PI)); }

int main(int argc, char** argv) {
  Planta pl;
  double th0 = 210, ref = 90, xlim = 0.20, Tf = 15, xref = 0;
  static char csv[256] = "";
  for (int i = 1; i < argc; i++) {
    char k[64]; double v;
    if (!strncmp(argv[i], "csv=", 4)) { strncpy(csv, argv[i] + 4, 255); continue; }
    if (sscanf(argv[i], "%63[^=]=%lf", k, &v) != 2) continue;
    if (!strcmp(k, "th0")) th0 = v; else if (!strcmp(k, "ref")) ref = v; else if (!strcmp(k, "xlim")) xlim = v;
    else if (!strcmp(k, "Tf")) Tf = v; else if (!strcmp(k, "xref")) xref = v;
    else if (!strcmp(k, "w0q_planta")) pl.w0q = v; else if (!strcmp(k, "kl_planta")) pl.kl = v; else if (!strcmp(k, "c_planta")) pl.c = v;
    else if (!strcmp(k, "qx")) MPC.q[0] = v; else if (!strcmp(k, "qv")) MPC.q[1] = v;
    else if (!strcmp(k, "qth")) MPC.q[2] = v; else if (!strcmp(k, "qw")) MPC.q[3] = v;
    else if (!strcmp(k, "q_term")) MPC.q_term = v; else if (!strcmp(k, "r_du")) MPC.r_du = v; else if (!strcmp(k, "r_u")) MPC.r_u = v;
    else if (!strcmp(k, "u_max")) { MPC.u_max = v; MPC.u_min = -v; } else if (!strcmp(k, "eps")) MPC.eps = v;
    else if (!strcmp(k, "alfa_n")) MPC.alfa_n = v; else if (!strcmp(k, "iters")) MPC.iters = (int)v;
    else if (!strcmp(k, "q_E")) MPC.q_E = v; else if (!strcmp(k, "E_alvo")) MPC.E_alvo = v;
    else if (!strcmp(k, "phi_sw")) MPC.phi_sw = v * M_PI / 180; else if (!strcmp(k, "q_sw_x")) MPC.q_sw_x = v;
    else if (!strcmp(k, "paralelo")) MPC.paralelo = (int)v;
    else if (!strcmp(k, "q_sw_v")) MPC.q_sw_v = v; else if (!strcmp(k, "q_xs")) MPC.q_xs = v; else if (!strcmp(k, "x_soft")) MPC.x_soft = v;
    else fprintf(stderr, "parametro desconhecido: %s\n", k);
  }
  FILE* fo = csv[0] ? fopen(csv, "w") : nullptr;
  if (fo) fprintf(fo, "t,r,x,v,th,w,u,modo,E\n");

  const double Ts = KAN_TS, th_ref = sensor_para_bancada(ref);
  pl.reiniciar(0.0, sensor_para_bancada(th0));
  mpc_reiniciar();
  double t_cap = -1, e2 = 0, x2 = 0, tcalc = 0, xmax = 0; int n_fim = 0, k_fim = 0; bool fim_curso = false;
  const int N = (int)(Tf / Ts);
  for (int k = 0; k < N; k++) {
    const double t = k * Ts;
    float y[4] = {(float)pl.s[0], (float)pl.s[1], (float)pl.th_med, (float)pl.w_med};
    MpcInfo info;
    auto t0 = std::chrono::high_resolution_clock::now();
    float u = pnmpc_passo(y, (float)xref, (float)th_ref, &info);
    tcalc += std::chrono::duration<double, std::micro>(std::chrono::high_resolution_clock::now() - t0).count();
    const double eth = wrapd(pl.s[2] - th_ref);
    if (fo) fprintf(fo, "%.3f,%.4f,%.6f,%.6f,%.6f,%.6f,%.5f,%d,%.4f\n", t, xref, pl.s[0], pl.s[1], pl.s[2], pl.s[3], u, info.modo, info.E);
    if (std::fabs(eth) < 5 * M_PI / 180) { if (t_cap < 0) t_cap = t; } else if (std::fabs(eth) > 20 * M_PI / 180) t_cap = -1;
    if (t > Tf - 3) { e2 += eth * eth; x2 += (pl.s[0] - xref) * (pl.s[0] - xref); n_fim++; }
    xmax = std::fmax(xmax, std::fabs(pl.s[0]));
    pl.passo(u, Ts); k_fim = k;
    if (std::fabs(pl.s[0]) > xlim) { fim_curso = true; break; }
  }
  if (fo) fclose(fo);
  const bool ok = !fim_curso && t_cap >= 0 && n_fim > 0;
  std::string cap = t_cap >= 0 ? std::to_string(t_cap).substr(0, 5) + " s" : "-";
  printf("%-12s t=%5.2f | captura %-8s | ultimos 3 s: th_rms %6.2f g, x_rms %6.1f mm | |x|max %.3f m | PC %.0f us/passo\n",
         fim_curso ? "FIM DE CURSO" : (ok ? "OK" : "nao capturou"), (k_fim + 1) * Ts, cap.c_str(),
         n_fim ? std::sqrt(e2 / n_fim) * 180 / M_PI : NAN, n_fim ? 1000 * std::sqrt(x2 / n_fim) : NAN,
         xmax, tcalc / (k_fim + 1));
  return ok ? 0 : 2;
}
