/* estado_rl.h -- o instantaneo do estado passado para o pacote e para a rede.
 *
 * POR QUE ISTO E UM HEADER E NAO FICA NO .ino: o pre-processador do Arduino
 * gera os prototipos das funcoes e os injeta no TOPO do sketch, acima de
 * qualquer struct definida ali. O resultado e
 *
 *     error: 'EstadoRL' does not name a type
 *
 * em toda funcao que receba ou devolva o tipo. Definido num header incluido,
 * o tipo passa a ser conhecido antes dos prototipos injetados.
 *
 * POR QUE UM INSTANTANEO, e nao leituras separadas: `posicao` e a contagem do
 * encoder sao alteradas por ISR. Se o pacote lesse um valor e a observacao da
 * rede lesse outro, a comparacao do modo observador acusaria diferenca onde
 * nao ha bug -- e, pior, o agente decidiria sobre um estado que nunca foi
 * transmitido. Lendo uma vez, o que a rede ve e EXATAMENTE o que o PC recebe.
 *
 * Isso sai de graca porque o pacote carrega v e omega como float32 e pos e enc
 * como int32: nao ha quantizacao no caminho. A comparacao fecha bit a bit sem
 * que a rede veja um estado pior que o do PC.
 */

#ifndef ESTADO_RL_H
#define ESTADO_RL_H

struct EstadoRL {
  long  pos;      /* passos, contados pela ISR do gerador de passo */
  long  enc;      /* contagens do encoder, 2400 por volta          */
  float v;        /* m/s                                           */
  float omega;    /* rad/s                                         */
};

#endif /* ESTADO_RL_H */
