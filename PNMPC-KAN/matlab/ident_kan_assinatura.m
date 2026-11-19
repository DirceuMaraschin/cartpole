function a = ident_kan_assinatura(kan)
%IDENT_KAN_ASSINATURA  Número que identifica a KAN (soma dos |parâmetros|).
%   Vai para o kan_planta_pesos.h (KAN_ASSINATURA) e o ESP32 devolve no handshake:
%   o ident_hil_kan compara com a KAN carregada no MATLAB para garantir que as
%   duas são a mesma rede.

a = 0;
for l = 1:numel(kan.camadas)
    C = kan.camadas{l};
    a = a + sum(abs(C.grid(:))) + sum(abs(C.coef(:))) + sum(abs(C.sb(:))) + ...
        sum(abs(C.sp(:))) + sum(abs(C.a(:))) + sum(abs(C.c(:)));
end
a = a + sum(abs(kan.ent_ganho(:))) + sum(abs(kan.ent_desloc(:))) + ...
    sum(abs(kan.sai_ganho(:))) + sum(abs(kan.sai_desloc(:))) + abs(kan.th_eq);
if isfield(kan, 'lin'), a = a + sum(abs(kan.lin(:))); end
end
