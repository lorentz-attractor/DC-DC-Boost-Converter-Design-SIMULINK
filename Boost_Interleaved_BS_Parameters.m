%==========================Parameters======================================
% Boost interleaved do caso de estudo do TCC em malha fechada com o Backstepping
% integral. Modelo: Boost_Interleaved_TCC_BS.slx (gerado por Boost_Interleaved_BS_Build.m).
% O conversor é o mesmo da malha aberta; aqui entram só o controlador e a referência.

Boost_Interleaved_Parameters;   % conversor, ponto de operação e chaves

%% Backstepping integral (cap. 4 do TCC; backstepping_workspace.m e BS_parametros.m)
% kv iguala o ts do PI linear (3,63 ms) na partida 250 -> 280 V; ki = 10*kv
b_ts = (Vref^2 - (Vref - 0.02*(Vref - Vin))^2)/(Vref^2 - Vin^2);  % faixa de 2 % em energia
kv   = log(1/b_ts)/3.63e-3;     % Ganho da malha de tensão (= 1062,8 1/s)
ki   = 10*kv;                   % Ganho da malha de corrente (= 10628 rad/s)
Leq  = L1*L2/(L1 + L2);         % Indutância equivalente (H)
y0   = C/2*VC_ini^2;            % Energia inicial no capacitor (J): superfície com xi_v(0) = 0

%% Referência: Vref na partida, depois degraus de +A1 e -A2 (iguais aos do modelo com PI)
A1     = 100;            % Degrau de subida (V)
A2     = 100;            % Degrau de descida (V)
T_deg1 = 0.02;           % Instante do degrau de subida (s)
T_deg2 = 0.04;           % Instante do degrau de descida (s)

%% Simulação
T_sim = 0.06;            % Tempo total (s)
