%==========================Parameters======================================
% Boost interleaved do caso de estudo do TCC em malha fechada com o PI.
% Modelo: Boost_Interleaved_TCC_PI.slx (gerado por Boost_Interleaved_PI_Build.m).
% O conversor é o mesmo da malha aberta; aqui entram só o PI e a referência.

Boost_Interleaved_Parameters;   % conversor, ponto de operação e chaves

%% Controlador PI: C(s) = (Kp_pi*s + Ki_pi)/s, igual ao do projeto do TCC
% (PI_workspace.m do repositório do TCC; lugar das raízes, zero em -2*w0)
Kp_pi = 1.06335e-4;      % Ganho proporcional
Ki_pi = 3.46678;         % Ganho integral

%% Referência: Vref na partida, depois degraus de +A1 e -A2 (como no PI_sim do TCC)
A1     = 100;            % Degrau de subida (V)
A2     = 100;            % Degrau de descida (V)
T_deg1 = 0.02;           % Instante do degrau de subida (s)
T_deg2 = 0.04;           % Instante do degrau de descida (s)

%% Simulação
T_sim = 0.06;            % Tempo total (s)
