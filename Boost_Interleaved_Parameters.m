%==========================Parameters======================================
% Conversor boost interleaved bifásico do caso de estudo do TCC
% (cap. 3, seção "Caso de Estudo"): chaves ideais e elementos sem perdas.
% Modelo: Boost_Interleaved_TCC.slx (malha aberta, d1 = d2 = D0).
%
% Observação: como o circuito não tem perdas, nada amortece a diferença
% iL1 - iL2. A partida assimétrica do PWM (a fase 1 conduz em t = 0, a fase 2
% só em Ts/2) deixa as fases desequilibradas (~5,7 A e ~4,7 A em vez de
% 5,23 A cada), e a diferença decai só pela pequena Ron_sw. A soma iL1 + iL2,
% vC e as ondulações batem com o ponto de operação do texto.

%% Conversor
Vin  = 250;        % Tensão de entrada (V)
Vref = 280;        % Tensão desejada no barramento (V)
L1   = 600e-6;     % Indutância da fase 1 (H)
L2   = 600e-6;     % Indutância da fase 2 (H)
C    = 10e-6;      % Capacitância do barramento (F)
R    = 30;         % Resistência de carga (Ohm)
fs   = 20e3;       % Frequência de chaveamento (Hz)
Ts   = 1/fs;       % Período de chaveamento (s)

%% Ponto de operação (regime permanente, MCC, sem perdas, d1 = d2 = D0)
D0   = 1 - Vin/Vref;     % Ciclo de trabalho nominal (~10,71 %)
Io0  = Vref/R;           % Corrente de carga (A)
Iin0 = Vref*Io0/Vin;     % Corrente total de entrada (A), balanço de potência
IL0  = Iin0/2;           % Corrente média por fase (A)

%% Ciclos de trabalho aplicados (malha aberta)
d1 = D0;                 % Fase 1
d2 = D0;                 % Fase 2 (portadora defasada de 180°, Ts/2)

%% Chaves "ideais": aproximação numérica no Simscape
Ron_sw  = 1e-3;          % Resistência em condução do MOSFET e do diodo (Ohm)
Goff_sw = 1e-6;          % Condutância em corte do MOSFET e do diodo (S)
Vf_sw   = 1e-3;          % Queda direta do diodo (V); o Simscape exige Vf > 0

%% Condição inicial: conversor pré-carregado com as chaves abertas (d = 0),
% como em PI_parametros.m e BS_parametros.m do TCC
VC_ini = Vin;            % Tensão inicial do barramento (V)
IL_ini = Vin/R/2;        % Corrente inicial por fase (A)

%% Simulação
T_sim = 0.02;            % Tempo total (s)
