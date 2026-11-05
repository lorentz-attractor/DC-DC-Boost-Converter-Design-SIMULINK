function Boost_Interleaved_Build(sobrescrever)
% BOOST_INTERLEAVED_BUILD Gera Boost_Interleaved_TCC.slx por script.
%   Boost interleaved bifásico do caso de estudo do TCC, chaveado, em Simscape
%   Electrical, malha aberta (d1 e d2 constantes, PWM com portadora dente de serra;
%   fase 2 defasada de Ts/2). Os valores ficam em Boost_Interleaved_Parameters.m:
%   o modelo só referencia as variáveis, então os parâmetros não precisam estar
%   no workspace para gerar o modelo, só para simular.
%
%   Boost_Interleaved_Build        gera o modelo; recusa se o .slx já existir
%   Boost_Interleaved_Build(true)  gera e sobrescreve o .slx existente

if nargin < 1, sobrescrever = false; end
pasta = fileparts(mfilename('fullpath'));
mdl   = 'Boost_Interleaved_TCC';
arq   = fullfile(pasta, [mdl '.slx']);
if isfile(arq) && ~sobrescrever
    error('O arquivo %s já existe. Use Boost_Interleaved_Build(true) para sobrescrever.', arq);
end
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);

lib.fonte  = 'fl_lib/Electrical/Electrical Sources/DC Voltage Source';
lib.ind    = 'fl_lib/Electrical/Electrical Elements/Inductor';
lib.cap    = 'fl_lib/Electrical/Electrical Elements/Capacitor';
lib.res    = 'fl_lib/Electrical/Electrical Elements/Resistor';
lib.diodo  = 'fl_lib/Electrical/Electrical Elements/Diode';
lib.mosfet = 'ee_lib/Semiconductors & Converters/MOSFET (Ideal, Switching)';
lib.amp    = 'fl_lib/Electrical/Electrical Sensors/Current Sensor';
lib.volt   = 'fl_lib/Electrical/Electrical Sensors/Voltage Sensor';
lib.terra  = 'fl_lib/Electrical/Electrical Elements/Electrical Reference';
lib.solver = 'nesl_utility/Solver Configuration';
lib.s2ps   = 'nesl_utility/Simulink-PS Converter';
lib.ps2s   = 'nesl_utility/PS-Simulink Converter';

novo = @(src, nome, pos, ori) add_block(src, [mdl '/' nome], 'Position', pos, 'Orientation', ori);

%% Circuito de potência
novo(lib.fonte, 'Vin', [ 60 200 100 260], 'down');   set_param([mdl '/Vin'], 'v0', 'Vin');
novo(lib.amp,   'Medidor iin', [130  80 170 120], 'right');

fases = {'1', 90; '2', 260};   % número da fase e altura (y) da linha da fase
for k = 1:2
    n = fases{k,1}; y = fases{k,2};
    novo(lib.ind,    ['L' n],          [220 y    260 y+20],  'right');
    set_param([mdl '/L' n], 'l', ['L' n], 'r', '0', 'g', '0', ...
        'i_L_specify', 'on', 'i_L_priority', 'High', 'i_L', 'IL_ini');
    novo(lib.amp,    ['Medidor iL' n], [290 y-10 330 y+30],  'right');
    novo(lib.mosfet, ['S' n],          [380 y+50 420 y+110], 'right');
    % chave "ideal": Ron e Goff pequenos, sem diodo interno (diode_param = 1)
    set_param([mdl '/S' n], 'Rds', 'Ron_sw', 'Goff', 'Goff_sw', 'Vth', '0.5', 'diode_param', '1');
    novo(lib.diodo,  ['D' n],          [470 y    510 y+20],  'right');
    set_param([mdl '/D' n], 'Vf', 'Vf_sw', 'Ron', 'Ron_sw', 'Goff', 'Goff_sw');
end

novo(lib.cap,    'C',          [600 180 620 230], 'down');
set_param([mdl '/C'], 'c', 'C', 'r', '0', 'g', '0', 'vc_specify', 'on', 'vc_priority', 'High', 'vc', 'VC_ini');
novo(lib.res,    'R',          [680 180 700 230], 'down');  set_param([mdl '/R'], 'R', 'R');
novo(lib.volt,   'Medidor vC', [760 180 800 220], 'down');
novo(lib.terra,  'Terra',      [ 60 420 100 440], 'down');
novo(lib.solver, 'Solver Configuration', [130 410 180 440], 'right');

%% Comando: PWM com portadora dente de serra (0 a 1) em fs; fase 2 adiantada de Ts/2
% As portadoras vêm do relógio (rem); o Repeating Sequence não aceita o salto da fase 2.
add_block('simulink/Sources/Clock', [mdl '/Relógio'], 'Position', [-40 530 -10 550]);
for k = 1:2
    n = fases{k,1}; y = 480 + 90*(k-1);
    add_block('simulink/Sources/Constant', [mdl '/d' n], 'Position', [ 40 y 100 y+20], 'Value', ['d' n]);
    add_block('simulink/User-Defined Functions/Fcn', [mdl '/Portadora ' n], 'Position', [ 40 y+35 100 y+65], ...
        'Expr', ['rem(u' repmat(' + Ts/2', 1, k-1) ', Ts)/Ts']);
    add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/Compara ' n], ...
        'Position', [150 y+5 180 y+50], 'Operator', '>');
    add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Gate ' n], ...
        'Position', [210 y+15 250 y+40], 'OutDataTypeStr', 'double');
    novo(lib.s2ps, ['S-PS ' n], [280 y+18 300 y+38], 'right');
    add_block('simulink/Sinks/To Workspace', [mdl '/d' n '_sim'], 'Position', [150 y-25 220 y-5], ...
        'VariableName', ['d' n '_sim'], 'SaveFormat', 'Timeseries', 'ShowName', 'off');
end

%% Medições: conversores PS -> Simulink, To Workspace e scopes
med  = {'Medidor iin','iin','A'; 'Medidor iL1','iL1','A'; 'Medidor iL2','iL2','A'; 'Medidor vC','vC','V'};
ymed = [30 70 110 150];
for k = 1:size(med,1)
    y = ymed(k);
    novo(lib.ps2s, ['PS-S ' med{k,2}], [900 y 920 y+20], 'right');
    set_param([mdl '/PS-S ' med{k,2}], 'Unit', med{k,3});
    add_block('simulink/Sinks/To Workspace', [mdl '/' med{k,2} '_sim'], ...
        'Position', [1060 y 1130 y+20], 'VariableName', [med{k,2} '_sim'], 'SaveFormat', 'Timeseries');
end
add_block('simulink/Sinks/Scope', [mdl '/Correntes'], 'Position', [980 230 1010 290], 'NumInputPorts', '3');
add_block('simulink/Sinks/Scope', [mdl '/Tensão'],    'Position', [980 320 1010 350]);
add_block('simulink/Sinks/Scope', [mdl '/Ciclos'],    'Position', [330 690 360 730], 'NumInputPorts', '2');

%% Ligações
% Portas: 2 terminais LConn1 = +, RConn1 = -; sensores LConn1 = +, RConn1 = sinal, RConn2 = -;
% MOSFET LConn1 = G, RConn1 = D, RConn2 = S
P = @(b) get_param([mdl '/' b], 'PortHandles');
liga = @(a, b) add_line(mdl, a, b, 'autorouting', 'smart');
vin = P('Vin'); ain = P('Medidor iin'); cap = P('C'); res = P('R'); vs = P('Medidor vC');
gnd = P('Terra'); sc = P('Solver Configuration');

liga(vin.LConn(1), ain.LConn(1));                     % fonte + -> medidor da corrente de entrada
for k = 1:2
    n = fases{k,1};
    ind = P(['L' n]); amp = P(['Medidor iL' n]); sw = P(['S' n]); dio = P(['D' n]);
    liga(ain.RConn(2), ind.LConn(1));                 % nó de entrada -> indutor da fase
    liga(ind.RConn(1), amp.LConn(1));                 % indutor -> medidor da fase
    liga(amp.RConn(2), sw.RConn(1));                  % nó de chaveamento -> dreno
    liga(sw.RConn(1), dio.LConn(1));                  % nó de chaveamento -> anodo
    liga(dio.RConn(1), cap.LConn(1));                 % catodo -> barramento
    liga(sw.RConn(2), gnd.LConn(1));                  % fonte do MOSFET -> terra
    % comando
    liga(P(['d' n]).Outport(1), P(['Compara ' n]).Inport(1));
    liga(P('Relógio').Outport(1), P(['Portadora ' n]).Inport(1));
    liga(P(['Portadora ' n]).Outport(1), P(['Compara ' n]).Inport(2));
    liga(P(['Compara ' n]).Outport(1), P(['Gate ' n]).Inport(1));
    liga(P(['Gate ' n]).Outport(1), P(['S-PS ' n]).Inport(1));
    liga(P(['S-PS ' n]).RConn(1), sw.LConn(1));
    liga(P(['d' n]).Outport(1), P(['d' n '_sim']).Inport(1));
    liga(P(['d' n]).Outport(1), P('Ciclos').Inport(k));
end
liga(cap.LConn(1), res.LConn(1));
liga(res.LConn(1), vs.LConn(1));
liga(sc.RConn(1), gnd.LConn(1));                      % terra em cadeia
liga(gnd.LConn(1), vin.RConn(1));
liga(vin.RConn(1), cap.RConn(1));
liga(cap.RConn(1), res.RConn(1));
liga(res.RConn(1), vs.RConn(2));

liga(ain.RConn(1), P('PS-S iin').LConn(1));
liga(P('Medidor iL1').RConn(1), P('PS-S iL1').LConn(1));
liga(P('Medidor iL2').RConn(1), P('PS-S iL2').LConn(1));
liga(vs.RConn(1), P('PS-S vC').LConn(1));
for k = 1:size(med,1)
    liga(P(['PS-S ' med{k,2}]).Outport(1), P([med{k,2} '_sim']).Inport(1));
end
liga(P('PS-S iL1').Outport(1), P('Correntes').Inport(1));
liga(P('PS-S iL2').Outport(1), P('Correntes').Inport(2));
liga(P('PS-S iin').Outport(1), P('Correntes').Inport(3));
liga(P('PS-S vC').Outport(1),  P('Tensão').Inport(1));

%% Nomes dos sinais (legendas dos scopes)
nomeia = @(ph, nome) set_param(get_param(ph, 'Line'), 'Name', nome);
nomeia(P('d1').Outport(1), 'd1');
nomeia(P('d2').Outport(1), 'd2');
for k = 1:size(med,1)
    nomeia(P(['PS-S ' med{k,2}]).Outport(1), med{k,2});
end

%% Scopes: título, legenda e estilo das linhas
titulos = {'Correntes', 'Correntes (A)'; 'Tensão', 'Tensão no barramento (V)'; 'Ciclos', 'Ciclos de trabalho'};
for k = 1:size(titulos,1)
    cfg = get_param([mdl '/' titulos{k,1}], 'ScopeConfiguration');
    cfg.Title = titulos{k,2};
    cfg.ShowLegend = true;
end
cfg = get_param([mdl '/Ciclos'], 'ScopeConfiguration');
cfg.YLimits = [0 0.2];
% d1: linha contínua azul; d2: pontilhado laranja (iguais em malha aberta, d2 fica por cima)
g = jsondecode(get_param([mdl '/Ciclos'], 'GraphicalSettings'));
g.GraphicalSettings.Style.Style.LineStyle = {'-'; ':'};
g.GraphicalSettings.Style.Style.LineColor = [0 0.447 0.741; 0.850 0.325 0.098];
g.GraphicalSettings.Style.Style.LineWidth = [1.5; 2];
set_param([mdl '/Ciclos'], 'GraphicalSettings', jsonencode(g));

%% Configuração da simulação
set_param(mdl, 'Solver', 'ode23t', 'StopTime', 'T_sim', 'MaxStep', 'Ts/50', 'ReturnWorkspaceOutputs', 'off');
Simulink.BlockDiagram.routeLine(find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line'));
save_system(mdl, arq);
fprintf('Modelo gerado: %s\n', arq);
end
