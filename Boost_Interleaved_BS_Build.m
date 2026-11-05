function Boost_Interleaved_BS_Build(sobrescrever)
% BOOST_INTERLEAVED_BS_BUILD Gera Boost_Interleaved_TCC_BS.slx a partir da malha aberta.
%   Carrega Boost_Interleaved_TCC.slx (que não é alterado) e salva uma cópia em
%   malha fechada: as constantes d1 e d2 dão lugar ao Backstepping integral do
%   cap. 4 do TCC, escrito como equações (bloco MATLAB Function), como no modelo
%   usado no projeto. As integrais dos erros ficam em integradores fora do bloco.
%   Malha de tensão com a superfície redefinida (xi_v(0) = 0), malha de corrente
%   com a superfície original e di_ref/dt calculada pelo modelo.
%   A mesma saída d comanda as duas fases. Valores em Boost_Interleaved_BS_Parameters.m.
%
%   Boost_Interleaved_BS_Build        gera o modelo; recusa se o .slx já existir
%   Boost_Interleaved_BS_Build(true)  gera e sobrescreve o .slx existente

if nargin < 1, sobrescrever = false; end
pasta = fileparts(mfilename('fullpath'));
base  = 'Boost_Interleaved_TCC';
mdl   = 'Boost_Interleaved_TCC_BS';
arq   = fullfile(pasta, [mdl '.slx']);
if isfile(arq) && ~sobrescrever
    error('O arquivo %s já existe. Use Boost_Interleaved_BS_Build(true) para sobrescrever.', arq);
end
if bdIsLoaded(mdl),  close_system(mdl, 0);  end
if bdIsLoaded(base), close_system(base, 0); end
load_system(fullfile(pasta, [base '.slx']));
save_system(base, arq);          % daqui em diante o modelo carregado é a cópia
P = @(b) get_param([mdl '/' b], 'PortHandles');
liga = @(a, b) add_line(mdl, a, b, 'autorouting', 'smart');

%% Remove as constantes d1 e d2 (ciclos de trabalho fixos da malha aberta)
for n = {'d1', 'd2'}
    lh = get_param([mdl '/' n{1}], 'LineHandles');
    delete_line(lh.Outport(ishandle(lh.Outport)));
    delete_block([mdl '/' n{1}]);
end

%% Referência: Vref + degrau de +A1 em T_deg1 - degrau de A2 em T_deg2
add_block('simulink/Sources/Constant', [mdl '/Vref'], 'Position', [-640 370 -590 390], 'Value', 'Vref');
add_block('simulink/Sources/Step', [mdl '/Degrau +'], 'Position', [-640 415 -610 445], ...
    'Time', 'T_deg1', 'Before', '0', 'After', 'A1');
add_block('simulink/Sources/Step', [mdl '/Degrau -'], 'Position', [-640 465 -610 495], ...
    'Time', 'T_deg2', 'Before', '0', 'After', 'A2');
add_block('simulink/Math Operations/Sum', [mdl '/Referência'], 'Position', [-550 365 -525 500], ...
    'Inputs', '++-', 'IconShape', 'rectangular');

%% Backstepping: equações do cap. 4 (erromalhatensao, superficietensaomod, irefmod,
% erromalhacorrente, leid); entradas medidas vC, iL1, iL2 e a referência
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Backstepping'], ...
    'Position', [-330 360 -170 560]);
codigo = strjoin({ ...
    'function [d, dzv, dzi, iref] = Backstepping(vc, iL1, iL2, vref, zv, zi, Vin, C, R, Leq, kv, ki, y0)', ...
    '% Backstepping integral do cap. 4 do TCC (d1 = d2 = d). zv = int(eps_v), zi = int(eps_i).', ...
    'itot = iL1 + iL2;', ...
    '% Malha de tensão: erro de energia e superfície redefinida, xi_v(0) = 0', ...
    'y    = C/2*vc^2;', ...
    'ev   = C/2*vref^2 - y;', ...
    'xi_v = (y0 - y)/kv + zv;', ...
    'iref = (kv^2*xi_v + kv*ev + vc^2/R)/Vin;          % yref_dot = 0 e i_ch = vo/R', ...
    '% di_ref/dt pelo modelo: y_dot = Vin*itot - vo*i_ch', ...
    'ydot  = Vin*itot - vc^2/R;', ...
    'diref = (kv^2*(-ydot/kv + ev) - kv*ydot + 2/(R*C)*ydot)/Vin;', ...
    '% Malha de corrente: superfície original e lei do ciclo de trabalho', ...
    'ei   = iref - itot;', ...
    'xi_i = ei/ki + zi;', ...
    'd    = 1 - (Vin - Leq*(diref + ki*ei + ki^2*xi_i))/vc;', ...
    'dzv  = ev;', ...
    'dzi  = ei;', ...
    'end'}, newline);
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/Backstepping']);
ch.Script = codigo;
for n = {'Vin', 'C', 'R', 'Leq', 'kv', 'ki', 'y0'}   % constantes lidas do workspace
    dado = ch.find('-isa', 'Stateflow.Data', 'Name', n{1});
    dado.Scope = 'Parameter';
end
add_block('simulink/Continuous/Integrator', [mdl '/Int ev'], 'Position', [-420 560 -390 590], ...
    'InitialCondition', '0');
add_block('simulink/Continuous/Integrator', [mdl '/Int ei'], 'Position', [-420 620 -390 650], ...
    'InitialCondition', '0');
add_block('simulink/Sinks/To Workspace', [mdl '/vref_sim'], 'Position', [-480 300 -410 320], ...
    'VariableName', 'vref_sim', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/iref_sim'], 'Position', [-110 300 -40 320], ...
    'VariableName', 'iref_sim', 'SaveFormat', 'Timeseries');

liga(P('Vref').Outport(1),       P('Referência').Inport(1));
liga(P('Degrau +').Outport(1),   P('Referência').Inport(2));
liga(P('Degrau -').Outport(1),   P('Referência').Inport(3));
liga(P('PS-S vC').Outport(1),    P('Backstepping').Inport(1));
liga(P('PS-S iL1').Outport(1),   P('Backstepping').Inport(2));
liga(P('PS-S iL2').Outport(1),   P('Backstepping').Inport(3));
liga(P('Referência').Outport(1), P('Backstepping').Inport(4));
liga(P('Int ev').Outport(1),     P('Backstepping').Inport(5));
liga(P('Int ei').Outport(1),     P('Backstepping').Inport(6));
liga(P('Backstepping').Outport(2), P('Int ev').Inport(1));
liga(P('Backstepping').Outport(3), P('Int ei').Inport(1));
liga(P('Backstepping').Outport(4), P('iref_sim').Inport(1));
liga(P('Referência').Outport(1), P('vref_sim').Inport(1));
for k = 1:2
    n = num2str(k);
    liga(P('Backstepping').Outport(1), P(['Compara ' n]).Inport(1));
    liga(P('Backstepping').Outport(1), P(['d' n '_sim']).Inport(1));
    liga(P('Backstepping').Outport(1), P('Ciclos').Inport(k));
end

%% Scopes: referência junto de vC; faixa de d para os degraus
set_param([mdl '/Tensão'], 'NumInputPorts', '2');
liga(P('Referência').Outport(1), P('Tensão').Inport(2));
nomeia = @(ph, nome) set_param(get_param(ph, 'Line'), 'Name', nome);
nomeia(P('Backstepping').Outport(1), 'd');
nomeia(P('Backstepping').Outport(4), 'iref');
nomeia(P('Referência').Outport(1), 'vref');
cfg = get_param([mdl '/Ciclos'], 'ScopeConfiguration');
cfg.YLimits = [0 0.5];

%% Simulação
% d depende de vC, iL1 e iL2 medidas sem atraso e, pelo PWM e pelo Simscape, volta a
% elas: o Simulink acusa um laço algébrico que não afeta o resultado. Aviso silenciado.
set_param(mdl, 'AlgebraicLoopMsg', 'none');
Simulink.BlockDiagram.routeLine(find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line'));
save_system(mdl);
fprintf('Modelo gerado: %s\n', arq);
end
