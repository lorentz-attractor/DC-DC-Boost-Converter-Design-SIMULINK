function Boost_Interleaved_PI_Build(sobrescrever)
% BOOST_INTERLEAVED_PI_BUILD Gera Boost_Interleaved_TCC_PI.slx a partir da malha aberta.
%   Carrega Boost_Interleaved_TCC.slx (que não é alterado) e salva uma cópia em
%   malha fechada: as constantes d1 e d2 dão lugar ao PI do TCC, uma função de
%   transferência (Kp_pi*s + Ki_pi)/s, como no modelo usado no projeto (PI_sim).
%   Erro = referência - vC medida; a mesma saída d comanda as duas fases.
%   Os valores ficam em Boost_Interleaved_PI_Parameters.m.
%
%   Boost_Interleaved_PI_Build        gera o modelo; recusa se o .slx já existir
%   Boost_Interleaved_PI_Build(true)  gera e sobrescreve o .slx existente

if nargin < 1, sobrescrever = false; end
pasta = fileparts(mfilename('fullpath'));
base  = 'Boost_Interleaved_TCC';
mdl   = 'Boost_Interleaved_TCC_PI';
arq   = fullfile(pasta, [mdl '.slx']);
if isfile(arq) && ~sobrescrever
    error('O arquivo %s já existe. Use Boost_Interleaved_PI_Build(true) para sobrescrever.', arq);
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
add_block('simulink/Sources/Constant', [mdl '/Vref'], 'Position', [-560 370 -510 390], 'Value', 'Vref');
add_block('simulink/Sources/Step', [mdl '/Degrau +'], 'Position', [-560 415 -530 445], ...
    'Time', 'T_deg1', 'Before', '0', 'After', 'A1');
add_block('simulink/Sources/Step', [mdl '/Degrau -'], 'Position', [-560 465 -530 495], ...
    'Time', 'T_deg2', 'Before', '0', 'After', 'A2');
add_block('simulink/Math Operations/Sum', [mdl '/Referência'], 'Position', [-470 365 -445 500], ...
    'Inputs', '++-', 'IconShape', 'rectangular');

%% Malha de tensão: erro -> PI (função de transferência) -> d1 = d2 = d
add_block('simulink/Math Operations/Sum', [mdl '/Erro'], 'Position', [-390 420 -370 440], ...
    'Inputs', '|+-', 'IconShape', 'round');
add_block('simulink/Continuous/Transfer Fcn', [mdl '/PI'], 'Position', [-330 405 -220 455], ...
    'Numerator', '[Kp_pi Ki_pi]', 'Denominator', '[1 0]');
add_block('simulink/Sinks/To Workspace', [mdl '/vref_sim'], 'Position', [-400 300 -330 320], ...
    'VariableName', 'vref_sim', 'SaveFormat', 'Timeseries');

liga(P('Vref').Outport(1),       P('Referência').Inport(1));
liga(P('Degrau +').Outport(1),   P('Referência').Inport(2));
liga(P('Degrau -').Outport(1),   P('Referência').Inport(3));
liga(P('Referência').Outport(1), P('Erro').Inport(1));
liga(P('PS-S vC').Outport(1),    P('Erro').Inport(2));       % realimentação de vC
liga(P('Erro').Outport(1),       P('PI').Inport(1));
liga(P('Referência').Outport(1), P('vref_sim').Inport(1));
for k = 1:2
    n = num2str(k);
    liga(P('PI').Outport(1), P(['Compara ' n]).Inport(1));
    liga(P('PI').Outport(1), P(['d' n '_sim']).Inport(1));
    liga(P('PI').Outport(1), P('Ciclos').Inport(k));
end

%% Scopes: referência junto de vC; faixa de d para os degraus
set_param([mdl '/Tensão'], 'NumInputPorts', '2');
liga(P('Referência').Outport(1), P('Tensão').Inport(2));
nomeia = @(ph, nome) set_param(get_param(ph, 'Line'), 'Name', nome);
nomeia(P('PI').Outport(1), 'd');
nomeia(P('Referência').Outport(1), 'vref');
cfg = get_param([mdl '/Ciclos'], 'ScopeConfiguration');
cfg.YLimits = [0 0.5];

%% Simulação
% O termo proporcional do PI passa direto ao comparador do PWM e, pelo Simscape, volta
% em vC: o Simulink acusa um laço algébrico que não afeta o resultado. Aviso silenciado.
set_param(mdl, 'AlgebraicLoopMsg', 'none');
Simulink.BlockDiagram.routeLine(find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line'));
save_system(mdl);
fprintf('Modelo gerado: %s\n', arq);
end
