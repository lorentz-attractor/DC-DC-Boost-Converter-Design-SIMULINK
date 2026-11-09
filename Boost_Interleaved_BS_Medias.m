function r = Boost_Interleaved_BS_Medias(quais, retida)
% BOOST_INTERLEAVED_BS_MEDIAS Backstepping chaveado com as medições médias em um período.
%   Simula Boost_Interleaved_TCC_BS.slx duas vezes, sem salvar o modelo: como está
%   (medições instantâneas, com a ondulação de chaveamento) e com vC, iL1 e iL2
%   passando, antes da lei de controle, pela média móvel exata de um período Ts.
%   É o experimento citado no fim da seção do modelo chaveado (cap. 5 do TCC): se as
%   médias levam o resultado de volta ao do modelo médio, a diferença vem da ondulação.
%   Imprime Mp e ts de cada trecho (critério de 2 % do degrau, sobre a média móvel de
%   vC, como em Boost_Interleaved_Exporta_Figuras) e a faixa do ciclo de trabalho, e
%   devolve os valores em r.instantaneas e r.medias.
%
%   A média de x em [t - Ts, t] é x0 + [Y(t) - Y(t - Ts)]/Ts, com Y = int(x - x0) e x0
%   o valor da pré-carga. Antes de t = 0 o conversor está parado na pré-carga (x = x0,
%   logo Y = 0), e o atraso de transporte parte de 0: a média é exata também no
%   primeiro período. Os blocos entram só no modelo carregado, fechado sem salvar.
%
%   r = Boost_Interleaved_BS_Medias               média móvel de vC, iL1 e iL2 (o experimento do texto)
%   r = Boost_Interleaved_BS_Medias(quais)        média só dos sinais em quais, entre {'vC','iL1','iL2'}
%   r = Boost_Interleaved_BS_Medias(quais, true)  média do período anterior, amostrada em kTs e
%                                                 retida por um período (como num controle digital)
%
%   Resultados (09/11/2026, R2026b, cerca de 20 s por chamada), Mp / ts da partida 250 -> 280 V:
%     medições instantâneas                 3,55 % / 5,93 ms   (subida 0,34 % / 2,90 ms; descida 1,02 % / 3,40 ms)
%     média móvel de vC, iL1 e iL2          0,01 % / 3,52 ms   (subida 0,22 % / 3,24 ms; descida 0,01 % / 3,39 ms)
%     média móvel só de iL1 e iL2           0,02 % / 3,56 ms
%     média móvel só de vC                  3,81 % / 5,85 ms
%     média retida de vC, iL1 e iL2         0,00 % / 3,58 ms   (subida 0,25 % / 3,16 ms; descida 0,02 % / 3,32 ms)
%   No modelo médio (BS_degraus_simula.m do TCC): 0,02 % / 3,42 ms.

if nargin < 1 || isempty(quais), quais = {'vC', 'iL1', 'iL2'}; end
if nargin < 2, retida = false; end
pasta = fileparts(mfilename('fullpath'));
mdl   = 'Boost_Interleaved_TCC_BS';
evalin('base', 'Boost_Interleaved_BS_Parameters;');   % o modelo lê os parâmetros do workspace base
p = evalin('base', ['struct(''Ts'',Ts, ''Vref'',Vref, ''A1'',A1, ''A2'',A2, ''VC_ini'',VC_ini, ' ...
    '''IL_ini'',IL_ini, ''T_deg1'',T_deg1, ''T_deg2'',T_deg2, ''T_sim'',T_sim)']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(fullfile(pasta, [mdl '.slx']));
fecha = onCleanup(@() close_system(mdl, 0));          % nunca salva o modelo

r.instantaneas = simula(mdl, p, 'Medições instantâneas (modelo como está)');
insere_medias(mdl, quais, retida);
tipo = {'Média móvel', 'Média retida'};
r.medias = simula(mdl, p, [tipo{retida + 1} ' em um período Ts de ' strjoin(quais, ', ')]);
end

function m = simula(mdl, p, nome)
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
m = metricas(out, p);
fprintf('%s:\n', nome);
for w = 1:numel(m)
    fprintf('  %-8s: Mp = %.2f %% | ts(2%%) = %.2f ms | d em [%.4f, %.4f]\n', ...
        m(w).trecho, m(w).Mp, m(w).ts, m(w).dmin, m(w).dmax);
end
end

function insere_medias(mdl, quais, retida)
% Troca as ligações PS-S -> Backstepping (entradas 1 a 3: vC, iL1, iL2) dos sinais em
% quais por médias móveis de um período. As demais ramificações dos sensores (registro e scopes)
% continuam com os valores instantâneos.
sinais = {'vC', 1, 'VC_ini'; 'iL1', 2, 'IL_ini'; 'iL2', 3, 'IL_ini'};
sinais = sinais(ismember(sinais(:,1), quais), :);
P = @(b) get_param([mdl '/' b], 'PortHandles');
liga = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
for k = 1:size(sinais, 1)
    [nome, porta, x0] = sinais{k,:};
    delete_line(mdl, ['PS-S ' nome '/1'], ['Backstepping/' num2str(porta)]);
    y0 = 700 + 120*k;                                 % posição dos blocos novos (só visual)
    b = @(s) ['Média ' nome ' ' s];
    add_block('simulink/Math Operations/Bias', [mdl '/' b('desvio')], ...
        'Position', [-900 y0 -870 y0+30], 'Bias', ['-' x0]);
    add_block('simulink/Continuous/Integrator', [mdl '/' b('integral')], ...
        'Position', [-840 y0 -810 y0+30], 'InitialCondition', '0');
    add_block('simulink/Continuous/Transport Delay', [mdl '/' b('atraso')], ...
        'Position', [-780 y0+50 -740 y0+80], 'DelayTime', 'Ts', 'InitialOutput', '0', ...
        'BufferSize', '16384');
    add_block('simulink/Math Operations/Sum', [mdl '/' b('janela')], ...
        'Position', [-700 y0 -680 y0+30], 'Inputs', '+-', 'IconShape', 'rectangular');
    add_block('simulink/Math Operations/Gain', [mdl '/' b('por Ts')], ...
        'Position', [-650 y0 -620 y0+30], 'Gain', '1/Ts');
    add_block('simulink/Math Operations/Bias', [mdl '/' b('média')], ...
        'Position', [-590 y0 -560 y0+30], 'Bias', x0);
    liga(P(['PS-S ' nome]).Outport(1), P(b('desvio')).Inport(1));
    liga(P(b('desvio')).Outport(1),   P(b('integral')).Inport(1));
    liga(P(b('integral')).Outport(1), P(b('janela')).Inport(1));
    liga(P(b('integral')).Outport(1), P(b('atraso')).Inport(1));
    liga(P(b('atraso')).Outport(1),   P(b('janela')).Inport(2));
    if retida   % média do período anterior, amostrada em kTs e retida por um período
        add_block('simulink/Discrete/Zero-Order Hold', [mdl '/' b('retenção')], ...
            'Position', [-675 y0+50 -645 y0+80], 'SampleTime', 'Ts');
        liga(P(b('janela')).Outport(1),   P(b('retenção')).Inport(1));
        liga(P(b('retenção')).Outport(1), P(b('por Ts')).Inport(1));
    else
        liga(P(b('janela')).Outport(1),   P(b('por Ts')).Inport(1));
    end
    liga(P(b('por Ts')).Outport(1),   P(b('média')).Inport(1));
    liga(P(b('média')).Outport(1),    P('Backstepping').Inport(porta));
end
end

function m = metricas(out, p)
% Mp e ts de cada trecho sobre a média móvel de vC em um período (o primeiro período
% após cada degrau é descartado), e a faixa do ciclo de trabalho calculado
tg = (0:p.Ts/200:p.T_sim)';
[tu, iu] = unique(out.vC_sim.Time);  y = squeeze(out.vC_sim.Data);
ym = movmean(interp1(tu, y(iu), tg), 200);
[td, id] = unique(out.d1_sim.Time);  d = squeeze(out.d1_sim.Data);  d = d(id);
r1 = p.Vref + p.A1;  r2 = r1 - p.A2;
J = {0, p.T_deg1, p.Vref, p.VC_ini, 'Partida'; p.T_deg1, p.T_deg2, r1, p.Vref, 'Subida'; ...
     p.T_deg2, p.T_sim, r2, r1, 'Descida'};
m = struct('trecho', J(:,5), 'Mp', [], 'ts', [], 'dmin', [], 'dmax', []);
for w = 1:size(J, 1)
    sel = tg >= J{w,1} + p.Ts & tg <= J{w,2};  tj = tg(sel) - J{w,1};  yj = ym(sel);
    dA = J{w,3} - J{w,4};
    m(w).Mp = max([0; sign(dA)*(yj - J{w,3})])/abs(dA)*100;
    f = find(abs(yj - J{w,3}) > 0.02*abs(dA), 1, 'last');
    if isempty(f), m(w).ts = 0; elseif f == numel(yj), m(w).ts = NaN; else, m(w).ts = tj(f+1)*1e3; end
    sd = td >= J{w,1} & td < J{w,2};
    m(w).dmin = min(d(sd));  m(w).dmax = max(d(sd));
end
end
