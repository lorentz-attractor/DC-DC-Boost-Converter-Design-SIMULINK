function r = Boost_Interleaved_Carga(R_novos, T_fim, quais, opcoes)
% BOOST_INTERLEAVED_CARGA Degraus de carga nos modelos chaveados com PI e com backstepping.
%   Simula Boost_Interleaved_TCC_PI.slx e Boost_Interleaved_TCC_BS.slx, sem salvar os
%   modelos, com a referência constante em Vref (sem os degraus de ±100 V): partida de
%   VC_ini a Vref com a carga nominal R e, em T_carga = 20 ms, com o conversor em regime,
%   a carga passa de R para R_novos(k) (um caso por valor). É o ensaio dos degraus de
%   carga do cap. 5 do TCC (Carga_degraus_simula.m, no modelo médio) repetido no modelo
%   chaveado, que representa a condução descontínua.
%
%   Só no modelo carregado, o resistor R dá lugar a um resistor variável comandado por
%   um degrau (R antes de T_carga, R_ch depois). A lei do backstepping continua com a
%   carga nominal R (parâmetro do bloco Backstepping), como no texto. A simulação é
%   interrompida se vC chegar a 2*Vref, como no modelo médio.
%
%   Imprime, para cada caso, o desvio máximo de vC em relação a Vref e o tempo de
%   recuperação (instante, contado do degrau, a partir do qual |vC - Vref| <= 1 % de
%   Vref), sobre a média móvel de vC em um período de chaveamento, a faixa do ciclo de
%   trabalho calculado e os intervalos em que alguma fase fica sem corrente (condução
%   descontínua). Devolve as curvas e as métricas em r (um elemento por caso).
%
%   O passo máximo é reduzido de Ts/50 (o dos modelos) para Ts/200: a recuperação do PI
%   depende de uma oscilação pouco amortecida, e com Ts/50 o erro de integração a amortece
%   um pouco mais (recuperação com 60 Ohm em 6,58 ms, em vez dos 6,77 ms obtidos com
%   Ts/200, com Ts/400 e com tolerância relativa de 1e-6).
%
%   r = Boost_Interleaved_Carga                  cargas de 60 e 120 Ohm, fim em 40 ms
%   r = Boost_Interleaved_Carga(R_novos)         outras cargas após o degrau (Ohm)
%   r = Boost_Interleaved_Carga(R_novos, T_fim)  outro instante final (s)
%   r = Boost_Interleaved_Carga(R_novos, T_fim, quais)           só os controladores em quais,
%                                                                entre {'PI', 'Backstepping'}
%   r = Boost_Interleaved_Carga(R_novos, T_fim, quais, opcoes)   opcoes: pares nome/valor de
%                                                                parâmetros do modelo, no lugar
%                                                                de {'MaxStep', 'Ts/200'}
%
%   Resultados (10/11/2026, R2026b, cerca de 70 s), desvio máximo / recuperação:
%     PI,           30 -> 60 Ohm    25,1 V / 6,77 ms; fases sem corrente entre 0,14 e 0,20 ms
%     PI,           30 -> 120 Ohm   39,1 V / 2,78 ms; depois, oscilação sustentada de ±2,13 V,
%                                   limitada pela condução descontínua (igual até 120 ms)
%     Backstepping, 30 -> 60 Ohm    não se recupera: vC chega a 560 V em 0,81 ms
%     Backstepping, 30 -> 120 Ohm   não se recupera: vC chega a 560 V em 0,44 ms
%   No modelo médio (Carga_degraus_simula.m do TCC): PI com 27,5 V / 8,29 ms (60 Ohm) e
%   instável (120 Ohm); backstepping com 560 V em 0,80 e 0,44 ms.

if nargin < 1 || isempty(R_novos), R_novos = [60 120]; end
if nargin < 2 || isempty(T_fim),   T_fim = 0.04; end
if nargin < 3 || isempty(quais),   quais = {'PI', 'Backstepping'}; end
if nargin < 4, opcoes = {'MaxStep', 'Ts/200'}; end
T_carga = 0.02;                  % instante do degrau de carga (s)
faixa   = 0.01;                  % faixa de recuperação: ±1 % de Vref
pasta   = fileparts(mfilename('fullpath'));
casos = {'Boost_Interleaved_TCC_PI', 'Boost_Interleaved_PI_Parameters', 'PI'; ...
         'Boost_Interleaved_TCC_BS', 'Boost_Interleaved_BS_Parameters', 'Backstepping'};
casos = casos(ismember(casos(:,3), quais), :);
r = [];
for k = 1:size(casos, 1)
    [mdl, par, nome] = casos{k,:};
    evalin('base', [par ';']);                   % os modelos leem os parâmetros do workspace base
    p = evalin('base', 'struct(''Ts'',Ts, ''Vref'',Vref, ''VC_ini'',VC_ini, ''R'',R)');
    p.T_carga = T_carga;  p.T_fim = T_fim;  p.faixa = faixa;
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    load_system(fullfile(pasta, [mdl '.slx']));
    fecha = onCleanup(@() close_system(mdl, 0));  % nunca salva o modelo
    carga_variavel(mdl);
    for Rn = R_novos
        in = Simulink.SimulationInput(mdl);       % variáveis só desta simulação
        in = in.setVariable('A1', 0);  in = in.setVariable('A2', 0);
        in = in.setVariable('T_carga', T_carga);  in = in.setVariable('R_ch', Rn);
        in = in.setVariable('T_sim', T_fim);
        if ~isempty(opcoes), in = in.setModelParameter(opcoes{:}); end
        m = metricas(sim(in), p, nome, Rn);
        imprime(m, p, Rn == R_novos(1));
        r = [r; m]; %#ok<AGROW>
    end
    delete(fecha);
end
end

function carga_variavel(mdl)
% Troca o resistor de carga R por um resistor variável (Foundation Library) com a
% resistência dada por um degrau, de R para R_ch em T_carga, e acrescenta a parada da
% simulação quando vC chega a 2*Vref. Os blocos entram só no modelo carregado.
P = @(b) get_param([mdl '/' b], 'PortHandles');
liga = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
pos = get_param([mdl '/R'], 'Position');  ori = get_param([mdl '/R'], 'Orientation');
lh  = get_param([mdl '/R'], 'LineHandles');
delete_line([lh.LConn lh.RConn]);                 % só os trechos que chegam aos terminais de R
delete_block([mdl '/R']);
add_block('fl_lib/Electrical/Electrical Elements/Variable Resistor', [mdl '/R variável'], ...
    'Position', pos, 'Orientation', ori);
add_block('simulink/Sources/Step', [mdl '/Degrau de carga'], 'Position', pos + [-130 80 -120 60], ...
    'Time', 'T_carga', 'Before', 'R', 'After', 'R_ch');
add_block('nesl_utility/Simulink-PS Converter', [mdl '/S-PS R'], 'Position', pos + [-70 85 -70 55], ...
    'Unit', 'Ohm');
% Portas do resistor variável: LConn1 = R (sinal físico), LConn2 = +, RConn1 = -
rv = P('R variável');  cap = P('C');
liga(P('Degrau de carga').Outport(1), P('S-PS R').Inport(1));
liga(P('S-PS R').RConn(1), rv.LConn(1));
liga(cap.LConn(1), rv.LConn(2));                  % barramento
liga(cap.RConn(1), rv.RConn(1));                  % terra
% Confere se o barramento e o sensor de tensão continuam ligados como antes
pc = get_param([mdl '/Medidor vC'], 'PortConnectivity');
no = @(tipo) get_param(pc(strcmp({pc.Type}, tipo)).DstBlock, 'Name');
assert(all(ismember({'D1', 'D2', 'C', 'R variável'}, no('LConn1'))) && ...
       all(ismember({'Terra', 'C', 'R variável'}, no('RConn2'))), ...
       'A troca da carga desfez alguma ligação do barramento em %s.', mdl);
% Parada: vC medida >= 2*Vref
add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/vC no dobro'], ...
    'Position', pos + [300 80 330 60], 'relop', '>=', 'const', '2*Vref');
add_block('simulink/Sinks/Stop Simulation', [mdl '/Parada'], 'Position', pos + [380 78 395 62]);
liga(P('PS-S vC').Outport(1), P('vC no dobro').Inport(1));
liga(P('vC no dobro').Outport(1), P('Parada').Inport(1));
end

function m = metricas(out, p, nome, Rn)
% Curvas e métricas de um caso. A tensão passa por uma média móvel de um período de
% chaveamento (como em Boost_Interleaved_Exporta_Figuras) antes das métricas.
t = out.vC_sim.Time;  vC = squeeze(out.vC_sim.Data);
i1 = em(out.iL1_sim, t);  i2 = em(out.iL2_sim, t);  d = em(out.d1_sim, t);
tg = (0:p.Ts/200:t(end))';
[tu, iu] = unique(t);
vCm = movmean(interp1(tu, vC(iu), tg), 200);
m = struct('nome', nome, 'Rn', Rn, 't', t, 'vC', vC, 'i1', i1, 'i2', i2, 'd', d, 'tg', tg, 'vCm', vCm);

% Partida com a carga nominal (confere com os modelos sem alteração); a janela termina um
% período antes do degrau de carga, porque a média móvel é centrada
sel = tg >= p.Ts & tg <= p.T_carga - p.Ts;  tj = tg(sel);  yj = vCm(sel);  dA = p.Vref - p.VC_ini;
m.Mp0 = max([0; yj - p.Vref])/dA*100;
f = find(abs(yj - p.Vref) > 0.02*dA, 1, 'last');
if isempty(f), m.ts0 = 0; elseif f == numel(yj), m.ts0 = NaN; else, m.ts0 = tj(f+1)*1e3; end
m.vC0 = mean(vCm(tg >= p.T_carga - 21*p.Ts & tg < p.T_carga - p.Ts));   % média em 20 períodos antes do degrau

% Degrau de carga: desvio máximo, recuperação e parada (ms, contados do degrau)
sel = tg >= p.T_carga;  tj = tg(sel) - p.T_carga;  ej = vCm(sel) - p.Vref;
[m.desvio, kd] = max(abs(ej));  m.t_desvio = tj(kd)*1e3;  m.sinal = sign(ej(kd));
m.t_parada = NaN;  m.t_rec = NaN;
if t(end) < p.T_fim - p.Ts, m.t_parada = (t(end) - p.T_carga)*1e3; end
fora = find(abs(ej) > p.faixa*p.Vref, 1, 'last');
if isnan(m.t_parada) && isempty(fora), m.t_rec = 0;
elseif isnan(m.t_parada) && fora < numel(ej), m.t_rec = tj(fora+1)*1e3; end
s = t >= p.T_carga;
m.dmin = min(d(s));  m.dmax = max(d(s));  m.vCmin = min(vC(s));  m.vCmax = max(vC(s));
m.imin = min([i1(s); i2(s)]);

% Condução descontínua: alguma fase com corrente abaixo de 10 mA (ms, contados do degrau)
z = s & (i1 < 0.01 | i2 < 0.01);
m.dcm_ini = NaN;  m.dcm_fim = NaN;  m.dcm_total = 0;
if any(z)
    m.dcm_ini = (t(find(z, 1)) - p.T_carga)*1e3;  m.dcm_fim = (t(find(z, 1, 'last')) - p.T_carga)*1e3;
    dt = diff(t);  m.dcm_total = sum(dt(z(1:end-1)))*1e3;
end

% Regime final: média e oscilação de vC (média móvel) nos últimos 5 ms
u = tg >= t(end) - 5e-3;
m.vC_fim = mean(vCm(u));  m.osc_fim = (max(vCm(u)) - min(vCm(u)))/2;
end

function x = em(reg, t)
% Amostras do sinal registrado reg nos instantes t (os registros usam os mesmos passos)
x = squeeze(reg.Data);
if numel(x) ~= numel(t) || any(reg.Time ~= t)
    [tu, iu] = unique(reg.Time);  x = interp1(tu, x(iu), t);
end
end

function imprime(m, p, primeiro)
if primeiro
    fprintf('%s chaveado, partida (%g -> %g V, %g Ohm): Mp = %.2f %% | ts(2%%) = %.2f ms | vC médio antes do degrau = %.3f V\n', ...
        m.nome, p.VC_ini, p.Vref, p.R, m.Mp0, m.ts0, m.vC0);
end
if isnan(m.dcm_ini), dcm = 'sem condução descontínua';
else, dcm = sprintf('fase sem corrente de %.2f a %.2f ms (%.2f ms no total)', m.dcm_ini, m.dcm_fim, m.dcm_total); end
if ~isnan(m.t_parada)
    fprintf('  %g -> %g Ohm: NÃO se recupera: vC chega a %g V em %.2f ms | d em [%.3f, %.3f] | %s\n', ...
        p.R, m.Rn, 2*p.Vref, m.t_parada, m.dmin, m.dmax, dcm);
else
    if isnan(m.t_rec), rec = 'não retorna à faixa'; else, rec = sprintf('%.2f ms', m.t_rec); end
    fprintf(['  %g -> %g Ohm: desvio máximo = %.1f V (%.1f %%, %+d) em %.2f ms | recuperação (±%g %%) = %s | ' ...
        'vC em [%.1f, %.1f] V | d em [%.3f, %.3f] | %s | últimos 5 ms: média %.2f V, oscilação de ±%.2f V\n'], ...
        p.R, m.Rn, m.desvio, m.desvio/p.Vref*100, m.sinal, m.t_desvio, p.faixa*100, rec, ...
        m.vCmin, m.vCmax, m.dmin, m.dmax, dcm, m.vC_fim, m.osc_fim);
end
end
