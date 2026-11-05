function Boost_Interleaved_Exporta_Figuras(pasta)
% BOOST_INTERLEAVED_EXPORTA_FIGURAS Simula os modelos chaveados com PI e com backstepping
% e exporta as figuras dos degraus de ±100 V (T_deg1 a T_sim; PNG, 300 dpi) para a pasta
% indicada, além de imprimir as métricas de cada trecho. Os modelos são fechados sem salvar.
%   Boost_Interleaved_Exporta_Figuras(pasta)
%
% Métricas: vC passa por uma média móvel de um período de chaveamento antes do critério
% de 2 % de cada degrau (o mesmo dos modelos médios do TCC).

if ~exist(pasta, 'dir'), mkdir(pasta); end
casos = {'Boost_Interleaved_TCC_PI', 'Boost_Interleaved_PI_Parameters', 'pi_degraus_chaveado.png', 'PI'; ...
         'Boost_Interleaved_TCC_BS', 'Boost_Interleaved_BS_Parameters', 'bs_degraus_chaveado.png', 'Backstepping'};
for k = 1:size(casos,1)
    [mdl, par, arq, nome] = casos{k,:};
    evalin('base', [par ';']);                   % os modelos leem os parâmetros do workspace base
    p = evalin('base', 'struct(''Ts'',Ts, ''Vref'',Vref, ''A1'',A1, ''A2'',A2, ''VC_ini'',VC_ini, ''D0'',D0, ''T_deg1'',T_deg1, ''T_deg2'',T_deg2, ''T_sim'',T_sim)');
    load_system(mdl);
    out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    close_system(mdl, 0);
    vC = out.vC_sim;  i1 = out.iL1_sim;  i2 = out.iL2_sim;  d = out.d1_sim;  ref = out.vref_sim;

    %% Métricas por trecho, sobre a média móvel de um período
    tg = (0:p.Ts/200:p.T_sim)';
    [tu, iu] = unique(vC.Time);  y = squeeze(vC.Data);
    ym = movmean(interp1(tu, y(iu), tg), 200);
    r1 = p.Vref + p.A1;  r2 = r1 - p.A2;
    J = {0, p.T_deg1, p.Vref, p.VC_ini, 'Partida'; p.T_deg1, p.T_deg2, r1, p.Vref, 'Subida'; ...
         p.T_deg2, p.T_sim, r2, r1, 'Descida'};
    fprintf('%s chaveado:\n', nome);
    for w = 1:3
        m = tg >= J{w,1} + p.Ts & tg <= J{w,2};  tj = tg(m) - J{w,1};  yj = ym(m);  dA = J{w,3} - J{w,4};
        Mp = max([0; sign(dA)*(yj - J{w,3})])/abs(dA)*100;
        f = find(abs(yj - J{w,3}) > 0.02*abs(dA), 1, 'last');
        if isempty(f), ts = 0; elseif f == numel(yj), ts = NaN; else, ts = tj(f+1)*1e3; end
        fprintf('  %-8s: Mp = %.2f %% | ts(2%%) = %.2f ms\n', J{w,5}, Mp, ts);
    end

    %% Figura: vC, correntes de fase e ciclo de trabalho na janela dos degraus
    fig = figure('Theme','light', 'Color','w', 'Position',[100 100 760 700]);
    ax = gobjects(3,1);
    ax(1) = subplot(3,1,1); plot(vC.Time*1e3, y, ref.Time*1e3, squeeze(ref.Data), 'k:'); grid on;
    ylabel('v_C (V)'); legend('Modelo chaveado', 'Referência', 'Location','southeast');
    ax(2) = subplot(3,1,2); plot(i1.Time*1e3, squeeze(i1.Data), i2.Time*1e3, squeeze(i2.Data)); grid on;
    ylabel('i_L (A)'); legend('i_{L1}', 'i_{L2}', 'Location','southeast');
    ax(3) = subplot(3,1,3); plot(d.Time*1e3, squeeze(d.Data)); grid on; yline(p.D0, 'k:');
    ylabel('d'); xlabel('t (ms)');
    linkaxes(ax, 'x'); xlim([p.T_deg1 p.T_sim]*1e3);
    ylim(ax(1), [r2 - 20, r1 + 20]);
    exportgraphics(fig, fullfile(pasta, arq), 'Resolution', 300);
    close(fig);
end
fprintf('Figuras salvas na pasta %s.\n', pasta);
end
