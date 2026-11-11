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
    % e, na última linha, o detalhe de quatro períodos em regime, em r1 (antes da descida)
    % e em r2 (no fim da simulação), com as correntes de fase e o ciclo de trabalho.
    % Tamanho fixo no papel: não depende da tela, e a largura (6,7 in a 300 dpi = 2010 px) é a das
    % demais figuras do TCC, para manter o mesmo tamanho de fonte no documento.
    fig = figure('Theme','light', 'Color','w', 'Units','inches', 'Position',[0.5 0.5 6.7 8.5]);
    set(fig, 'PaperUnits','inches', 'PaperPositionMode','manual', 'PaperPosition',[0 0 6.7 8.5], 'PaperSize',[6.7 8.5]);
    tl = tiledlayout(fig, 4, 2, 'TileSpacing','compact', 'Padding','compact');
    ax = gobjects(3,1);
    ax(1) = nexttile(tl, [1 2]); plot(vC.Time*1e3, y, ref.Time*1e3, squeeze(ref.Data), 'k:'); grid on;
    ylabel('v_C (V)'); legend('Modelo chaveado', 'Referência', 'Location','southeast');
    ax(2) = nexttile(tl, [1 2]); plot(i1.Time*1e3, squeeze(i1.Data), i2.Time*1e3, squeeze(i2.Data)); grid on;
    ylabel('i_L (A)'); legend('i_{L1}', 'i_{L2}', 'Location','southeast');
    ax(3) = nexttile(tl, [1 2]); plot(d.Time*1e3, squeeze(d.Data)); grid on; yline(p.D0, 'k:');
    ylabel('d'); xlabel('t (ms)');
    linkaxes(ax, 'x'); xlim(ax(1), [p.T_deg1 p.T_sim]*1e3);
    ylim(ax(1), [r2 - 20, r1 + 20]);
    jan = {p.T_deg2 - 0.25e-3, r1; p.T_sim - 0.25e-3, r2};     % início da janela e tensão
    for w = 1:2
        t0 = jan{w,1};  tf = t0 + 4*p.Ts;
        az = nexttile(tl);
        m1 = i1.Time >= t0 & i1.Time <= tf;  m2 = i2.Time >= t0 & i2.Time <= tf;  md = d.Time >= t0 & d.Time <= tf;
        yyaxis(az, 'left');
        co = colororder(fig);                                  % as cores das correntes nas linhas de cima
        plot(az, i1.Time(m1)*1e3, squeeze(i1.Data(m1)), '-', 'Color', co(1,:)); hold(az, 'on');
        plot(az, i2.Time(m2)*1e3, squeeze(i2.Data(m2)), '-', 'Color', co(2,:)); hold(az, 'off');
        ylabel(az, 'i_L (A)');
        yyaxis(az, 'right');
        dd = squeeze(d.Data(md));
        plot(az, d.Time(md)*1e3, dd, 'k-');
        ylim(az, mean(dd) + [-0.06 0.06]);                    % mesma escala nos dois controladores
        ylabel(az, 'd');
        az.YAxis(1).Color = [0.15 0.15 0.15];  az.YAxis(2).Color = [0.15 0.15 0.15];
        grid(az, 'on'); xlim(az, [t0 tf]*1e3); xlabel(az, 't (ms)');
        title(az, sprintf('Detalhe em %d V', round(jan{w,2})), 'FontWeight','normal');
        if w == 2, lg = legend(az, 'i_{L1}', 'i_{L2}', 'd', 'Orientation','horizontal'); lg.Layout.Tile = 'south'; end
    end
    print(fig, fullfile(pasta, arq), '-dpng', '-r300');
    close(fig);
end
fprintf('Figuras salvas na pasta %s.\n', pasta);
end
