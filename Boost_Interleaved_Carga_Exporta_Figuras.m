function Boost_Interleaved_Carga_Exporta_Figuras(pasta, r)
% BOOST_INTERLEAVED_CARGA_EXPORTA_FIGURAS Simula os degraus de carga nos modelos chaveados
% com PI e com backstepping (Boost_Interleaved_Carga, cargas de 60 e 120 Ohm) e exporta a
% figura do cap. 5 do TCC (PNG, 300 dpi) para a pasta indicada: a tensão no barramento dos
% dois controladores em cada degrau e as correntes de fase com o PI no degrau para a
% carga mais leve, em que a condução fica descontínua.
%   Boost_Interleaved_Carga_Exporta_Figuras(pasta)
%   Boost_Interleaved_Carga_Exporta_Figuras(pasta, r)   reaproveita o resultado r de Boost_Interleaved_Carga

if ~exist(pasta, 'dir'), mkdir(pasta); end
if nargin < 2, r = Boost_Interleaved_Carga; end
Vref = 280;  R = 30;  T_carga = 0.02;  faixa = 0.01;      % como em Boost_Interleaved_Carga
R_novos = unique([r.Rn], 'stable');
janela  = T_carga*1e3 + [-0.5 12];                         % como em carga_degraus_medio (ms)

% Com as margens compactas, 684 px de largura dão ao PNG a mesma largura (e o mesmo tamanho
% de fonte no texto) das figuras de 760 px feitas com subplot, como carga_degraus_medio
f1 = figure('Theme','light', 'Color','w', 'Position',[100 100 684 480]);
cores = colororder(f1);  cor = cores([1 3],:);  estilo = {'-', '-.'};   % como em carga_degraus_medio
n = numel(R_novos);
tiledlayout(f1, n + 1, 1, 'TileSpacing','compact', 'Padding','compact');
for k = 1:n
    nexttile; hold on; grid on;
    rk = r([r.Rn] == R_novos(k));
    h = gobjects(numel(rk), 1);
    for m = numel(rk):-1:1                                 % o PI por cima do backstepping
        h(m) = plot(rk(m).t*1e3, rk(m).vC, estilo{m}, 'Color', cor(m,:), 'LineWidth', 1.2);
    end
    yline(Vref*(1 + [-1 1]*faixa), 'k:', 'HandleVisibility','off');
    xlim(janela);  ylim(Vref + [-60 60]);
    ylabel('v_C (V)');  title(sprintf('Carga: %g \\Omega \\rightarrow %g \\Omega', R, R_novos(k)));
    if k == 1, legend(h, 'PI', 'Backstepping completo', 'Location','northeast'); end
end
% Correntes de fase com o PI no último degrau
pik = r(strcmp({r.nome}, 'PI') & [r.Rn] == R_novos(end));
nexttile; hold on; grid on;
plot(pik.t*1e3, pik.i1, pik.t*1e3, pik.i2);
xlim(janela);  ylim([0 7]);
ylabel('i_L (A)');  xlabel('t (ms)');
title(sprintf('Correntes de fase com o PI: %g \\Omega \\rightarrow %g \\Omega', R, R_novos(end)));
legend('i_{L1}', 'i_{L2}', 'Location','northeast');
exportgraphics(f1, fullfile(pasta, 'carga_degraus_chaveado.png'), 'Resolution', 300);
close(f1);
fprintf('Figura salva na pasta %s.\n', pasta);
end
