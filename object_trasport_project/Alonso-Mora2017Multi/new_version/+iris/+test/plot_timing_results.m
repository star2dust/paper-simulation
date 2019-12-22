function plot_timing_results(all_results, fname)

if nargin < 2
  fname = 'timing.dat'
end

n_trials = 10;
n_obs = reshape([all_results.n_obs], n_trials, []);
iters = reshape([all_results.iters], n_trials, []);
total_time = reshape([all_results.total_time], n_trials, []);
e_time = reshape([all_results.e_time], n_trials, []);
p_time = reshape([all_results.p_time], n_trials, []);
linear_ref = mean(n_obs) * (mean(total_time(:,end)) / mean(n_obs(:,end)));
T = table(mean(n_obs)', std(n_obs)',...
  mean(iters)', std(iters)', ...
  mean(total_time)', std(total_time)',...
  mean(e_time)', std(e_time)',...
  mean(p_time)', std(p_time)',...
  linear_ref',...
  'VariableNames', {'n_obs', 'n_obs_std',...
                    'iters', 'iters_std',...
                    'total_time', 'total_time_std',...
                    'e_time', 'e_time_std',...
                    'p_time', 'p_time_std',...
                    'linear_ref'});
writetable(T, fname, 'Delimiter', '\t');
figure(6)
set(gcf, 'Position', [100, 100, 500, 700])
set(gcf, 'defaulttextinterpreter','latex');
clf
subplot(3,1,[1,2])
cla
hold on
opts = {'LineWidth', 2};
errorbar(mean(n_obs), mean(total_time), std(total_time), 'k.-', opts{:})
errorbar(mean(n_obs), mean(e_time), std(e_time), '.-', 'Color', [.8,.1,.1], opts{:})
errorbar(mean(n_obs), mean(p_time), std(p_time), '.-', 'Color', [.1,.8,.1], opts{:})
set(gca, 'XScale', 'log')
set(gca, 'YScale', 'log')
% loglog([1, mean(n_obs(:,end))], [mean(total_time(:,end))/mean(n_obs(:,end)), mean(total_time(:,end))], 'k--')
loglog(mean(n_obs), linear_ref, 'k--')
legend({'Total Time', 'Time Computing Ellipse', 'Time Computing Planes', '$\frac{1}{2}$'}, 'Location', 'NorthWest', 'FontSize', 12, 'Interpreter', 'LaTeX');
xlim([min([all_results.n_obs])-1, max([all_results.n_obs])*1.5]);
xlabel('Number of obstacles','FontSize', 12)
ylabel('CPU Time (s)','FontSize', 12)
set(gca, 'FontSize', 12);
set(gca, 'defaulttextinterpreter','latex');
subplot(3,1,3)
errorbar(mean(n_obs), mean(iters), std(iters), 'ko', opts{:})
set(gca, 'XScale', 'log');
ylim([0, max(mean(iters) + std(iters)) + 1])
xlim([min([all_results.n_obs])-1, max([all_results.n_obs])*1.5]);
xlabel('Number of obstacles','FontSize', 12)
ylabel('Number of major iterations','FontSize', 12)
% h = bar([all_results.n_obs], [all_results.iters]);
% set(h, 'BarWidth', 0.8);
set(gca, 'FontSize', 12);
set(gca, 'defaulttextinterpreter','latex');
set(gcf, 'PaperPositionMode', 'auto')
print(gcf, 'fig/last_timing_results', '-dpdf')

n_in_poly = 0;
n_outside_poly = 0;
for j = 1:size(all_results,1)
  for k = 1:size(all_results,2)
    hist = all_results(j,k).p_history{end};
    if all(hist.A * all_results(j,k).start <= hist.b)
      n_in_poly = n_in_poly + 1;
    else
      n_outside_poly = n_outside_poly + 1;
    end
  end
end
n_in_poly
n_outside_poly
n_in_poly / (n_in_poly + n_outside_poly)
