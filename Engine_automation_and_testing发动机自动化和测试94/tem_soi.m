%% ========================================================================
% 发动机稳态工况寻优与可视化脚本 (V4 - SOI vs T_in)
% =========================================================================
%
% 目标:
% 在固定约束下，遍历SOI和T_in的组合，绘制性能MAP图，
% 并寻找热效率最高的最优参数组合。
%
%==========================================================================

% --- 1. 初始化和设置 ---
clear; clc; close all;
fprintf('开始执行稳态工况寻优与可视化 (SOI vs T_in)...\n\n');

% 首先运行包含发动机参数的脚本
initialize_engine_parameters;

% 加载Simulink模型到内存中
load_system('engine_modelQ4');

% --- 2. 设定固定参数、寻优目标与搜索范围 ---

% --- 修改部分: 设定新的固定参数 ---
P.RPM = 600; % 固定转速 (rev/min)
P.P_in = 1.66e5; % <-- 固定进气压力为166 kPa
PHI_target = 0.8; % <-- 固定目标当量比
fprintf('固定参数: 转速=%d RPM, 进气压力=%.1f kPa, 当量比 Φ=%.1f\n', P.RPM, P.P_in/1000, PHI_target);

% --- 设定目标扭矩范围 ---
T_max = 450;
target_torques = linspace(T_max * 0.9, T_max * 1.0, 2);
torque_tolerance = 15;

% --- 初始化最终结果存储表格 ---
optimal_results = table();

% --- 3. 外层循环: 遍历每一个目标扭矩 ---
for t_idx = 1:length(target_torques)

current_target_torque = target_torques(t_idx);
fprintf('\n-----------------------------------------------------------------\n');
fprintf('开始为目标扭矩 %.1f N·m 进行寻优...\n', current_target_torque);
fprintf('-----------------------------------------------------------------\n');

% --- 修改部分: 设定新的搜索范围 ---
soi_range = linspace(0, 35, 10); % 喷油提前角搜索范围 (X轴)
t_in_range = linspace(350, 450, 2); % 进气温度搜索范围 (Y轴)

% --- 初始化用于存储当前扭矩目标下所有结果的矩阵 ---
torque_map = zeros(length(t_in_range), length(soi_range));
efficiency_map = zeros(length(t_in_range), length(soi_range));

total_sims = length(soi_range) * length(t_in_range);
sim_count = 0;
fprintf('将在 %d 个参数组合中进行搜索，请耐心等待...\n\n', total_sims);

% --- 内层双重嵌套循环，遍历(SOI, T_in)参数组合 ---
% 修改部分: 外层循环遍历进气温度
for j = 1:length(t_in_range)
% 修改部分: 内层循环遍历喷油时刻
for i = 1:length(soi_range)
sim_count = sim_count + 1;

% 设置当前循环的参数
current_t_in = t_in_range(j);
current_soi = soi_range(i)-10.8;

set_param('engine_modelQ4/SOI_Timing', 'Value', num2str(current_soi));

% 核心计算：根据约束确定喷油量
% 注意：这里的m_air会随变化的T_in而改变
m_air = (P.P_in * (2.6e-3)) / (P.R_air * current_t_in);
P.m_fuel = (PHI_target / P.AFR_stoich) * m_air;

P.T_in = current_t_in; % 设置当前的进气温度

% 运行仿真
fprintf('运行 %d/%d: T_in=%.1fK, SOI=%.1fdeg -> m_fuel=%.2emg\n',...
sim_count, total_sims, current_t_in, current_soi, P.m_fuel*1e6);
sim_output = sim('engine_modelQ4');

% 计算性能并记录
V_cycle = sim_output.V_out; P_cycle = sim_output.P_out;
angle_difference = abs(V_cycle - min(V_cycle)); [~, idx_tdc] = min(angle_difference);
if idx_tdc > 1; V_comp = V_cycle(1:idx_tdc); P_comp = P_cycle(1:idx_tdc); else; V_comp = V_cycle(1); P_comp = P_cycle(1); end
V_exp = V_cycle(idx_tdc:end); P_exp = P_cycle(idx_tdc:end);
V_common = linspace(min(V_cycle), max(V_cycle), 1000);
[V_comp_unique, u_idx_c] = unique(V_comp, 'stable'); P_comp_unique = P_comp(u_idx_c);
[V_exp_unique, u_idx_e] = unique(V_exp, 'stable'); P_exp_unique = P_exp(u_idx_e);
P_lower_interp = interp1(V_comp_unique, P_comp_unique, V_common, 'pchip');
P_upper_interp = interp1(V_exp_unique, P_exp_unique, V_common, 'pchip');
delta_P = P_upper_interp - P_lower_interp;
W_net = trapz(V_common, delta_P);

actual_torque = W_net / (4 * pi);
Q_in = P.m_fuel * P.LHV;
if Q_in > 0; actual_efficiency = W_net / Q_in; else; actual_efficiency = 0; end

% 存储到表格
torque_map(j, i) = actual_torque;
efficiency_map(j, i) = actual_efficiency * 100; % 存储为百分比
end
end

% --- 4. 对当前目标扭矩的寻优结果进行分析和可视化 ---
qualified_indices = find(abs(torque_map - current_target_torque) <= torque_tolerance);
if isempty(qualified_indices)
fprintf('警告: 未能找到扭矩满足要求的工况点，请尝试调整搜索范围。\n');
continue;
end
[max_eff, idx_in_qualified] = max(efficiency_map(qualified_indices));
optimal_idx = qualified_indices(idx_in_qualified);
[best_j, best_i] = ind2sub(size(torque_map), optimal_idx);

% 获取最优参数
optimal_soi = soi_range(best_i);
optimal_tin = t_in_range(best_j);

% --- 绘制可视化MAP图 ---
figure('Name', sprintf('寻优过程: 目标扭矩 = %.1f N·m', current_target_torque));
% 修改部分: [X, Y]是坐标轴网格，X是喷油时刻，Y是进气温度
[X, Y] = meshgrid(soi_range, t_in_range);
Z_eff = efficiency_map;
Z_tor = torque_map;

min_eff=min(Z_eff(:)); max_eff=max(Z_eff(:))+0.001;

contourf(X, Y, Z_eff, 20);
hold on;
clim([min_eff max_eff]);
contour(X, Y, Z_tor, 'k--', 'ShowText', 'on');
contour(X, Y, Z_tor, [current_target_torque, current_target_torque], 'r-', 'LineWidth', 3);

% 修改部分: 在图上标记最优的SOI和T_in
% plot(optimal_soi, optimal_tin, 'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'r');

hold off;
colorbar; title(sprintf('目标扭矩 %.1f N·m 的最优参数寻优图, %.1f RPM', current_target_torque ,P.RPM));
xlabel('喷油提前角 (度 BTDC)'); ylabel('进气温度 (K)'); % <-- 修改 Y轴标签
c = colorbar; c.Label.String = '热效率 (%)';

% --- 记录最终结果 ---
% 修改部分: 记录的参数变为SOI和T_in
temp_table = table(current_target_torque, optimal_soi, optimal_tin, max_eff, ...
'VariableNames', {'Target_Torque', 'Optimal_SOI', 'Optimal_Tin_K', 'Max_Efficiency_pct'});
optimal_results = [optimal_results; temp_table];
end

% --- 5. 总结报告 ---
fprintf('\n\n======================== 最终寻优总结报告 ========================\n');
disp('在各个目标扭矩下，热效率最高对应的最优参数如下:');
disp(optimal_results);
fprintf('====================================================================\n');