%% ========================================================================
% 发动机性能二维MAP图绘制与寻优脚本 (V4 - T_in vs RPM)
% =========================================================================
%
% 目标:
% 在固定约束下，遍历转速(RPM)和进气温度(T_in)的组合，
% 绘制以“热效率”为背景色，“扭矩”为等高线的二维MAP图。
%
%==========================================================================

% --- 1. 初始化和设置 ---
clear; clc; close all;
fprintf('开始执行稳态工况寻优与可视化 (T_in vs RPM)...\n\n');

% 首先运行包含发动机参数的脚本
initialize_engine_parameters;

% 加载Simulink模型到内存中
load_system('engine_modelQ4');

% --- 2. 设定固定参数与搜索范围 ---

% --- 修改部分: 设定新的固定参数 ---
P.P_in = 1.66e5; % 固定进气压力为166 kPa
P.SOI_deg_BTDC = 10; % 固定喷油提前角为10度
PHI_target = 0.8; % 固定目标当量比

fprintf('固定参数: 进气压力 = %.1f kPa, 当量比 Φ=%.1f, SOI=%d deg\n', P.P_in/1000, PHI_target, P.SOI_deg_BTDC);
set_param('engine_model/SOI_Timing', 'Value', num2str(P.SOI_deg_BTDC));

% --- 修改部分: 设定新的搜索范围 ---
rpm_range = linspace(360, 1800, 11); % 转速搜索范围 (RPM), 分11步
t_in_range = linspace(350, 450, 7); % 进气温度搜索范围 (K), 分10步

total_sims = length(rpm_range) * length(t_in_range);
fprintf('将在 %d 个参数组合中进行搜索，请耐心等待...\n\n', total_sims);

% --- 3. 初始化用于存储所有结果的矩阵 ---
% 修改部分: 矩阵的维度现在由t_in和rpm的长度决定
torque_map = zeros(length(t_in_range), length(rpm_range));
efficiency_map = zeros(length(t_in_range), length(rpm_range));
sim_count = 0;

% --- 4. 双重嵌套循环，遍历参数组合 ---
% 修改部分: 外层循环遍历进气温度 (Y轴)
for j = 1:length(t_in_range)
% 修改部分: 内层循环遍历转速 (X轴)
for i = 1:length(rpm_range)
sim_count = sim_count + 1;

% 设置当前循环的参数
current_t_in = t_in_range(j);
fprintf('kkkkkk%d \n',current_t_in);
current_rpm = rpm_range(i);

% 核心计算：根据约束条件确定喷油量
V_max_approx = 2.6e-3;
m_air = (P.P_in * V_max_approx) / (P.R_air * current_t_in);
m_fuel = (PHI_target / P.AFR_stoich) * m_air;

P.T_in = current_t_in;
P.RPM = current_rpm;
P.m_fuel = m_fuel;

% 运行仿真
fprintf('运行 %d/%d: RPM=%.0f, T_in=%.1fK -> m_fuel=%.2emg\n',...
sim_count, total_sims, current_rpm, current_t_in, m_fuel*1e6);
sim_output = sim('engine_modelQ4');

% 计算性能并记录
V_cycle = sim_output.V_out; P_cycle = sim_output.P_out;
angle_difference = abs(V_cycle - min(V_cycle)); [~, idx_tdc] = min(angle_difference);
if idx_tdc > 1; V_comp = V_cycle(1:idx_tdc); P_comp = P_cycle(1:idx_tdc); else; V_comp = V_cycle(1); P_comp = P_cycle(1); end
V_exp = V_cycle(idx_tdc:end); P_exp = P_cycle(idx_tdc:end);
V_common = linspace(min(V_cycle), max(V_cycle), 1000);
[V_comp_unique, u_idx_c] = unique(V_comp, 'stable'); P_comp_unique = P_comp(u_idx_c);
[V_exp_unique, u_idx_e] = unique(V_exp, 'stable'); P_exp_unique = P_exp(u_idx_e);
P_lower_interp = interp1(V_comp_unique, P_comp_unique, V_common, 'pchip', 'extrap');
P_upper_interp = interp1(V_exp_unique, P_exp_unique, V_common, 'pchip', 'extrap');
delta_P = P_upper_interp - P_lower_interp;
W_net = trapz(V_common, delta_P);

actual_torque = W_net / (4 * pi);
Q_in = m_fuel * P.LHV;
actual_efficiency = W_net / Q_in;

% 存储到表格
torque_map(j, i) = actual_torque;
efficiency_map(j, i) = actual_efficiency * 100; % 存储为百分比
end
end

% --- 5. 结果可视化 ---
fprintf('\n仿真与计算全部完成，正在绘制结果图...\n');

figure('Name', sprintf('Engine Map @ P_in=%.1fkPa, SOI=%d, Phi=%.1f', P.P_in/1000, P.SOI_deg_BTDC, PHI_target));

% 修改部分: [X, Y]是坐标轴网格，X是转速，Y是进气温度
[X, Y] = meshgrid(rpm_range, t_in_range);

% Z_eff 是效率数据， Z_tor是扭矩数据
Z_eff = efficiency_map;
Z_tor = torque_map;

min_eff=min(Z_eff(:));
max_eff=max(Z_eff(:));

% 绘制效率云图
contourf(X, Y, Z_eff, 20);
hold on;
caxis([min_eff max_eff]);
% 在图上叠加扭矩等高线
[C, h] = contour(X, Y, Z_tor, 'k--', 'LineWidth', 1, 'ShowText','on');
clabel(C, h, 'FontSize', 10, 'Color', 'k', 'FontWeight', 'bold');

hold off;

% 美化图表
colorbar;
title(sprintf('发动机MAP图 @ P_{in}=%.1fkPa, SOI=%d deg, Φ=%.1f', P.P_in/1000, P.SOI_deg_BTDC, PHI_target));
xlabel('转速 (RPM)'); % <-- 修改 X轴标签
ylabel('进气温度 (K)'); % <-- 修改 Y轴标签
c = colorbar;
c.Label.String = '热效率 (%)';

% --- 6. 从全量结果中分析并报告最优解 ---
fprintf('\n======================== 分析结果 ========================\n');
% 找到整个MAP图上热效率最高的点
[max_eff_col, row_indices] = max(efficiency_map);
[max_eff_total, col_idx] = max(max_eff_col);
row_idx = row_indices(col_idx);

% 提取最优点的参数和性能
optimal_rpm = rpm_range(col_idx);
optimal_tin = t_in_range(row_idx);
optimal_torque = torque_map(row_idx, col_idx);

fprintf('在固定P_in, SOI和当量比的条件下，\n');
fprintf('热效率最高的工况点出现在:\n');
fprintf(' - 转速 (RPM): %.0f RPM\n', optimal_rpm);
fprintf(' - 进气温度 (T_in): %.2f K\n', optimal_tin);
fprintf('\n在该最优效率点，发动机的性能为:\n');
fprintf(' - 最高热效率: %.2f %%\n', max_eff_total);
fprintf(' - 对应平均扭矩: %.2f N·m\n', optimal_torque);
fprintf('==========================================================\n');