%% ========================================================================
% 发动机性能二维MAP图绘制与寻优脚本 (V3 - 简化版)
% =========================================================================
%
% 目标:
% 在固定转速和喷油时刻下，遍历进气压力和喷油量的组合，
% 绘制以“热效率”为背景色，“扭矩”为等高线的二维MAP图，
% 从而直观地寻找最优参数组合。
%
%==========================================================================

% --- 1. 初始化和设置 ---
clear; clc; close all;
fprintf('开始执行二维MAP图寻优与可视化...\n\n');

% 首先运行包含发动机参数的脚本
initialize_engine_parameters;

% 加载Simulink模型到内存中
load_system('engine_modelQ4');

% --- 2. 设定固定参数与搜索范围 ---

% --- 固定参数 ---
P.RPM = 1800; % 固定转速 (rev/min)
P.SOI_deg_BTDC = 10; % <-- 固定喷油提前角为10度
fprintf('固定参数: 转速 = %d RPM, 喷油提前角 = %d 度 BTDC\n', P.RPM, P.SOI_deg_BTDC);
set_param('engine_modelQ4/SOI_Timing', 'Value', num2str(P.SOI_deg_BTDC));

% --- 搜索范围设定 ---
p_in_range = linspace(1.2e5, 2.5e5, 5); % 进气压力搜索范围 (Pa), 分15步
m_fuel_range = linspace(2.5e-5, 30e-5, 10); % 喷油量搜索范围 (kg/cycle), 分20步

total_sims = length(p_in_range) * length(m_fuel_range);
fprintf('将在 %d 个参数组合中进行搜索，请耐心等待...\n\n', total_sims);

% --- 3. 初始化用于存储所有结果的矩阵 ---
% 创建二维矩阵来存储结果
torque_map = zeros(length(p_in_range), length(m_fuel_range));
efficiency_map = zeros(length(p_in_range), length(m_fuel_range));


% --- 4. 双重嵌套循环，遍历并保存所有结果 ---
for j = 1:length(p_in_range) % 进气压力的循环
for i = 1:length(m_fuel_range) % 喷油量的循环

% --- 设置当前循环的参数 ---
P.P_in = p_in_range(j);
P.m_fuel = m_fuel_range(i);

% 运行仿真并计算性能...
fprintf('运行工况 %d/%d: P_in=%.1fkPa, m_fuel=%.2emg\n', ...
(j-1)*length(m_fuel_range)+i, total_sims, P.P_in/1000, P.m_fuel*1e6);
sim_output = sim('engine_modelQ4');

V_cycle = sim_output.V_out; P_cycle = sim_output.P_out; phi_cycle = sim_output.phi_out;
angle_difference = abs(phi_cycle - 180); [~, idx_tdc] = min(angle_difference);
if idx_tdc > 1; V_comp = V_cycle(1:idx_tdc); P_comp = P_cycle(1:idx_tdc); else; V_comp = V_cycle(1); P_comp = P_cycle(1); end
V_exp = V_cycle(idx_tdc:end); P_exp = P_cycle(idx_tdc:end);
V_common = linspace(min(V_cycle), max(V_cycle), 1000);
[V_comp_unique, u_idx_c] = unique(V_comp, 'stable'); P_comp_unique = P_comp(u_idx_c);
[V_exp_unique, u_idx_e] = unique(V_exp, 'stable'); P_exp_unique = P_exp(u_idx_e);
P_lower_interp = interp1(V_comp_unique, P_comp_unique, V_common, 'pchip', 'extrap');
P_upper_interp = interp1(V_exp_unique, P_exp_unique, V_common, 'pchip', 'extrap');
delta_P = P_upper_interp - P_lower_interp;
W_net = trapz(V_common, delta_P);

% 存储结果到二维矩阵中
torque_map(j, i) = W_net / (4 * pi);

m_air = (P.P_in * max(V_cycle)) / (P.R_air * P.T_in);
equivalence_ratio = (P.m_fuel / m_air) * P.AFR_stoich;
if equivalence_ratio <= 1; m_fuel_burned = P.m_fuel; else; m_fuel_burned = m_air / P.AFR_stoich; end
Q_in = m_fuel_burned * P.LHV;

if Q_in > 0; efficiency_map(j, i) = W_net / Q_in; else; efficiency_map(j, i) = 0; end
end
end

% --- 5. 结果可视化 ---
fprintf('\n仿真与计算全部完成，正在绘制结果图...\n');

figure('Name', sprintf('Engine Map @ %d RPM', P.RPM));

% [X, Y]是坐标轴网格，X是喷油量，Y是进气压力
[X, Y] = meshgrid(m_fuel_range, p_in_range);

% Z_eff 是效率数据， Z_tor是扭矩数据
Z_eff = efficiency_map * 100; % 转为百分比

min_eff=min(Z_eff(:));
max_eff=max(Z_eff(:));


Z_tor = torque_map;

% --- 绘制效率云图 ---
contourf(X, Y, Z_eff, 20); % 绘制填充的等高线图
hold on;

clim([min_eff max_eff]);

% --- 在图上叠加扭矩等高线 ---
[C, h] = contour(X, Y, Z_tor, 'k--', 'LineWidth', 1, 'ShowText','on'); % 用虚线表示扭矩
clabel(C, h, 'FontSize', 10, 'Color', 'k', 'FontWeight', 'bold');

hold off;

% --- 美化图表 ---
colorbar; % 显示颜色条，其数值代表热效率
title(sprintf('发动机MAP图 @ %d RPM, SOI = %d BTDC', P.RPM, P.SOI_deg_BTDC));
xlabel('喷油量 (kg/cycle)');
ylabel('进气压力 (Pa)');
c = colorbar;
c.Label.String = '热效率 (%)';

fprintf('所有分析已完成。\n');