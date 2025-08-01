%% ========================================================================
% 发动机性能二维MAP图绘制与寻优脚本 (V4 - SOI vs. m_fuel)
% =========================================================================
%
% 目标:
% 在固定转速和进气压力下，遍历喷油提前角和喷油量的组合，
% 绘制以“热效率”为背景色，“扭矩”为等高线的二维MAP图。
%
%==========================================================================

% --- 1. 初始化和设置 ---
clear; clc; close all;
fprintf('开始执行二维MAP图寻优与可视化 (SOI vs. m_fuel)...\n\n');

% 首先运行包含发动机参数的脚本
initialize_engine_parameters;

% 加载Simulink模型到内存中
load_system('engine_modelQ4');

% --- 2. 设定固定参数与搜索范围 ---

% --- 修改部分: 设定新的固定参数 ---
P.RPM = 1800; % 固定转速 (rev/min)
P.P_in = 1.66e5; % <-- 固定进气压力为166 kPa (1.66e5 Pa)
fprintf('固定参数: 转速 = %d RPM, 进气压力 = %.1f kPa\n', P.RPM, P.P_in/1000);

% --- 修改部分: 设定新的搜索范围 ---
soi_range = linspace(0, 25, 6); % 喷油提前角搜索范围 (度BTDC), 分21步
m_fuel_range = linspace(4e-5, 12e-5, 10); % 喷油量搜索范围 (kg/cycle), 分20步

total_sims = length(soi_range) * length(m_fuel_range);
fprintf('将在 %d 个参数组合中进行搜索，请耐心等待...\n\n', total_sims);

% --- 3. 初始化用于存储所有结果的矩阵 ---
% 修改部分: 矩阵的维度现在由soi和m_fuel的长度决定
torque_map = zeros(length(soi_range), length(m_fuel_range));
efficiency_map = zeros(length(soi_range), length(m_fuel_range));


% --- 4. 双重嵌套循环，遍历并保存所有结果 ---
% 修改部分: 外层循环现在遍历喷油提前角
for j = 1:length(soi_range)
% 内层循环遍历喷油量
for i = 1:length(m_fuel_range)

% --- 设置当前循环的参数 ---
current_soi = soi_range(j);
current_m_fuel = m_fuel_range(i);

P.m_fuel = current_m_fuel;
% 使用 set_param 修改名为'SOI_Timing'的模块的值
set_param('engine_modelQ4/SOI_Timing', 'Value', num2str(current_soi));

% 运行仿真并计算性能...
fprintf('运行工况 %d/%d: SOI=%.1f deg, m_fuel=%.2emg\n', ...
(j-1)*length(m_fuel_range)+i, total_sims, current_soi, current_m_fuel*1e6);
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

% 修改部分: 更新图表标题和坐标轴
figure('Name', sprintf('Engine Map @ %d RPM, P_in = %.1f kPa', P.RPM, P.P_in/1000));

% [X, Y]是坐标轴网格，X是喷油量，Y是喷油时刻
[X, Y] = meshgrid(m_fuel_range, soi_range);

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
colorbar;
title(sprintf('发动机MAP图 @ %d RPM, P_{in} = %.1f kPa', P.RPM, P.P_in/1000));
xlabel('喷油量 (kg/cycle)');
ylabel('喷油提前角 (度 BTDC)'); % <-- 修改 Y轴标签
c = colorbar;
c.Label.String = '热效率 (%)';

fprintf('所有分析已完成。\n');