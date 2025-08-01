%% ========================================================================
% 发动机多工况热效率寻优与可视化脚本
% =========================================================================
%
% 目标:
% 在固定转速和当量比的约束下，针对一系列的目标扭矩，
% 通过遍历(SOI, P_in)参数空间，寻找每个目标扭矩下热效率最高的
% 参数组合，并将寻优过程与结果可视化。
%
%==========================================================================

% --- 1. 初始化和设置 ---
clear; clc; close all;
fprintf('开始执行多工况热效率寻优脚本...\n\n');

% 首先运行包含发动机参数的脚本
initialize_engine_parameters;

% 加载Simulink模型到内存中
load_system('engine_modelQ4');

% --- 2. 设定固定参数、寻优目标与搜索范围 ---

% --- 固定参数 ---
P.RPM = 1000; % <-- 固定一个有代表性的转速
P.T_in = 423; % <-- 固定一个合理的进气温度 (K)
PHI_target = 0.8; % <-- 固定目标当量比 (柴油机典型浓工况)
fprintf('固定参数: 转速=%d RPM, T_in=%.1f K, 当量比 Φ=%.1f\n', P.RPM, P.T_in, PHI_target);

% --- 设定目标扭矩范围 ---
% 假设通过之前的仿真，已知该发动机的峰值扭矩约为500 N·m
T_max = 450;
target_torques = linspace(T_max * 0.9, T_max * 1.0, 2); % 从10%到100%扭矩，取10个点
torque_tolerance = 15; % 扭矩误差容忍范围 (± N·m)

% --- 初始化最终结果存储表格 ---
optimal_results = table();


% --- 3. 外层循环: 遍历每一个目标扭矩 ---
for t_idx = 1:length(target_torques)

current_target_torque = target_torques(t_idx);
fprintf('\n-----------------------------------------------------------------\n');
fprintf('开始为目标扭矩 %.1f N·m 进行寻优...\n', current_target_torque);
fprintf('-----------------------------------------------------------------\n');

% --- 内层循环的搜索范围设定 ---
soi_range = linspace(0, 25, 8); % 喷油提前角搜索范围
p_in_range = linspace(1.2e5, 2.5e5, 6); % 进气压力搜索范围

% --- 初始化用于存储当前扭矩目标下所有结果的矩阵 ---
torque_map = zeros(length(p_in_range), length(soi_range));
efficiency_map = zeros(length(p_in_range), length(soi_range));

% --- 内层双重嵌套循环，遍历(SOI, P_in)参数组合 ---
for j = 1:length(p_in_range)
for i = 1:length(soi_range)
% 设置当前循环的参数
current_p_in = p_in_range(j);
current_soi = soi_range(i);

P.P_in = current_p_in;
set_param('engine_modelQ4/SOI_Timing', 'Value', num2str(current_soi));

% 核心计算：根据约束确定喷油量
m_air = (P.P_in * (2.6e-3)) / (P.R_air * P.T_in);
P.m_fuel = (PHI_target / P.AFR_stoich) * m_air;

% 运行仿真
sim_output = sim('engine_modelQ4');

% 计算并存储结果
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

torque_map(j, i) = W_net / (4 * pi);
Q_in = P.m_fuel * P.LHV;
if Q_in > 0; efficiency_map(j, i) = W_net / Q_in; else; efficiency_map(j, i) = 0; end
end
end

% --- 4. 对当前目标扭矩的寻优结果进行分析和可视化 ---

% 筛选出所有扭矩在目标范围内的“合格”点
qualified_indices = find(abs(torque_map - current_target_torque) <= torque_tolerance);

if isempty(qualified_indices)
fprintf('警告: 未能找到扭矩满足要求的工况点，请尝试调整搜索范围。\n');
continue; % 继续下一个目标扭矩的寻优
end

% 在“合格”的点中，找到热效率最高的那个点的索引
[max_eff, idx_in_qualified] = max(efficiency_map(qualified_indices));
optimal_idx = qualified_indices(idx_in_qualified);

% 将一维索引转换为二维的行列索引
[best_j, best_i] = ind2sub(size(torque_map), optimal_idx);

% 获取最优参数
optimal_soi = soi_range(best_i)+P.RPM*0.018;
optimal_pin = p_in_range(best_j);

% --- 绘制可视化MAP图 ---
figure('Name', sprintf('寻优过程: 目标扭矩 = %.1f N·m', current_target_torque));
[X, Y] = meshgrid(soi_range, p_in_range);
Z_eff = efficiency_map * 100;
Z_tor = torque_map;

min_eff=min(Z_eff(:));
max_eff=max(Z_eff(:))+0.001;

contourf(X, Y, Z_eff, 20); % 绘制效率云图
hold on;

clim([min_eff max_eff]);

contour(X, Y, Z_tor, 'k--', 'ShowText', 'on'); % 叠加扭矩等高线
contour(X, Y, Z_tor, [current_target_torque, current_target_torque], 'r-', 'LineWidth', 3); % 高亮目标扭矩线

% 在图上用一个大大的红色星号标记出找到的最优点
plot(optimal_soi, optimal_pin, 'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'r');

hold off;
colorbar; title(sprintf('目标扭矩 %.1f N·m 的最优参数寻优图', current_target_torque));
xlabel('喷油提前角 (度 BTDC)'); ylabel('进气压力 (Pa)');
c = colorbar; c.Label.String = '热效率 (%)';

% --- 记录最终结果 ---
temp_table = table(current_target_torque, optimal_soi, optimal_pin, max_eff*100, ...
'VariableNames', {'Target_Torque', 'Optimal_SOI', 'Optimal_Pin_Pa', 'Max_Efficiency_pct'});
optimal_results = [optimal_results; temp_table];
end

% --- 5. 总结报告 ---
fprintf('\n\n======================== 最终寻优总结报告 ========================\n');
disp('在各个目标扭矩下，热效率最高对应的最优参数如下:');
disp(optimal_results);
fprintf('====================================================================\n');