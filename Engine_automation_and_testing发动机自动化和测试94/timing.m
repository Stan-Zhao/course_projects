%% Engine Simulation with Performance Calculation

% --- 1. 初始化和设置 ---
clear; clc; close all;
% 首先运行包含发动机参数的脚本
initialize_engine_parameters;

% 定义要扫描的喷油提前角 (SOI) 范围
soi_range_BTDC = 5:5:25;

% 创建空的数组，用于存储所有计算结果
num_points = length(soi_range_BTDC);
torque_results = zeros(1, num_points);
power_results_kW = zeros(1, num_points);
efficiency_results = zeros(1, num_points);
bsfc_results = zeros(1, num_points);

% --- 2. 循环运行仿真 ---
for i = 1:num_points
current_soi = soi_range_BTDC(i);
fprintf('正在运行仿真: 喷油提前角 = %d deg BTDC\n', current_soi);

% --- 修改模型参数 ---
% 将脚本中的 P 结构体参数传递给 Simulink 模型
assignin('base', 'P', P);
% 修改本次循环的喷油提前角
set_param('engine_model/SOI_Timing', 'Value', num2str(current_soi));

% --- 运行仿真 ---
sim_output = sim('engine_model');

% --- 3. 数据后处理与性能计算 ---
V_cycle = sim_output.V_out;
P_cycle = sim_output.P_out;

% (1) 计算净功和扭矩
W_net = trapz(V_cycle, P_cycle); % 净功 (J)
torque = W_net / (4 * pi); % 平均扭矩 (N·m)

% (2) 计算功率
cycles_per_second = (P.RPM / 60) / 2;
power_watts = W_net * cycles_per_second; % 功率 (W)
power_kW = power_watts / 1000; % 功率 (kW)

% (3) 计算热效率
% 在这里需要重复燃烧逻辑来计算Q_in
m_air = (P.P_in * max(V_cycle)) / (P.R_air * P.T_in);
equivalence_ratio = (P.m_fuel / m_air) * P.AFR_stoich;
if equivalence_ratio <= 1
m_fuel_burned = P.m_fuel;
else
m_fuel_burned = m_air / P.AFR_stoich;
end
Q_in = m_fuel_burned * P.LHV; % 燃烧热量 (J)
thermal_efficiency = W_net / Q_in; % 热效率 (-)

% (4) 计算燃油消耗率 (BSFC)
fuel_rate_g_per_hour = P.m_fuel * 1000 * cycles_per_second * 3600; % (g/h)
% 只有当功率大于0时，BSFC才有意义，避免除以零
if power_kW > 0
bsfc = fuel_rate_g_per_hour / power_kW; % (g/kWh)
else
bsfc = inf; % 或 NaN
end

% --- 存储结果 ---
torque_results(i) = torque;
power_results_kW(i) = power_kW;
efficiency_results(i) = thermal_efficiency * 100; % 存为百分比
bsfc_results(i) = bsfc;
end

% --- 4. 结果可视化 ---
fprintf('仿真完成，正在绘制结果图...\n');

figure('Name', '发动机性能随喷油时刻的变化');

% 创建一个 2x2 的子图布局
subplot(2, 2, 1);
plot(soi_range_BTDC, torque_results, 'o-', 'LineWidth', 2);
title('Torque vs. Injection Timing');
xlabel('Injection Timing (deg BTDC)');
ylabel('Torque (N·m)');
grid on;

subplot(2, 2, 2);
plot(soi_range_BTDC, power_results_kW, 's-r', 'LineWidth', 2);
title('Power vs. Injection Timing');
xlabel('Injection Timing (deg BTDC)');
ylabel('Output Power (kW)');
grid on;

subplot(2, 2, 3);
plot(soi_range_BTDC, efficiency_results, 'd-g', 'LineWidth', 2);
title('Thermal Efficiency vs. Injection Timing');
xlabel('Injection Timing (deg BTDC)');
ylabel('Thermal Effficiency (%)');
grid on;

subplot(2, 2, 4);
plot(soi_range_BTDC, bsfc_results, '*-k', 'LineWidth', 2);
title('Fuel Consumption vs. Injection Timing');
xlabel('Injection Timing (deg BTDC)');
ylabel('BSFC (g/kWh)');
grid on;


% =========================================================================
% ### 新增：绘制缸内工作过程对比图 ###
% =========================================================================

% --- 1. 循环前: 创建图形窗口和子图布局 ---
figure('Name', '不同喷油时刻下的缸内工作过程对比');
color_map = lines(num_points); % 创建一个颜色数组，让每条线颜色不同
legend_text = cell(1, num_points); % 创建一个单元数组用于存放图例文本

% 设置 P-V 子图
subplot(2, 2, 1);
hold on; % 开启保持模式，允许多次绘图
title('P-V 图对比');
xlabel('Volume (m^3)');
ylabel('Pressure (Pa)');
grid on;

% 设置 P-φ 子图
subplot(2, 2, 2);
hold on;
title('P-φ 图对比');
xlabel('Crank Angle (deg)');
ylabel('Pressure (Pa)');
grid on;

% 设置 T-φ 子图
subplot(2, 2, 3);
hold on;
title('T-φ 图对比');
xlabel('Crank Angle (deg)');
ylabel('Torque (N·m)');
grid on;


% --- 2. 循环中: 重新运行仿真并绘图 ---
for i = 1:num_points
current_soi = soi_range_BTDC(i);
fprintf('正在为对比图重新运行仿真: SOI = %d deg\n', current_soi);

% 修改参数并运行仿真 (与之前的循环相同)
set_param('engine_model/SOI_Timing', 'Value', num2str(current_soi));
sim_output = sim('engine_model');

% 获取绘图所需数据
P_cycle = sim_output.P_out;
V_cycle = sim_output.V_out;
T_cycle = sim_output.T_out;
phi_cycle = sim_output.phi_out;

% --- 在对应的子图上绘制当前工况的曲线 ---

% 绘制 P-V 图
subplot(2, 2, 1);
plot(V_cycle, P_cycle, 'LineWidth', 1.5, 'Color', color_map(i,:));

% 绘制 P-φ 图
subplot(2, 2, 2);
plot(phi_cycle, P_cycle, 'LineWidth', 1.5, 'Color', color_map(i,:));

% 绘制 T-φ 图
subplot(2, 2, 3);
plot(phi_cycle, T_cycle, 'LineWidth', 1.5, 'Color', color_map(i,:));

% 准备图例文本
legend_text{i} = sprintf('SOI = %d deg BTDC', current_soi);
end


% --- 3. 循环后: 添加图例 ---
subplot(2, 2, 1);
legend(legend_text);
hold off; % 关闭保持模式

subplot(2, 2, 2);
legend(legend_text);
hold off;

subplot(2, 2, 3);
legend(legend_text);
hold off;

fprintf('所有绘图已完成。\n');
