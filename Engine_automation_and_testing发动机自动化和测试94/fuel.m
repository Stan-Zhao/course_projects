%% Engine Simulation with Performance Calculation - SWEEPING INJECTION QUANTITY

% --- 1. 初始化和设置 ---
clear; clc; close all;
% 首先运行包含发动机参数的脚本，它会提供一个初始的 P 结构体
initialize_engine_parameters;

% 在运行仿真前，先显式加载Simulink模型到内存中
load_system('engine_modelQ4');

% --- 修改部分: 定义要扫描的喷油量范围 ---
% 您可以根据需要修改这个范围。单位是 kg/cycle。
% P.m_fuel 的初始值约为 5.5e-5 kg，我们在此基础上进行扫描。
m_fuel_range_kg = (3.5e-5) : (1e-5) : (7.5e-5);

% 创建空的数组，用于存储所有计算结果
num_points = length(m_fuel_range_kg);
torque_results = zeros(1, num_points);
power_results_kW = zeros(1, num_points);
efficiency_results = zeros(1, num_points);
bsfc_results = zeros(1, num_points);


% --- 2. 循环运行仿真 ---
for i = 1:num_points
% --- 修改部分: 获取当前的喷油量 ---
current_m_fuel = m_fuel_range_kg(i);
fprintf('正在运行仿真: 喷油量 = %.2e kg/cycle\n', current_m_fuel);

% --- 修改部分: 直接修改参数结构体 P 中的喷油量字段 ---
P.m_fuel = current_m_fuel;
% 注意：我们固定了喷油时刻，脚本不再修改它

% --- 运行仿真 ---
% sim 命令会使用工作区中最新的 P 结构体 (如果您的模型这样设置)
sim_output = sim('engine_model');

% --- 3. 数据后处理与性能计算 ---
V_cycle = sim_output.V_out;
P_cycle = sim_output.P_out;

% (1) 计算净功和扭矩
W_net = trapz(V_cycle, P_cycle);
torque = W_net / (4 * pi);

% (2) 计算功率
cycles_per_second = (P.RPM / 60) / 2;
power_watts = W_net * cycles_per_second;
power_kW = power_watts / 1000;

% (3) 计算热效率
m_air = (P.P_in * max(V_cycle)) / (P.R_air * P.T_in);
equivalence_ratio = (P.m_fuel / m_air) * P.AFR_stoich;
if equivalence_ratio <= 1
m_fuel_burned = P.m_fuel;
else
m_fuel_burned = m_air / P.AFR_stoich;
end
Q_in = m_fuel_burned * P.LHV;
thermal_efficiency = W_net / Q_in;

% (4) 计算燃油消耗率 (BSFC)
fuel_rate_g_per_hour = P.m_fuel * 1000 * cycles_per_second * 3600;
if power_kW > 0
bsfc = fuel_rate_g_per_hour / power_kW;
else
bsfc = inf;
end

% --- 存储结果 ---
torque_results(i) = torque;
power_results_kW(i) = power_kW;
efficiency_results(i) = thermal_efficiency * 100;
bsfc_results(i) = bsfc;
end

% --- 4. 结果可视化 ---
fprintf('仿真完成，正在绘制结果图...\n');

% === 图一：发动机宏观性能图 ===
figure('Name', '发动机性能随喷油量的变化'); % <-- 修改部分: 窗口标题

% 创建一个 2x2 的子图布局
subplot(2, 2, 1);
plot(m_fuel_range_kg, torque_results, 'o-', 'LineWidth', 2); % <-- 修改部分: X轴数据
title('Toque vs. Ignition Fuel'); % <-- 修改部分: 标题
xlabel('Ignition Fuel (kg/cycle)'); % <-- 修改部分: X轴标签
ylabel('Torque (N·m)');
grid on;

subplot(2, 2, 2);
plot(m_fuel_range_kg, power_results_kW, 's-r', 'LineWidth', 2);
title('Power vs. Ignition Fuel');
xlabel('Ignition Fuel (kg/cycle)');
ylabel('Output Power (kW)');
grid on;

subplot(2, 2, 3);
plot(m_fuel_range_kg, efficiency_results, 'd-g', 'LineWidth', 2);
title('Thermal Efficiency vs. Ignition Fuel');
xlabel('Ignition Fuel (kg/cycle)');
ylabel('Thermal Efficiency  (%)');
grid on;

subplot(2, 2, 4);
plot(m_fuel_range_kg, bsfc_results, '*-k', 'LineWidth', 2);
title('Fuel Consumption vs. Ignition Fuel');
xlabel('Ignition Fuel (kg/cycle)');
ylabel('BSFC (g/kWh)');
grid on;


% === 图二：缸内工作过程对比图 ===
figure('Name', '不同喷油量下的缸内工作过程对比'); % <-- 修改部分: 窗口标题
color_map = lines(num_points);
legend_text = cell(1, num_points);

% 设置 P-V 子图
subplot(2, 2, 1); hold on; title('P-V 图对比'); xlabel('Volume (m^3)'); ylabel('Pressure (Pa)'); grid on;
% 设置 P-φ 子图
subplot(2, 2, 2); hold on; title('P-φ 图对比'); xlabel('Crank Angle  (deg)'); ylabel('Pressure (Pa)'); grid on;
% 设置 T-φ 子图
subplot(2, 2, 3); hold on; title('T-φ 图对比'); xlabel('Crank Angle  (deg)'); ylabel('Torque (N·m)'); grid on;

% 重新运行仿真并绘图
for i = 1:num_points
current_m_fuel = m_fuel_range_kg(i); % <-- 修改部分: 循环变量
fprintf('正在为对比图重新运行仿真: 喷油量 = %.2e kg\n', current_m_fuel);

P.m_fuel = current_m_fuel; % <-- 修改部分: 修改参数
sim_output = sim('engine_modelQ4');

P_cycle = sim_output.P_out; V_cycle = sim_output.V_out; T_cycle = sim_output.T_out; phi_cycle = sim_output.phi_out;

subplot(2, 2, 1); plot(V_cycle, P_cycle, 'LineWidth', 1.5, 'Color', color_map(i,:));
subplot(2, 2, 2); plot(phi_cycle, P_cycle, 'LineWidth', 1.5, 'Color', color_map(i,:));
subplot(2, 2, 3); plot(phi_cycle,T_cycle, 'LineWidth', 1.5, 'Color', color_map(i,:)); % 注意: T-phi图的坐标轴顺序

legend_text{i} = sprintf('m_{fuel} = %.2e kg', current_m_fuel); % <-- 修改部分: 图例文本
end

% 添加图例
subplot(2, 2, 1); legend(legend_text); hold off;
subplot(2, 2, 2); legend(legend_text); hold off;
subplot(2, 2, 3); legend(legend_text); hold off;

fprintf('所有绘图已完成。\n');