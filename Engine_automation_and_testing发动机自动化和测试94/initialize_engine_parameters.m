%% ==========================================================
%% 发动机模型初始化参数 (Engine Model Parameters)
%% ==========================================================
clear; clc; close all;

disp('正在加载发动机参数...');

% --- 1. 几何参数 (Geometric Parameters) ---
P.Bore = 0.140; % 缸径 (m)
P.Stroke = 0.152; % 冲程 (m)
P.ConnRod = 0.305; % 连杆长度 (m)
P.CR = 10; % 压缩比 (-)

% --- 2. 热力学参数 (Thermodynamic Parameters) ---
P.gamma = 1.4; % 绝热指数 (-)
P.P_in = 166000; % 进气压力 (Pa)
P.T_in = 423; % 进气温度 (K)
P.R_air = 287; % 空气气体常数 (J/kg*K)
P.cv = P.R_air / (P.gamma - 1); % 空气定容比热 (J/kg*K)

% --- 3. 燃料与燃烧参数 (Fuel & Combustion Parameters) ---
P.LHV = 42.5e6; % 燃料低热值 (J/kg)
P.AFR_stoich = 14.5; % 化学计量空燃比 (-)
P.m_fuel = 15e-5; % 每个循环的喷油量 (kg/cycle), 从g转换为kg

% --- 4. 运行与控制参数 (Operational & Control Parameters) ---
P.RPM = 1800; % 发动机转速 (rev/min)
P.SOI_deg_BTDC = 10; % 喷油提前角 (度, 上止点前)

disp('参数加载完成。');

% abs(P.SOI_deg_BTDC-0.018*P.RPM)