%% =======================  三容水箱闭环控制系统  ==========================
%% 1、完成三容水箱系统参数辨识。2、完成PID控制器的参数整定
clc;
clear;
close all;

%通过辨识得到确定的参数，获取一组输入输出值 
A = 130;      %水箱横截面积 cm^2
Sn = 2;      %连接阀横截面积 cm^2
a1 = 0.6;     %阀门流量系数 假设阀门规格相同
a2 = 0.6;
a3 = 0.6; 
g = 981;      %cm/s^2

Up_Lim_1 = 20;  %三容水箱输入流量上限
Low_Lim_1 = 0;

%水箱目标水位 cm
Ts = 50000;
H_in = [10*ones(2000,1);linspace(10,40,10)';40*ones(Ts,1);linspace(40,15,10)';20*ones(6000,1)];
sim_time = 1:1:length(H_in);
h_in = [sim_time', H_in];
simtime = length(H_in);    % 仿真时间
% level_cons = 0;  %水箱初始水位  cm

modelName = 'ThreeTank_Control2017a';
nfspath = '\\192.168.3.7\f\NFSD\';


%设置输入格式  ；两输入
TC = Simulink.SimulationData.Dataset;%输入结构体名称
element1 = Simulink.SimulationData.Signal;
element1.Name = 'InputSig1';
element1.Values = timeseries(H_in);     %输入时间序列
TC{1} = element1;
element2 = Simulink.SimulationData.Signal;
element2.Name = 'InputSig2';
element2.Values = timeseries(H_in);
TC{2} = element2;

% p, i, d 参数序列
K_p = 30:1:35;
K_d = 0:1:5;
K_i = 0:1;

%生成仿真任务
numTasks = length(K_p) * length(K_d) * length(K_i);
outputs = cell(numTasks, 1);    %创建一个单元格数组存储仿真结果

tic;
%创建并行池
delete(gcp('nocreate'));
if isempty(gcp('nocreate'))
    pool = parpool('j124',8);
end

%提交并行计算仿真任务
simCount = 0;
for p = K_p
    for d = K_d
        for i = K_i
            simCount = simCount +1;
            simResult(1,simCount) = p;
            simResult(2,simCount) = d;
            simResult(3,simCount) = i;
            %定义仿真函数
%             simFcn = @(modelName, TC, p, d, i) simulateModel(modelName, TC, P, D, I);
            
            %提交任务 使用并行计算工具箱
            f(simCount) = parfeval(pool, @simulateModel, 1, modelName, TC, p, d, i);    %参数：p,fcn,numout,in1,in2,...
        
        end
    end
end

%获取并存储仿真结果
minSSE = Inf;
h_level = [];
for i =  1:simCount
    %等待仿真任务完成并获取结果
    [completedIdx, outputData] = fetchNext(f);
    %存储仿真结果
    outputs{completedIdx} = outputData;
    h_level = outputData.simout.Data;
    
    %计算拟合优度指标
    SSE = sum((h_level(1:end) - H_in).^2);     %误差平方和，SSE越小，曲线拟合程度越
    if(minSSE >= SSE)
        minSSE = SSE;
        minIndex = simCount;
    end
end

%关闭并行计算池
delete(gcp('nocreate'));
toc;

%使用最优的 P I D 值，计算输出响应
Kp = simResult(1,minIndex); %49
Kd = simResult(2,minIndex); %8
Ki = simResult(3,minIndex); %0.8

% assignin('base','Kp_1',Kp);
% assignin('base','Kd_1',Kd);
% assignin('base','Ki_1',Ki);
% assignin('base','Kp_2',Kp);
% assignin('base','Kd_2',Kd);
% assignin('base','Ki_2',Ki);
% 
% %仿真
% sim(modelName);
% figure(1);
% plot(outputs{minIndex, 1}.simout.Time, outputs{minIndex,1}.simout.Data);  %输出响应 液位高度
% hold on;
% plot(sim_time', H_in);  %期望液位高度输出 cm
% title("期望液位高度与输出响应液位高度对比");
% legend("实际输出液位", "期望输出液位");
% xlabel("仿真时间/s");ylabel("液位高度/cm");

%仿真模型函数
function outputData = simulateModel(modelName , TC, P, D, I)
    %加载simulink模型
    load_system(modelName);
    stoptime = TC{1}.Values.Time(end);
    %设置仿真参数
    simIn = Simulink.SimulationInput(modelName);  %模型名称 
    simIn = setExternalInput(simIn, TC);
    simIn = simIn.setModelParameter('StartTime', '0', 'StopTime', num2str(stoptime), 'FixedStep', '1');
    
    simIn = simIn.setBlockParameter([modelName '/PID Controller'], 'P', num2str(P));  %运行变量名称
    simIn = simIn.setBlockParameter([modelName '/PID Controller'], 'I', num2str(I));
    simIn = simIn.setBlockParameter([modelName '/PID Controller'], 'D', num2str(D));
    simIn = simIn.setBlockParameter([modelName '/PID Controller1'], 'P', num2str(P));
    simIn = simIn.setBlockParameter([modelName '/PID Controller1'], 'I', num2str(I));
    simIn = simIn.setBlockParameter([modelName '/PID Controller1'], 'D', num2str(D));
    
    %执行仿真
    outputData = sim(simIn);
    close_system(modelName);
    %获取仿真结果
%     outputData = simOut.get('outputData');
    
end


