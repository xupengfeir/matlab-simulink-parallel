%% =======================  ����ˮ��ջ�����ϵͳ  ==========================
%% 1���������ˮ��ϵͳ������ʶ��2�����PID�������Ĳ�������
clc;
clear;
close all;

%ͨ����ʶ�õ�ȷ���Ĳ�������ȡһ���������ֵ 
A = 130;      %ˮ������� cm^2
Sn = 2;      %���ӷ������� cm^2
a1 = 0.6;     %��������ϵ�� ���跧�Ź����ͬ
a2 = 0.6;
a3 = 0.6; 
g = 981;      %cm/s^2

Up_Lim_1 = 20;  %����ˮ��������������
Low_Lim_1 = 0;

%ˮ��Ŀ��ˮλ cm
Ts = 50000;
H_in = [10*ones(2000,1);linspace(10,40,10)';40*ones(Ts,1);linspace(40,15,10)';20*ones(6000,1)];
sim_time = 1:1:length(H_in);
h_in = [sim_time', H_in];
simtime = length(H_in);    % ����ʱ��
% level_cons = 0;  %ˮ���ʼˮλ  cm

modelName = 'ThreeTank_Control2017a';
nfspath = '\\192.168.3.7\f\NFSD\';


%���������ʽ  ��������
TC = Simulink.SimulationData.Dataset;%����ṹ������
element1 = Simulink.SimulationData.Signal;
element1.Name = 'InputSig1';
element1.Values = timeseries(H_in);     %����ʱ������
TC{1} = element1;
element2 = Simulink.SimulationData.Signal;
element2.Name = 'InputSig2';
element2.Values = timeseries(H_in);
TC{2} = element2;

% p, i, d ��������
K_p = 30:1:35;
K_d = 0:1:5;
K_i = 0:1;

%���ɷ�������
numTasks = length(K_p) * length(K_d) * length(K_i);
outputs = cell(numTasks, 1);    %����һ����Ԫ������洢������

tic;
%�������г�
delete(gcp('nocreate'));
if isempty(gcp('nocreate'))
    pool = parpool('j124',8);
end

%�ύ���м����������
simCount = 0;
for p = K_p
    for d = K_d
        for i = K_i
            simCount = simCount +1;
            simResult(1,simCount) = p;
            simResult(2,simCount) = d;
            simResult(3,simCount) = i;
            %������溯��
%             simFcn = @(modelName, TC, p, d, i) simulateModel(modelName, TC, P, D, I);
            
            %�ύ���� ʹ�ò��м��㹤����
            f(simCount) = parfeval(pool, @simulateModel, 1, modelName, TC, p, d, i);    %������p,fcn,numout,in1,in2,...
        
        end
    end
end

%��ȡ���洢������
minSSE = Inf;
h_level = [];
for i =  1:simCount
    %�ȴ�����������ɲ���ȡ���
    [completedIdx, outputData] = fetchNext(f);
    %�洢������
    outputs{completedIdx} = outputData;
    h_level = outputData.simout.Data;
    
    %��������Ŷ�ָ��
    SSE = sum((h_level(1:end) - H_in).^2);     %���ƽ���ͣ�SSEԽС��������ϳ̶�Խ
    if(minSSE >= SSE)
        minSSE = SSE;
        minIndex = simCount;
    end
end

%�رղ��м����
delete(gcp('nocreate'));
toc;

%ʹ�����ŵ� P I D ֵ�����������Ӧ
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
% %����
% sim(modelName);
% figure(1);
% plot(outputs{minIndex, 1}.simout.Time, outputs{minIndex,1}.simout.Data);  %�����Ӧ Һλ�߶�
% hold on;
% plot(sim_time', H_in);  %����Һλ�߶���� cm
% title("����Һλ�߶��������ӦҺλ�߶ȶԱ�");
% legend("ʵ�����Һλ", "�������Һλ");
% xlabel("����ʱ��/s");ylabel("Һλ�߶�/cm");

%����ģ�ͺ���
function outputData = simulateModel(modelName , TC, P, D, I)
    %����simulinkģ��
    load_system(modelName);
    stoptime = TC{1}.Values.Time(end);
    %���÷������
    simIn = Simulink.SimulationInput(modelName);  %ģ������ 
    simIn = setExternalInput(simIn, TC);
    simIn = simIn.setModelParameter('StartTime', '0', 'StopTime', num2str(stoptime), 'FixedStep', '1');
    
    simIn = simIn.setBlockParameter([modelName '/PID Controller'], 'P', num2str(P));  %���б�������
    simIn = simIn.setBlockParameter([modelName '/PID Controller'], 'I', num2str(I));
    simIn = simIn.setBlockParameter([modelName '/PID Controller'], 'D', num2str(D));
    simIn = simIn.setBlockParameter([modelName '/PID Controller1'], 'P', num2str(P));
    simIn = simIn.setBlockParameter([modelName '/PID Controller1'], 'I', num2str(I));
    simIn = simIn.setBlockParameter([modelName '/PID Controller1'], 'D', num2str(D));
    
    %ִ�з���
    outputData = sim(simIn);
    close_system(modelName);
    %��ȡ������
%     outputData = simOut.get('outputData');
    
end


