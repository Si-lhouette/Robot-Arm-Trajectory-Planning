%% 机器人学大作业 - 机械臂设计及其定点转动Minisnap轨迹规划
% Author：高钰满，高诗宁
% Date：2019.12.25 Merry Christmas
% Email: 714023273@qq.com

%% 1.机械臂模型建立
a1 = 0.5;
d3max = 0.7;
d6 = 0.2;
% D-H参数表
L1=RevoluteMDH('a',0,   'alpha',0,      'd',0);
L2=RevoluteMDH('a',a1,  'alpha',-pi/2,  'd',0 , 'offset',0);
L3=PrismaticMDH('a',0,  'alpha', pi/2,  'theta',0);
L3.qlim = [0,d3max];
L4=RevoluteMDH('a',0,   'alpha',0,      'd',0);
L5=RevoluteMDH('a',0,   'alpha',-pi/2,  'd',0);
L6=RevoluteMDH('a',0,   'alpha',pi/2,   'd',d6);
bot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'my robot');
bot.display();%显示D-H参数表

% 机械臂演示
%bot.plot([0,0,0.2,0,0,0]);
% teach(bot) % Matlab Robotics Toolbox源码已被修改

%% 2.路径点选取
% 2.1.以期望第六关节末端为原点，建立球坐标系
co = [0.9, 0, 0];
cr = 0.2;
phi = linspace(0, pi, 50);
te = deg2rad(50);

% 将在球面上取的点记录在向量中，作为第三关节末端坐标
px_all=[];
py_all=[];
pz_all=[];
R06_all={};

% 计算球面上取点的坐标[px,py,pz]和基坐标系下旋转矩阵R06
for i=1:length(phi)
    % 2.2.计算坐标[px,py,pz]
    px = co(1)+cr*sin(phi(i))*cos(te);
    px_all=[px_all,px];
    py = co(2)+cr*sin(phi(i))*sin(te);
    py_all=[py_all,py];
    pz = co(3)+cr*cos(phi(i));
    pz_all=[pz_all,pz];
    
    % 2.3.计算旋转矩阵R06 - Rodriguez formula 
    k6_e = -(co - [px,py,pz])./0.2;                 % 期望的末端Z轴在基坐标系下的单位向量，使末端指向圆心
    Z = [0,0,1];
    C = cross(Z',k6_e');                            % 叉乘求出旋转轴C
    C = C./norm(C);
    C_te = acos(dot(Z,k6_e)/(norm(Z)*norm(k6_e)));  % 求出绕旋转轴的旋转角C_te
    v_t = 1-cos(C_te); 
    % 求从基坐标系Z轴变换到期望末端Z轴向量的旋转矩阵
    R06 = [C(1)*C(1)*v_t+cos(C_te), C(1)*C(2)*v_t-C(3)*sin(C_te), C(1)*C(3)*v_t+C(2)*sin(C_te);
           C(1)*C(2)*v_t+C(3)*sin(C_te), C(2)*C(2)*v_t+cos(C_te), C(2)*C(3)*v_t-C(1)*sin(C_te);
           C(1)*C(3)*v_t-C(2)*sin(C_te), C(2)*C(3)*v_t+C(1)*sin(C_te), C(3)*C(3)*v_t+cos(C_te)];
    
%     k6 = R06*[0;0;1]; %debug时由此值确认R06是否正确
    if sum(isnan(C))==3
        R06 = eye(3);
    end
    R06_all=[R06_all,R06];
end
% 同上方法求另一个半面
te = te+pi;
for i=length(phi):-1:1
    % 2.2.计算坐标[px,py,pz]
    px = co(1)+cr*sin(phi(i))*cos(te);
    px_all=[px_all,px];
    py = co(2)+cr*sin(phi(i))*sin(te);
    py_all=[py_all,py];
    pz = co(3)+cr*cos(phi(i));
    pz_all=[pz_all,pz];
    
    % 2.3.计算旋转矩阵R06 - Rodriguez formula
    k6_e = -(co - [px,py,pz])./0.2;                 % 期望的末端Z轴在基坐标系下的单位向量，使末端指向圆心
    Z = [0,0,1];
    C = cross(Z',k6_e');                            % 叉乘求出旋转轴C
    C = C./norm(C);
    C_te = acos(dot(Z,k6_e)/(norm(Z)*norm(k6_e)));  % 求出绕旋转轴的旋转角C_te
    v_t = 1-cos(C_te); 
    % 求从基坐标系Z轴变换到期望末端Z轴向量的旋转矩阵
    R06 = [C(1)*C(1)*v_t+cos(C_te), C(1)*C(2)*v_t-C(3)*sin(C_te), C(1)*C(3)*v_t+C(2)*sin(C_te);
           C(1)*C(2)*v_t+C(3)*sin(C_te), C(2)*C(2)*v_t+cos(C_te), C(2)*C(3)*v_t-C(1)*sin(C_te);
           C(1)*C(3)*v_t-C(2)*sin(C_te), C(2)*C(3)*v_t+C(1)*sin(C_te), C(3)*C(3)*v_t+cos(C_te)];
    
%     k6 = R06*[0;0;1]; %debug时由此值确认R06是否正确
    if sum(isnan(C))==3
        R06 = eye(3);
    end
    R06_all=[R06_all,R06];
end
% 画出路径点
[sx,sy,sz]=sphere();
figure;
surf(cr*sx+co(1),cr*sy+co(2),cr*sz+co(3),'FaceAlpha',0.1,'EdgeColor','none');%绘制球
hold on
plot3(px_all,py_all,pz_all,'r')
grid on
hold on

%% 3.由路径点计算各关节变量
t1_all=[];
t2_all=[];
L3_all=[];
t4_all=[];
t5_all=[];
t6_all=[];
% 代入公式，计算关节变量
for i=1:length(px_all)
    t1 = atan2(py_all(i),px_all(i));
    t1_all=[t1_all,t1];
    t2 = atan2(cos(t1)*px_all(i)+sin(t1)*py_all(i)-a1,pz_all(i));
    t2_all=[t2_all,t2];
    L3 = cos(t1)*sin(t2)*px_all(i) + sin(t1)*sin(t2)*py_all(i) + cos(t2)*pz_all(i) - a1*sin(t2);
    L3_all=[L3_all,L3];
    
    R06 = R06_all{i};
    t4 = atan2(-sin(t1)*R06(1,3)+cos(t1)*R06(2,3),cos(t2)*(cos(t1)*R06(1,3)+sin(t1)*R06(2,3))-sin(t2)*R06(3,3));
    t4_all=[t4_all,t4];
    s5 = -((cos(t1)*cos(t2)*cos(t4)-sin(t1)*sin(t4))*R06(1,3) + (cos(t1)*sin(t4)+sin(t1)*cos(t2)*cos(t4))*R06(2,3) - sin(t2)*cos(t4)*R06(3,3));
    c5 = -(cos(t1)*sin(t2)*R06(1,3) + sin(t1)*sin(t2)*R06(2,3) + cos(t2)*R06(3,3));
    t5 = atan2(s5,c5);
    t5_all=[t5_all,t5];
end

q_all = [t1_all',t2_all',L3_all',t4_all',t5_all',zeros(length(t1_all),1)];
q_all_h = q_all;
q_all_h(:,[1,2,4,5])=rad2deg(q_all_h(:,[1,2,4,5]));

%% 4.生成机械臂动图
q_all = q_all(2:end-1,:);
% figure
grid on
T=bot.fkine(q_all);%根据插值，得到末端执行器位姿
T=T.T;
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));%输出末端轨迹
hold on
set(gcf,'color','w');
bot.plot(q_all,'scale',0.8,'jointcolor',[1,0.4,0],'view',[12,15]);%动画演示

%% 5.轨迹规划
path = q_all(1:size(q_all,1)/2,:); % 此处生成转半圈的轨迹
path(size(q_all,1)/2,:)=[]; %删除连接处重复点

n = 6; %6个关节
% 5.1. 轨迹生成参数初始化
d_order       = 4;
n_order       = 7;% 轨迹多项式阶数
n_seg         = size(path,1)-1;% 轨迹段数
n_poly_perseg = n_order+1; % 每段轨迹的参数个数

% 每段轨迹的时间分配 Fixed Mode
ts = zeros(n_seg, 1);
for i = 1:n_seg
    ts(i) = 0.1;
end
ts(1)=0.2; %梯形加速过程
ts(end) = 0.2;

% 5.2. Minisnap - QP问题求解
poly_coef_qall=[];
for i=1:n
    poly_coef_q = MinimumSnapQPSolver(path(:, i), ts, n_seg, n_order);
    % or use colseform MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order)
    poly_coef_qall=[poly_coef_qall,poly_coef_q];
end

% 5.3 代入求解P,V,A多项式参数，画出轨迹P,V,A曲线
qall_n=[];
dqall_n=[];
ddqall_n=[];
tall=[];
ts_all=[];%key point
t_begin = 0;

tstep = 0.01;
for j = 1:n
    k = 1;
    for i=0:n_seg-1
        % 求每段轨迹的P,V,A多项式参数
        start_idx = n_poly_perseg * i;
        P = poly_coef_qall(start_idx+1 : start_idx+n_poly_perseg,j);
        P = flipud(P);
        V=[];
        A=[];
        for m = 1:n_order+1
            if m<=n_order
                V=[V;(n_order+1-m)*P(m)];
            else
                V=[0;V];
            end
        end
        for m = 1:n_order+1
            if m<=n_order-1
                A=[A;(n_order-m)*V(m+1)];
            else
                A=[0;A];
            end
        end
        ts_all=[ts_all;t_begin];
        % 在求解出的每段轨迹中进一步取点
        for t=0:tstep:ts(i+1)
            qall_n(k,j)  = polyval(P,t);
            dqall_n(k,j)  = polyval(V,t);
            ddqall_n(k,j)  = polyval(A,t);
            k = k+1;
            if j==1
                tall = [tall;t+t_begin];
            end
        end
        t_begin = t_begin+ts(i+1);
    end
end
ts_all=[ts_all;t_begin];

% 绘图
figure
subplot(3,1,1)
plot(tall, qall_n ,'LineWidth',1);
grid on
title('P');
xlabel('t/s')
ylabel('p/rad')
legend('t1','t2','L3','t4','t5','t6')

subplot(3,1,2)
plot(tall, dqall_n ,'LineWidth',1);
grid on
title('V');
xlabel('t/s')
ylabel('v/rad/s')
legend('t1','t2','L3','t4','t5','t6')

subplot(3,1,3)
plot(tall, ddqall_n ,'LineWidth',1);
grid on
title('A');
xlabel('t/s')
ylabel('a/rad/s^2')
legend('t1','t2','L3','t4','t5','t6')
hold on
set(gcf,'color','w');


function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end

function poly_coef = MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond =   [waypoints(end), 0, 0, 0];
    %#####################################################
    % you have already finished this function in hw1
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 1: compute M
    M = getM(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Ct
    Ct = getCt(n_seg, n_order);
    C = Ct';
    R = C * inv(M)' * Q * inv(M) * Ct;
    R_cell = mat2cell(R, [n_seg+7 3*(n_seg-1)], [n_seg+7 3*(n_seg-1)]);
    R_pp = R_cell{2, 2};
    R_fp = R_cell{1, 2};
    %#####################################################
    % STEP 3: compute dF
    dF = [start_cond';waypoints(2:end-1);end_cond'];

    dP = -inv(R_pp) * R_fp' * dF;
    poly_coef = inv(M) * Ct * [dF;dP];
end

