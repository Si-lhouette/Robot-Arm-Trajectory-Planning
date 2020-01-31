function q_all = get_q(phi, te)
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
% 机械臂演示
%bot.plot([0,0,0.2,0,0,0]);
% teach(bot)

%% 2.轨路径点选取
% 2.1.以期望第六关节末端为原点，建立球坐标系
co = [0.9, 0, 0];
cr = 0.2;
%phi = linspace(0, pi, 100);

% 计算球面上取点的坐标[px,py,pz]和基坐标系下旋转矩阵R06
% 2.2.计算坐标[px,py,pz]
px = co(1)+cr*sin(phi)*cos(te);
py = co(2)+cr*sin(phi)*sin(te);
pz = co(3)+cr*cos(phi);

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

%% 3.由路径点计算各关节变量
% 代入公式，计算关节变量
t1 = atan2(py,px);
t2 = atan2(cos(t1)*px+sin(t1)*py-a1,pz);
L3 = cos(t1)*sin(t2)*px + sin(t1)*sin(t2)*py + cos(t2)*pz - a1*sin(t2);
t4 = atan2(-sin(t1)*R06(1,3)+cos(t1)*R06(2,3),cos(t2)*(cos(t1)*R06(1,3)...
    +sin(t1)*R06(2,3))-sin(t2)*R06(3,3));
s5 = -((cos(t1)*cos(t2)*cos(t4)-sin(t1)*sin(t4))*R06(1,3) + ...
    (cos(t1)*sin(t4)+sin(t1)*cos(t2)*cos(t4))*R06(2,3) - sin(t2)*cos(t4)*R06(3,3));
c5 = -(cos(t1)*sin(t2)*R06(1,3) + sin(t1)*sin(t2)*R06(2,3) + cos(t2)*R06(3,3));
t5 = atan2(s5,c5);
t6 = 0;

q_all = [t1,t2,L3,t4,t5,t6];
q_all_h = q_all;
q_all_h(:,[1,2,4,5])=rad2deg(q_all_h(:,[1,2,4,5]));

end