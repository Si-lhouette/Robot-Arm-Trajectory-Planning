function q_all = get_q(phi, te)
%% 1.��е��ģ�ͽ���
a1 = 0.5;
d3max = 0.7;
d6 = 0.2;

% D-H������
L1=RevoluteMDH('a',0,   'alpha',0,      'd',0);
L2=RevoluteMDH('a',a1,  'alpha',-pi/2,  'd',0 , 'offset',0);
L3=PrismaticMDH('a',0,  'alpha', pi/2,  'theta',0);
L3.qlim = [0,d3max];
L4=RevoluteMDH('a',0,   'alpha',0,      'd',0);
L5=RevoluteMDH('a',0,   'alpha',-pi/2,  'd',0);
L6=RevoluteMDH('a',0,   'alpha',pi/2,   'd',d6);
bot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'my robot');
% ��е����ʾ
%bot.plot([0,0,0.2,0,0,0]);
% teach(bot)

%% 2.��·����ѡȡ
% 2.1.�����������ؽ�ĩ��Ϊԭ�㣬����������ϵ
co = [0.9, 0, 0];
cr = 0.2;
%phi = linspace(0, pi, 100);

% ����������ȡ�������[px,py,pz]�ͻ�����ϵ����ת����R06
% 2.2.��������[px,py,pz]
px = co(1)+cr*sin(phi)*cos(te);
py = co(2)+cr*sin(phi)*sin(te);
pz = co(3)+cr*cos(phi);

% 2.3.������ת����R06 - Rodriguez formula 
k6_e = -(co - [px,py,pz])./0.2;                 % ������ĩ��Z���ڻ�����ϵ�µĵ�λ������ʹĩ��ָ��Բ��
Z = [0,0,1];
C = cross(Z',k6_e');                            % ��������ת��C
C = C./norm(C);
C_te = acos(dot(Z,k6_e)/(norm(Z)*norm(k6_e)));  % �������ת�����ת��C_te
v_t = 1-cos(C_te); 
% ��ӻ�����ϵZ��任������ĩ��Z����������ת����
R06 = [C(1)*C(1)*v_t+cos(C_te), C(1)*C(2)*v_t-C(3)*sin(C_te), C(1)*C(3)*v_t+C(2)*sin(C_te);
       C(1)*C(2)*v_t+C(3)*sin(C_te), C(2)*C(2)*v_t+cos(C_te), C(2)*C(3)*v_t-C(1)*sin(C_te);
       C(1)*C(3)*v_t-C(2)*sin(C_te), C(2)*C(3)*v_t+C(1)*sin(C_te), C(3)*C(3)*v_t+cos(C_te)];

%     k6 = R06*[0;0;1]; %debugʱ�ɴ�ֵȷ��R06�Ƿ���ȷ
if sum(isnan(C))==3
    R06 = eye(3);
end

%% 3.��·���������ؽڱ���
% ���빫ʽ������ؽڱ���
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