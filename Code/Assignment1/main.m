clear; clc;
a = -1; b = 1; % ����[-1, 1]�������
num = 0; % ���ɹ����ռ��ڵ���
for i = 1:100000
    r = a + (b-a).*rand(3, 1);
    x = r(1); y = r(2); z = r(3);
    flag = monte(x, y, z);
    if flag
        num = num + 1;
        plot3(x, y, z, 'r.');
        hold on; grid on; axis equal;
        xlabel('x'); ylabel('y'); zlabel('z');
    end
end
disp(num);