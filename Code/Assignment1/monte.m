function flag = monte(x, y, z)
    a1 = 0.5; d3max = 0.3; d6 = 0.2;    % 机械臂参数
    right = 0; wrong = 0; res = []; % 是否存在逆运动学解计数
    for a = 0:0.2*pi*pi       % 操作臂姿态
        for b = 0:0.2*pi:pi
           for c = 0:0.2*pi:pi
              R = [cos(a)*cos(b) cos(a)*sin(b)*sin(c)-sin(a)*cos(c) cos(a)*sin(b)*cos(c)+sin(a)*sin(c);
                   sin(a)*cos(b) sin(a)*sin(b)*sin(c)+cos(a)*cos(c) sin(a)*sin(b)*cos(c)-cos(a)*sin(c);
                   -sin(b) cos(b)*sin(c) cos(b)*cos(c)]; % 旋转矩阵
              ax = R(1, 3); ay = R(2, 3); az = R(3, 3);  % 齐次变换矩阵的相关项
              ox = R(1, 2); oy = R(2, 2); oz = R(3, 2);
              px = x - ax*d6; py = y - ay*d6; pz = z - az*d6;   % 第六个关节的位置
              % 逆运动学求解
              t1 = atan2(py, px);
              t2 = atan2(cos(t1)*px+sin(t1)*py-a1, pz);
              d3 = cos(t1)*sin(t2)*px + sin(t1)*sin(t2)*py + cos(t2)*pz - a1*sin(t2);
              if d3 > d3max     % 不存在逆运动学解
                  wrong = wrong + 1;
                  continue;
              end
              t4 = atan2(-sin(t1)*ax + cos(t1)*ay, cos(t2)*(cos(t1)*ax+sin(t1)*ay) - sin(t2)*az);

              s5 = (cos(t1)*cos(t2)*cos(t4)-sin(t1)*sin(t4))*ax + (cos(t1)*sin(t4)+sin(t1)*cos(t2)*cos(t4))*ay - sin(t2)*cos(t4)*az;
              c5 = cos(t1)*sin(t2)*ax + sin(t1)*sin(t2)*ay + cos(t2)*az;
              t5 = atan2(s5, c5);

              s6 = (cos(t1)*(sin(t2)*sin(t5)-cos(t2)*cos(t4)*cos(t5))+sin(t1)*sin(t4)*cos(t5))*ox ...
                  + (sin(t1)*(sin(t2)*sin(t5)-cos(t2)*cos(t4)*cos(t5))-cos(t1)*sin(t4)*cos(t5))*oy ...
                  + (cos(t2)*sin(t5)+sin(t2)*cos(t4)*cos(t5))*oz;
              c6 = -(sin(t1)*cos(t4)+cos(t1)*cos(t2)*sin(t4))*ox + (cos(t1)*cos(t4)-sin(t1)*cos(t2)*sin(t4))*oy + sin(t2)*sin(t4)*oz;
              t6 = atan2(s6, c6);
              % 确定求得的解是否为有效逆运动学解
              temp = rad2deg([t1 t2 t4 t5 t6]);
              temp = [temp d3];
              res = [res; temp];
              if isfinite(temp)
                  right = right + 1;
              else
                  wrong = wrong + 1;
              end
           end
        end
    end
    if ~wrong
        flag = 1;
    else
        flag = 0;
    end
end