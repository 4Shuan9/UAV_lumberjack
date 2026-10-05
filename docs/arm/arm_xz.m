%% 清屏
clc; % 清空命令行窗口
clear; % 清空工作区变量
close all; % 关闭所有打开的图形窗口
% % 时间参数
% t_start = 0;  
% t_end = 7;
% h = 0.01;
% t = t_start:h:t_end;
% N = length(t);

% 机械臂连杆长度
L0 = 0.11; % 无人机质心(O0)-机械臂第一关节(O1)
L1 = 0.18; % 机械臂第一关节(O1)-机械臂第二关节(O2)-大臂长度
L2 = 0.17; % 机械臂第二关节(O2)-机械臂第三关节(O3)-小臂长度
L3 = 0.025; % 机械臂第三关节(O3)-末端执行器连接处(O4)
L4 = 0.05; % 机械臂第四关节(O4)-末端执行器中心(O5)
% 关节角
% theta1=-80;theta2=70;theta4=0; % 折叠
theta1=0;
theta2=0;
theta4=0;
% theta1=-10;
% theta2=10;
% theta4=30;
% 目标点
% P_target=[0;0;0];
%% 期望位姿
phi=0;theta=0;psi=0;

Tb4_target = [  0,0,0,0;
                0,0,0,0;
                0,0,0,0;
                0,0,0,1];

P_target=[0.2;0;-0.5]; % 目标点

Tb4_target(1:3,4)=P_target;
hh=0.001;
% for phi = -pi:hh:pi
    for theta = -pi:hh:pi
        % for psi = -pi/2:hh:pi/2
            Tb4_target(1:3,1:3)=RxRyRz(phi,theta,psi);% ° -> rad
            % P_target=[0;-0.4;-1];
            Tb4_target(1:3,4)=P_target;
            [theta1, theta2, theta4, flag] = inv_kin_b(Tb4_target, L0, L1, L2, L3, L4);
            if flag==1
                break;
            end
        % end
    end
% end

% [theta1, theta2, theta4, flag] = inv_kin_b(Tb4_target, L0, L1, L2, L3);

theta1 = theta1*180/pi% °
theta2 = theta2*180/pi% ° 
theta4 = theta4*180/pi% °
%% 正运动学(通过关节角推算机械臂的姿态)
Ob=[0;0;0;1];%基坐标系

Ry=[cos(pi/2),0,sin(pi/2),0;
    0,1,0,0;
    -sin(pi/2),0,cos(pi/2),0;
    0,0,0,1;]; 
T_b0=Ry*Matrix_T_mn(pi/2,0,0,0);
T_01=Matrix_T_mn(0 *pi/180,L0,theta1 *pi/180,0); % ° -> rad
% T_12=Matrix_T_mn(0 *pi/180,L1,(-90+theta2) *pi/180,0);% 只求正运动用这一行
T_12=Matrix_T_mn(0 *pi/180,L1,(theta2) *pi/180,0);%求逆运动时，角度已经包含-90
T_23=Matrix_T_mn(-90 *pi/180,0,0,L2);
T_34=Matrix_T_mn(0,0,theta4 *pi/180,L3);
T_45=Matrix_T_mn(0,0,0,L4);

T_b1=T_b0*T_01;
T_b2=T_b0*T_01*T_12;
T_b3=T_b0*T_01*T_12*T_23;
T_b4=T_b0*T_01*T_12*T_23*T_34;
T_b5=T_b0*T_01*T_12*T_23*T_34*T_45;

O0=T_b0*[0;0;0;1];
O1=T_b1*[0;0;0;1];
O2=T_b2*[0;0;0;1];
O3=T_b3*[0;0;0;1];
O4=T_b4*[0;0;0;1];
O5=T_b5*[0;0;0;1];
%% 绘图
figure;
plot3(P_target(1),P_target(2),P_target(3), 'ko', 'LineWidth', 2);
hold on;
draw_frame(Matrix_T_mn(0,0,0,0),Ob,'Ob'); % Matrix_T_mn(alpha,a,theta,d);

draw_frame(T_b0,O0,'O0');
draw_frame(T_b1,O1,'O1');
draw_frame(T_b2,O2,'O2');
draw_frame(T_b3,O3,'O3');
draw_frame(T_b4,O4,'O4');
draw_frame(T_b5,O5,'O5');

hold off;
axis on;     %'on'显示坐标轴  %'off'隐藏坐标轴%
axis equal;
box on;      % 'on'/'off'是否显示边框
grid on;  % 'on'/'off'是否保留网格
xlabel('x(m)'); ylabel('y(m)');zlabel('z(m)');

function draw_frame(T,p,label)
    scale = 0.2; % 坐标轴箭头长度
    R = T(1:3, 1:3);
    
    % 绘制X轴 (红色)
    quiver3(p(1), p(2), p(3), R(1,1)*scale, R(2,1)*scale, R(3,1)*scale, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);hold on;
    % 绘制Y轴 (绿色)
    quiver3(p(1), p(2), p(3), R(1,2)*scale, R(2,2)*scale, R(3,2)*scale, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    % 绘制Z轴 (蓝色)
    quiver3(p(1), p(2), p(3), R(1,3)*scale, R(2,3)*scale, R(3,3)*scale, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % 添加文字标注
    text(p(1), p(2), p(3), [' ' label], 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');
end 
%% 求逆解
function [theta1, theta2, theta4, flag] = inv_kin_b(Tb4_target, L0, L1, L2, L3, L4)
    flag = 1;
    K = L2+L3+L4;
    % 提取期望位姿
    X_be1 = Tb4_target(1,1); Y_be1 = Tb4_target(1,2); Z_be1 = Tb4_target(1,3);
    X_be2 = Tb4_target(2,1); Y_be2 = Tb4_target(2,2); Z_be2 = Tb4_target(2,3);
    X_be3 = Tb4_target(3,1); Y_be3 = Tb4_target(3,2); Z_be3 = Tb4_target(3,3);
    P_x = Tb4_target(1,4); P_y = Tb4_target(2,4); P_z = Tb4_target(3,4);
    
    % 机构约束校验 n'_x必须等于0
    if abs(Z_be2) > 1e-6
        disp('目标位姿不满足机构约束，无解');
        theta1 = 0;
        theta2 = 0;
        theta4 = 0;
        flag=0; return;
    end
    
    theta4 = atan2(X_be1, Y_be1);
    phi = atan2(Z_be3, Z_be1);
    
    A = P_x - K*Z_be1;
    B = K*Z_be3 -P_z - L0;
    
    if abs(A^2+B^2 - L1^2) > 1e-2 % 误差范围
        disp('目标位置超出工作空间，无解');
        theta1 = 0;
        theta2 = 0;
        flag=0; return;
    end
    
    theta1 = atan2(A,B);
    theta2 = phi - theta1;
    disp('有解');
    flag=1;
end
%% 矩阵运算
function R=RxRyRz(phi,theta,psi)
    Rx = [1,0,0;
          0,cos(phi),-sin(phi);
          0,sin(phi),cos(phi)];
    Ry=[cos(theta),0,sin(theta);
        0,1,0;
        -sin(theta),0,cos(theta)]; 
    Rz = [cos(psi),-sin(psi),0;
          sin(psi),cos(psi),0;
          0,0,1];
    % R=Rz*Ry*Rx;
    R=Rx*Ry*Rz;
end

function T_mn = Matrix_T_mn(alpha,a,theta,d)
    T_mn=Matrix_Rx(alpha)*Matrix_Tx(a)*Matrix_Rz(theta)*Matrix_Tz(d);
end

function [Rx] = Matrix_Rx(alpha)
    Rx = [1,0,0,0;
          0,cos(alpha),-sin(alpha),0;
          0,sin(alpha),cos(alpha),0;
          0,0,0,1;]; 
end

function [Tx] = Matrix_Tx(a)
    Tx = [1,0,0,a;
          0,1,0,0;
          0,0,1,0;
          0,0,0,1;]; 
end

function [Rz] = Matrix_Rz(theta)
    Rz = [cos(theta),-sin(theta),0,0;
          sin(theta),cos(theta),0,0;
          0,0,1,0;
          0,0,0,1;]; 
end

function [Tz] = Matrix_Tz(d)
    Tz = [1,0,0,0;
          0,1,0,0;
          0,0,1,d;
          0,0,0,1;]; 
end