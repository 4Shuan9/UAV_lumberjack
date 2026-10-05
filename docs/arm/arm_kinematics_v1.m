%% UAV_lumberjack current 3-DOF arm kinematics V1
% Reference model derived from current x500_lumberjack/model.sdf.
% All joint angles are LOGICAL angles in degrees.
% V1 TCP = chainsaw_body origin.

clc; clear; close all;

%% Known poses for quick verification
Q = [
   -30, -150, -90;   % HOME
   -60,   60,   0;   % PREWORK
   -45,  -45,   0;   % CUT_A
   -30,  -60,   0    % CUT_B
];

names = {"HOME", "PREWORK", "CUT_A", "CUT_B"};

for i = 1:size(Q,1)
    q = deg2rad(Q(i,:));
    T = arm_fk(q);
    fprintf('\n[%s]\n', names{i});
    fprintf('q = [%.1f, %.1f, %.1f] deg\n', Q(i,1), Q(i,2), Q(i,3));
    fprintf('TCP = [%.6f, %.6f, %.6f] m\n', T(1,4), T(2,4), T(3,4));

    sols = arm_ik_pose(T);
    fprintf('Pose IK candidates: %d\n', size(sols,1));
    if ~isempty(sols)
        disp(rad2deg(sols));
    end
end

%% Position-only IK example: PREWORK TCP
T_pre = arm_fk(deg2rad([-60, 60, 0]));
p_target = T_pre(1:3,4);
sols = arm_ik_position(p_target);
fprintf('\n[PREWORK position-only IK]\n');
disp(rad2deg(sols));

%% ---------------- Local functions ----------------
function T = arm_fk(q)
    % q = [q2 q3 q4] logical radians.
    q2 = q(1); q3 = q(2); q4 = q(3);

    z_j2 = -0.110;
    L1 = 0.180;
    L2 = 0.170;
    j4_to_wrist = 0.025;
    saw_offset = [0.060; 0.030; 0.0];

    R_upper = Ry(-q2);
    R_forearm = Ry(-(q2 + q3));
    R_wrist = R_forearm * Rx(q4);

    p = [0;0;z_j2];
    p = p + R_upper * [L1;0;0];
    p = p + R_forearm * [L2 + j4_to_wrist;0;0];
    p = p + R_wrist * saw_offset;

    R_tcp = R_forearm * Rx(q4 + pi/2);
    T = [R_tcp, p; 0 0 0 1];
end

function sols = arm_ik_position(p_target)
    % Position-only IK for V1 TCP. Returns geometric candidates in radians.
    z_j2 = -0.110;
    L1 = 0.180;
    L2 = 0.170;
    j4_to_wrist = 0.025;
    sx = 0.060;
    sy = 0.030;

    x = p_target(1);
    y = p_target(2);
    z = p_target(3) - z_j2;

    sols = [];
    tol = 1e-9;

    if abs(y) > sy + tol
        return;
    end

    cq4 = max(-1,min(1,y/sy));
    q4_abs = acos(cq4);
    q4_list = unique_tol([q4_abs, -q4_abs]);

    fixed_x = L2 + j4_to_wrist + sx;

    for q4 = q4_list
        second_z = sy * sin(q4);
        Lb = hypot(fixed_x, second_z);
        delta = atan2(second_z, fixed_x);

        c_elbow = (x^2 + z^2 - L1^2 - Lb^2)/(2*L1*Lb);
        if c_elbow < -1-tol || c_elbow > 1+tol
            continue;
        end
        c_elbow = max(-1,min(1,c_elbow));
        e_abs = acos(c_elbow);
        e_list = unique_tol([e_abs, -e_abs]);

        for e = e_list
            q2 = atan2(z,x) - atan2(Lb*sin(e), L1 + Lb*cos(e));
            q3 = e - delta;
            q = wrap_pi([q2, q3, q4]);

            T = arm_fk(q);
            if norm(T(1:3,4)-p_target) < 1e-7
                sols = add_unique_solution(sols, q);
            end
        end
    end
end

function sols = arm_ik_pose(T_target)
    % Exact pose IK for the 3-DOF mechanism.
    % Reachable orientations satisfy R = Ry(tool_pitch)*Rx(tool_roll).
    z_j2 = -0.110;
    L1 = 0.180;
    L2 = 0.170;
    j4_to_wrist = 0.025;
    saw_offset = [0.060;0.030;0];

    R = T_target(1:3,1:3);
    p = T_target(1:3,4);

    tool_pitch = atan2(-R(3,1), R(1,1));
    tool_roll = atan2(-R(2,3), R(2,2));

    R_check = Ry(tool_pitch)*Rx(tool_roll);
    if rotation_error(R,R_check) > 1e-7
        sols = [];
        return;
    end

    q23 = -tool_pitch;
    q4 = wrap_pi(tool_roll - pi/2);

    R_forearm = Ry(-q23);
    R_wrist = R_forearm*Rx(q4);

    known = R_forearm*[L2+j4_to_wrist;0;0] + R_wrist*saw_offset;
    u = p - [0;0;z_j2] - known;

    if abs(u(2)) > 1e-7 || abs(norm(u)-L1) > 1e-7
        sols = [];
        return;
    end

    q2 = atan2(u(3),u(1));
    q3 = q23-q2;
    q = wrap_pi([q2,q3,q4]);

    T_fk = arm_fk(q);
    if norm(T_fk(1:3,4)-p) < 1e-7 && rotation_error(T_fk(1:3,1:3),R) < 1e-7
        sols = q;
    else
        sols = [];
    end
end

function ok = within_limits(q)
    qdeg = rad2deg(q);
    ok = qdeg(1)>=-165 && qdeg(1)<=15 && ...
         qdeg(2)>=-150 && qdeg(2)<=150 && ...
         qdeg(3)>=-180 && qdeg(3)<=180;
end

function R = Rx(a)
    R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function R = Ry(a)
    R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function q = wrap_pi(q)
    q = mod(q + pi, 2*pi) - pi;
end

function e = rotation_error(Ra,Rb)
    c = (trace(Ra'*Rb)-1)/2;
    c = max(-1,min(1,c));
    e = acos(c);
end

function out = unique_tol(v)
    out = [];
    for x = v
        if isempty(out) || all(abs(out-x)>1e-10)
            out(end+1) = x; %#ok<AGROW>
        end
    end
end

function sols = add_unique_solution(sols,q)
    for i=1:size(sols,1)
        if norm(wrap_pi(sols(i,:)-q)) < 1e-8
            return;
        end
    end
    sols(end+1,:) = q; %#ok<AGROW>
end
