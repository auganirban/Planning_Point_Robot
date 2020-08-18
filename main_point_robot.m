clc;
clear;
close all;

% Make simulation parameters global
global contact_normal; global closeset_dist;   
global safe_dist;      global num_of_contact;
global ln_segs;        global contact_normal_array; 
global ln_bias_array;  global qg;
global qs;             global h;
global kp;             global v;
global q_o;            global mid_pt;
global dof;

% Make maze parameters global
global maze_height; global maze_length; 
global maze_thick; global gap;

% Add PATH solver to matlab's search path
addpath('pathmexa64');
addpath('Point_robot_supporting_files');

% Define parameters
num_of_contact = 1; safe_dist = 0.01;
dof = 2; h = 0.01; kp = 1.95*h; tf = 10;

% Define start and goal configuration
qs = [0.4; 0.9]; 
qg = [5.5; 0.5];

% Define maze parameters
maze_height = 1.0; maze_length = 5; maze_thick = 0.10; gap = 0.5;
[ln_segs, contact_normal_array, ln_bias_array] = surf_norm();

% Draw the maze
drawmaze();
axis('equal');
tic;

% ///////////////////////////////////
%        Start of PATH solver      //
%////////////////////////////////////
num_unknown = dof + num_of_contact;
for i = 1:num_unknown
    l(i) = -Inf;     u(i) = Inf;
    if i == num_unknown
        l(i) = 0;
    end
end

t = 0; q_o = qs; z = zeros(num_unknown, 1); 
total_tm = 0;
qo_array = []; ip_array = []; comp_vel_array = []; time_array = [];
itr = 0; itr_array = [];
while t < tf
    
    dist_qg = (qg - q_o);                      % current error 
    % Termination criterion
    if dist_qg <= 1e-1
        fprintf("Time taken: %2.4f\n", t);
        total_tm = t;
        t = tf;
        continue;
    end
    v = (kp/h)*(dist_qg/norm(dist_qg));        % input vel towards goal
    
    % Compute collision information
    [contact_normal, closeset_dist] = get_colli_info(q_o);
    
    % Solve complementarity problem    
    [z,f,J] = pathmcp(z,l,u,'mcpfuncjacEval');
    fprintf('Collision-free node: \n'); 
    disp(z); fprintf('--------------\n');
    
    % Update
    q_o = z(1:dof);
    t = t + h;
    itr = itr + 1;
    
    % Fill the plotting arrays
    qo_array = [qo_array, q_o];
    ip_array = [ip_array, v];
    comp_vel_array = [comp_vel_array, z(end)*contact_normal];
    time_array = [time_array, t];
    itr_array = [itr_array, itr]; 
    
    % Visualize
    visualize_points(q_o, mid_pt, contact_normal);
    
end
toc;

figure(3)
drawmaze();
h3 = plot(qo_array(1, :), qo_array(2, :), 'r--');
m1 = plot(qs(1), qs(2), 'o', 'MarkerFaceColor', 'r');
m2 = plot(qg(1), qg(2), 'o', 'MarkerFaceColor', 'g');
legend([h3, m1, m2],{'configuration trajectory', 'Start', 'Goal'},'Location','northwest','NumColumns', 3)

figure(2)
subplot(3, 1, 1)
drawmaze();
h3 = plot(qo_array(1, :), qo_array(2, :), 'r--');
m1 = plot(qs(1), qs(2), 'o', 'MarkerFaceColor', 'r');
m2 = plot(qg(1), qg(2), 'o', 'MarkerFaceColor', 'g');
legend([h3, m1, m2],{'configuration trajectory', 'Start', 'Goal'},'Location','northwest','NumColumns', 3)

subplot(3, 1, 2)
plot(itr_array, ip_array(1,:), itr_array, ip_array(2,:))
xlabel("iterations"); ylabel("$$\dot{\gamma}$$", 'Interpreter','latex');
legend({'v_x', 'v_y'}, 'Orientation', 'horizontal');
grid on

subplot(3, 1, 3)
plot(itr_array, comp_vel_array(1,:), itr_array, comp_vel_array(2,:))
xlabel("iterations"); ylabel("v_c");
legend({'v_{c_x}', 'v_{c_y}'}, 'Orientation', 'horizontal');
grid on