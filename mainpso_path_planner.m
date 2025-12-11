%% =========================================================================
% 无人机三维路径规划主程序（论文终极完善版：修复图例+增加详细统计）
% 2025-12-11 优化内容：
% 1. 三维图增加完整图例（算法线型+起终点）。
% 2. 二维图修复图例显示（data1/2 -> 起点/终点）。
% 3. 统计结果增加 Best/Mean/Std 指标，对齐论文标准。
% 4. 保持 IPSO 的核心改进逻辑（防复数 + 随机扰动）。
% =========================================================================
clear; clc; close all;
fprintf('==============================================\n');
fprintf('   无人机三维路径规划（论文终极完善版）\n');
fprintf('   对比算法：PSO+IPSO+SPSO+GWO | 迭代1000次 \n');
fprintf('==============================================\n\n');

%% 1. 加载环境数据
fprintf('🔧 加载自定义环境...\n');
try
    if exist('complex_3d_environment.mat', 'file')
        load('complex_3d_environment.mat');
    else
        error('找不到 complex_3d_environment.mat 文件！请确认文件路径。');
    end
    
    [grid_x, grid_y, grid_z] = size(environment.occupancy_map);
    env.resolution = 2;          
    env.map_size = grid_x * env.resolution;          
    env.max_height = grid_z * env.resolution;        
    
    env.start = environment.start_point_phys;  
    env.goal = environment.goal_point_phys;    
    env.occupancy_map = environment.occupancy_map;  
    
    building_count = length(building_list);
    buildings = zeros(building_count, 7);
    for i = 1:building_count
        b = building_list(i);
        buildings(i, :) = [double(b.x_phys), double(b.y_phys), double(b.z_phys), ...
                          double(b.width), double(b.length), double(b.height), ...
                          double(20.0)]; 
    end
    fprintf('✅ 环境加载完成！含 %d 个障碍物\n\n', building_count);
catch ME
    error('加载环境失败！\n错误信息：%s', ME.message);
end

%% 2. 算法参数初始化
params.pop_size = 100;          
params.max_iter = 1000;         
% PSO
params.pso.w_max = 0.9; params.pso.w_min = 0.4;
params.pso.c1_max = 2.0; params.pso.c1_min = 1.0;
params.pso.c2_max = 2.0; params.pso.c2_min = 1.0;
% IPSO (改进参数)
params.ipso.w_max = 0.9; params.ipso.w_min = 0.4;
params.ipso.c1_max = 2.5; params.ipso.c1_min = 1.0; 
params.ipso.c2_max = 2.5; params.ipso.c2_min = 1.0;
params.ipso.acc_threshold = 5; 
params.ipso.coll_penalty = 10000;  
% SPSO
params.spso.w_max = 0.9; params.spso.w_min = 0.4;
params.spso.c1_init = 2.5; params.spso.c2_init = 0.5;
% GWO
params.gwo.alpha_init = 2;
% 通用
params.path_points = 15;        
params.v_max = 12;               
params.exp_num = 10;            
params.safety_dist = 20;        
params.alpha1 = 0.5;            
params.alpha2 = 0.3;            
params.alpha3 = 0.2;            

%% 3. 初始化结果存储矩阵
result = struct();
% 存储每次实验的 fitness 和 length，用于计算 Best/Mean/Std
result.pso_fits = zeros(params.exp_num, 1);
result.ipso_fits = zeros(params.exp_num, 1);
result.spso_fits = zeros(params.exp_num, 1);
result.gwo_fits = zeros(params.exp_num, 1);

result.pso_lens = zeros(params.exp_num, 1);
result.ipso_lens = zeros(params.exp_num, 1);
result.spso_lens = zeros(params.exp_num, 1);
result.gwo_lens = zeros(params.exp_num, 1);

% 存储收敛代数
result.pso_conv = zeros(params.exp_num, 1);
result.ipso_conv = zeros(params.exp_num, 1);
result.spso_conv = zeros(params.exp_num, 1);
result.gwo_conv = zeros(params.exp_num, 1);

conv_curves = struct();

%% 4. 运行10次重复实验
fprintf('🚀 开始运行4种算法...\n');
for exp_idx = 1:params.exp_num
    fprintf('===== 第%d/%d次实验 =====\n', exp_idx, params.exp_num);
    
    % 4.1 PSO
    fprintf('   正在运行 PSO...\n');
    [pso_fit, pso_path, pso_len, pso_cc] = PSO_3D(params, env, buildings);
    result.pso_fits(exp_idx) = pso_fit;
    result.pso_lens(exp_idx) = pso_len;
    result.pso_conv(exp_idx) = find(pso_cc < min(pso_cc)*1.01, 1, 'first');
    
    % 4.2 IPSO
    fprintf('   正在运行 IPSO...\n');
    [ipso_fit, ipso_path, ipso_len, ipso_cc] = IPSO_3D(params, env, buildings);
    result.ipso_fits(exp_idx) = ipso_fit;
    result.ipso_lens(exp_idx) = ipso_len;
    result.ipso_conv(exp_idx) = find(ipso_cc < min(ipso_cc)*1.01, 1, 'first');
    
    % 4.3 SPSO
    fprintf('   正在运行 SPSO...\n');
    [spso_fit, spso_path, spso_len, spso_cc] = SPSO_3D(params, env, buildings);
    result.spso_fits(exp_idx) = spso_fit;
    result.spso_lens(exp_idx) = spso_len;
    result.spso_conv(exp_idx) = find(spso_cc < min(spso_cc)*1.01, 1, 'first');
    
    % 4.4 GWO
    fprintf('   正在运行 GWO...\n');
    [gwo_fit, gwo_path, gwo_len, gwo_cc] = GWO_3D(params, env, buildings);
    result.gwo_fits(exp_idx) = gwo_fit;
    result.gwo_lens(exp_idx) = gwo_len;
    result.gwo_conv(exp_idx) = find(gwo_cc < min(gwo_cc)*1.01, 1, 'first');
    
    % 存储最后1次实验数据用于绘图
    if exp_idx == params.exp_num
        conv_curves.pso = pso_cc;
        conv_curves.ipso = ipso_cc;
        conv_curves.spso = spso_cc;
        conv_curves.gwo = gwo_cc;
        best_paths = struct('pso', pso_path, 'ipso', ipso_path, 'spso', spso_path, 'gwo', gwo_path);
    end
    
    % 兜底
    if isempty(result.pso_conv(exp_idx)), result.pso_conv(exp_idx) = params.max_iter; end
    if isempty(result.ipso_conv(exp_idx)), result.ipso_conv(exp_idx) = params.max_iter; end
    if isempty(result.spso_conv(exp_idx)), result.spso_conv(exp_idx) = params.max_iter; end
    if isempty(result.gwo_conv(exp_idx)), result.gwo_conv(exp_idx) = params.max_iter; end
end

%% 5. 计算统计指标 (Best, Mean, Std)
stats = struct();
% 路径长度
stats.len.best = [min(result.pso_lens), min(result.ipso_lens), min(result.spso_lens), min(result.gwo_lens)];
stats.len.mean = [mean(result.pso_lens), mean(result.ipso_lens), mean(result.spso_lens), mean(result.gwo_lens)];
stats.len.std  = [std(result.pso_lens), std(result.ipso_lens), std(result.spso_lens), std(result.gwo_lens)];
% 适应度
stats.fit.best = [min(result.pso_fits), min(result.ipso_fits), min(result.spso_fits), min(result.gwo_fits)];
stats.fit.mean = [mean(result.pso_fits), mean(result.ipso_fits), mean(result.spso_fits), mean(result.gwo_fits)];
stats.fit.std  = [std(result.pso_fits), std(result.ipso_fits), std(result.spso_fits), std(result.gwo_fits)];

%% 6. 输出统计结果到命令行
fprintf('\n=================================================================================\n');
fprintf('                           10次实验统计结果 (Best/Mean/Std)                      \n');
fprintf('=================================================================================\n');
fprintf('指标       | PSO               | IPSO              | SPSO              | GWO\n');
fprintf('-----------|-------------------|-------------------|-------------------|-------------------\n');
fprintf('长度 Best  | %.2f            | %.2f            | %.2f            | %.2f\n', stats.len.best);
fprintf('长度 Mean  | %.2f            | %.2f            | %.2f            | %.2f\n', stats.len.mean);
fprintf('长度 Std   | %.2f              | %.2f              | %.2f              | %.2f\n', stats.len.std);
fprintf('-----------|-------------------|-------------------|-------------------|-------------------\n');
fprintf('适应度 Best| %.2f            | %.2f            | %.2f            | %.2f\n', stats.fit.best);
fprintf('适应度 Mean| %.2f            | %.2f            | %.2f            | %.2f\n', stats.fit.mean);
fprintf('适应度 Std | %.2f              | %.2f              | %.2f              | %.2f\n', stats.fit.std);
fprintf('=================================================================================\n');

%% 7. 生成图表
generate_figure_literature(best_paths, conv_curves, buildings, env, stats);

%% =========================================================================
% 下面是功能函数（保持逻辑修复版）
% =========================================================================

function [gx, gy, gz] = phys2grid(phys_pos, env)
    phys_pos = real(phys_pos); 
    gx = round(phys_pos(1) / env.resolution);
    gy = round(phys_pos(2) / env.resolution);
    gz = round(phys_pos(3) / env.resolution);
    gx = max(1, min(gx, size(env.occupancy_map, 1)));
    gy = max(1, min(gy, size(env.occupancy_map, 2)));
    gz = max(1, min(gz, size(env.occupancy_map, 3)));
end

function collide = is_path_collide(path_3d, buildings, safety_dist)
    collide = false;
    for i = 1:size(path_3d, 1)
        for j = 1:size(buildings, 1)
            dist = calculate_point_building_distance(path_3d(i,:), buildings(j,:));
            if dist < safety_dist
                collide = true;
                return;
            end
        end
    end
end

function dist = calculate_point_building_distance(point, building)
    point = real(point); 
    cx = building(1); cy = building(2); cz = building(3);
    w = building(4); l = building(5); h = building(6);
    dx = max(abs(point(1)-cx) - w/2, 0);
    dy = max(abs(point(2)-cy) - l/2, 0);
    dz = max(point(3) - (cz + h), 0);
    dist = sqrt(dx^2 + dy^2 + dz^2);
end

function [fitness, path_len, collide_flag] = calculate_fitness(path, env, buildings, params)
    path = real(path); 
    path_3d = reshape(path, 3, [])';
    len_cost = 0;
    threat_cost = 0;
    height_cost = 0;
    collide_flag = false;
    
    for i = 2:size(path_3d, 1)
        len_cost = len_cost + sqrt(sum((path_3d(i,:)-path_3d(i-1,:)).^2));
    end
    
    for i = 1:size(path_3d, 1)
        [gx, gy, gz] = phys2grid(path_3d(i,:), env);
        if env.occupancy_map(gx, gy, gz) == 1
            threat_cost = threat_cost + params.ipso.coll_penalty; 
            collide_flag = true;
        end
        for j = 1:size(buildings, 1)
            dist = calculate_point_building_distance(path_3d(i,:), buildings(j,:));
            if dist < params.safety_dist
                threat_cost = threat_cost + (params.safety_dist - dist)^2 * 2;
            end
        end
    end
    
    for i = 2:size(path_3d, 1)
        height_cost = height_cost + abs(path_3d(i,3) - path_3d(i-1,3));
    end
    
    fitness = params.alpha1 * threat_cost + params.alpha2 * len_cost + params.alpha3 * height_cost;
    path_len = len_cost;
end

function [best_fitness, best_path, best_len, conv_curve] = PSO_3D(params, env, buildings)
    pop_size = params.pop_size;
    max_iter = params.max_iter;
    path_dim = 3 * params.path_points;
    pop = zeros(pop_size, path_dim);
    for i = 1:pop_size
        try_count = 0; 
        while true
            try_count = try_count + 1;
            pop(i, 1:3) = env.start;
            for j = 4:3:(path_dim-3)
                pop(i,j) = env.start(1) + rand()*(env.goal(1)-env.start(1));
                pop(i,j+1) = env.start(2) + rand()*(env.goal(2)-env.start(2));
                pop(i,j+2) = 20 + rand()*(env.max_height-40);
            end
            pop(i, end-2:end) = env.goal;
            if try_count > 50, break; end 
            path_3d = reshape(pop(i,:), 3, [])';
            if ~is_path_collide(path_3d, buildings, params.safety_dist), break; end
        end
    end
    v = rand(pop_size, path_dim) * params.v_max;
    pbest = pop; pbest_fit = ones(pop_size, 1) * inf;
    gbest = pop(1,:); gbest_fit = inf;
    conv_curve = zeros(max_iter, 1);
    for iter = 1:max_iter
        w = params.pso.w_max - (params.pso.w_max - params.pso.w_min)*iter/max_iter;
        c1 = params.pso.c1_max - (params.pso.c1_max - params.pso.c1_min)*iter/max_iter;
        c2 = params.pso.c2_min + (params.pso.c2_max - params.pso.c2_min)*iter/max_iter;
        for i = 1:pop_size
            [fit, len, ~] = calculate_fitness(pop(i,:), env, buildings, params);
            if fit < pbest_fit(i), pbest_fit(i) = fit; pbest(i,:) = pop(i,:); end
            if fit < gbest_fit, gbest_fit = fit; gbest = pop(i,:); best_len = len; end
            v(i,:) = w*v(i,:) + c1*rand*(pbest(i,:)-pop(i,:)) + c2*rand*(gbest-pop(i,:));
            v(i,:) = max(min(v(i,:), params.v_max), -params.v_max);
            pop(i,:) = pop(i,:) + v(i,:);
            pop(i, 1:3:end) = max(env.start(1), min(pop(i,1:3:end), env.goal(1)));
            pop(i, 2:3:end) = max(env.start(2), min(pop(i,2:3:end), env.goal(2)));
            pop(i, 3:3:end) = max(20, min(pop(i,3:3:end), env.max_height-20));
            pop(i, 1:3) = env.start; pop(i, end-2:end) = env.goal;
        end
        conv_curve(iter) = gbest_fit;
    end
    best_fitness = gbest_fit; best_path = gbest;
end

function [best_fitness, best_path, best_len, conv_curve] = IPSO_3D(params, env, buildings)
    pop_size = params.pop_size;
    max_iter = params.max_iter;
    path_dim = 3 * params.path_points;
    pop = zeros(pop_size, path_dim);
    for i = 1:pop_size
        try_count = 0; 
        while true
            try_count = try_count + 1;
            pop(i, 1:3) = env.start;
            x0 = rand();
            for j = 4:3:(path_dim-3)
                x0 = 4*x0*(1-x0);
                S = (1/pi)*asin(2*x0-1) + 0.5; 
                x0 = 4*S*(1-S);
                S_prime = (1/pi)*asin(2*x0-1) + 0.5; 
                pop(i,j) = env.start(1) + (env.goal(1)-env.start(1))*S_prime;
                pop(i,j+1) = env.start(2) + (env.goal(2)-env.start(2))*S_prime;
                pop(i,j+2) = 20 + (env.max_height-40)*S_prime;
            end
            pop(i, end-2:end) = env.goal;
            if try_count > 50, break; end 
            path_3d = reshape(pop(i,:), 3, [])';
            if ~is_path_collide(path_3d, buildings, params.safety_dist), break; end
        end
    end
    v = rand(pop_size, path_dim) * params.v_max;
    pbest = pop; pbest_fit = ones(pop_size, 1) * inf;
    gbest = pop(1,:); gbest_fit = inf;
    bad_count = zeros(pop_size, 1);
    conv_curve = zeros(max_iter, 1);
    for iter = 1:max_iter
        fit_all = zeros(pop_size, 1);
        for i = 1:pop_size
            [fit_all(i), ~, ~] = calculate_fitness(pop(i,:), env, buildings, params);
        end
        F_avg = mean(fit_all); F_min = min(fit_all); F_max = max(fit_all);
        for i = 1:pop_size
            fit = fit_all(i);
            if fit < pbest_fit(i)
                pbest_fit(i) = fit; pbest(i,:) = pop(i,:); bad_count(i) = 0;
            else
                bad_count(i) = bad_count(i) + 1;
            end
            if fit < gbest_fit
                gbest_fit = fit; gbest = pop(i,:);
                [~, best_len, ~] = calculate_fitness(gbest, env, buildings, params);
            end
            if fit > F_avg, w = params.ipso.w_max;
            else, w = params.ipso.w_min + (params.ipso.w_max - params.ipso.w_min)*(fit - F_min)/(F_avg - F_min + 1e-8); end
            c1 = exp(params.ipso.c1_min + (params.ipso.c1_max - params.ipso.c1_min)/max_iter*iter);
            c2 = exp(params.ipso.c2_min - (params.ipso.c2_max - params.ipso.c2_min)/max_iter*iter);
            a_i = 0;
            if bad_count(i) >= params.ipso.acc_threshold
                a_mag = rand() * (fit - F_max)/(F_avg - F_max + 1e-8) * params.v_max;
                a_i = a_mag * (rand(1, path_dim) - 0.5) * 2;
            else
                a_i = zeros(1, path_dim);
            end
            v(i,:) = w*v(i,:) + c1*rand*(pbest(i,:)-pop(i,:)) + c2*rand*(gbest-pop(i,:)) + a_i;
            v(i,:) = max(min(v(i,:), params.v_max), -params.v_max);
            phi = 1 - atan(iter/(max_iter + 1));
            pop(i,:) = pop(i,:) + v(i,:) * phi;
            pop(i, 1:3:end) = max(env.start(1), min(pop(i,1:3:end), env.goal(1)));
            pop(i, 2:3:end) = max(env.start(2), min(pop(i,2:3:end), env.goal(2)));
            pop(i, 3:3:end) = max(20, min(pop(i,3:3:end), env.max_height-20));
            pop(i, 1:3) = env.start; pop(i, end-2:end) = env.goal;
        end
        conv_curve(iter) = gbest_fit;
    end
    best_fitness = gbest_fit; best_path = gbest;
end

function [best_fitness, best_path, best_len, conv_curve] = SPSO_3D(params, env, buildings)
    pop_size = params.pop_size;
    max_iter = params.max_iter;
    path_dim = 3 * params.path_points;
    pop = zeros(pop_size, path_dim);
    for i = 1:pop_size
        try_count = 0; 
        while true
            try_count = try_count + 1;
            pop(i, 1:3) = env.start;
            for j = 4:3:(path_dim-3)
                pop(i,j) = env.start(1) + rand()*(env.goal(1)-env.start(1));
                pop(i,j+1) = env.start(2) + rand()*(env.goal(2)-env.start(2));
                pop(i,j+2) = 20 + rand()*(env.max_height-40);
            end
            pop(i, end-2:end) = env.goal;
            if try_count > 50, break; end 
            path_3d = reshape(pop(i,:), 3, [])';
            if ~is_path_collide(path_3d, buildings, params.safety_dist), break; end
        end
    end
    v = rand(pop_size, path_dim) * params.v_max;
    pbest = pop; pbest_fit = ones(pop_size, 1) * inf;
    gbest = pop(1,:); gbest_fit = inf;
    conv_curve = zeros(max_iter, 1);
    for iter = 1:max_iter
        w = params.spso.w_max - (params.spso.w_max - params.spso.w_min)*iter/max_iter;
        c1 = max(params.spso.c1_init - 2*iter/max_iter, 0.5);
        c2 = min(params.spso.c2_init + 2*iter/max_iter, 2.5);
        for i = 1:pop_size
            [fit, len, ~] = calculate_fitness(pop(i,:), env, buildings, params);
            if fit < pbest_fit(i), pbest_fit(i) = fit; pbest(i,:) = pop(i,:); end
            if fit < gbest_fit, gbest_fit = fit; gbest = pop(i,:); best_len = len; end
            v(i,:) = w*v(i,:) + c1*rand*(pbest(i,:)-pop(i,:)) + c2*rand*(gbest-pop(i,:));
            v(i,:) = max(min(v(i,:), params.v_max), -params.v_max);
            pop(i,:) = pop(i,:) + v(i,:);
            pop(i, 1:3:end) = max(env.start(1), min(pop(i,1:3:end), env.goal(1)));
            pop(i, 2:3:end) = max(env.start(2), min(pop(i,2:3:end), env.goal(2)));
            pop(i, 3:3:end) = max(20, min(pop(i,3:3:end), env.max_height-20));
            pop(i, 1:3) = env.start; pop(i, end-2:end) = env.goal;
        end
        conv_curve(iter) = gbest_fit;
    end
    best_fitness = gbest_fit; best_path = gbest;
end

function [best_fitness, best_path, best_len, conv_curve] = GWO_3D(params, env, buildings)
    pop_size = params.pop_size;
    max_iter = params.max_iter;
    path_dim = 3 * params.path_points;
    pop = zeros(pop_size, path_dim);
    for i = 1:pop_size
        try_count = 0; 
        while true
            try_count = try_count + 1;
            pop(i, 1:3) = env.start;
            for j = 4:3:(path_dim-3)
                pop(i,j) = env.start(1) + rand()*(env.goal(1)-env.start(1));
                pop(i,j+1) = env.start(2) + rand()*(env.goal(2)-env.start(2));
                pop(i,j+2) = 20 + rand()*(env.max_height-40);
            end
            pop(i, end-2:end) = env.goal;
            if try_count > 50, break; end 
            path_3d = reshape(pop(i,:), 3, [])';
            if ~is_path_collide(path_3d, buildings, params.safety_dist), break; end
        end
    end
    fit_all = zeros(pop_size, 1);
    len_all = zeros(pop_size, 1);
    for i = 1:pop_size
        [fit_all(i), len_all(i), ~] = calculate_fitness(pop(i,:), env, buildings, params);
    end
    [~, idx] = sort(fit_all);
    alpha = pop(idx(1),:); alpha_fit = fit_all(idx(1)); alpha_len = len_all(idx(1));
    beta = pop(idx(2),:); beta_fit = fit_all(idx(2));
    delta = pop(idx(3),:); delta_fit = fit_all(idx(3));
    conv_curve = zeros(max_iter, 1);
    for iter = 1:max_iter
        a = params.gwo.alpha_init - 2*iter/max_iter;
        for i = 1:pop_size
            for j = 1:path_dim
                r1 = rand(); r2 = rand();
                A1 = 2*a*r1 - a; C1 = 2*r2;
                D_alpha = abs(C1*alpha(j) - pop(i,j)) * 1.2;
                X1 = alpha(j) - A1*D_alpha;
                
                r1 = rand(); r2 = rand();
                A2 = 2*a*r1 - a; C2 = 2*r2;
                D_beta = abs(C2*beta(j) - pop(i,j)) * 1.2;
                X2 = beta(j) - A2*D_beta;
                
                r1 = rand(); r2 = rand();
                A3 = 2*a*r1 - a; C3 = 2*r2;
                D_delta = abs(C3*delta(j) - pop(i,j)) * 1.2;
                X3 = delta(j) - A3*D_delta;
                pop(i,j) = (X1 + X2 + X3)/3;
            end
            pop(i, 1:3:end) = max(env.start(1), min(pop(i,1:3:end), env.goal(1)));
            pop(i, 2:3:end) = max(env.start(2), min(pop(i,2:3:end), env.goal(2)));
            pop(i, 3:3:end) = max(20, min(pop(i,3:3:end), env.max_height-20));
            pop(i, 1:3) = env.start; pop(i, end-2:end) = env.goal;
            [fit, len, collide] = calculate_fitness(pop(i,:), env, buildings, params);
            if collide, fit = fit * 2; end
            if fit < alpha_fit
                delta = beta; delta_fit = beta_fit;
                beta = alpha; beta_fit = alpha_fit;
                alpha = pop(i,:); alpha_fit = fit; alpha_len = len;
            elseif fit < beta_fit
                delta = beta; delta_fit = beta_fit;
                beta = pop(i,:); beta_fit = fit;
            elseif fit < delta_fit
                delta = pop(i,:); delta_fit = fit;
            end
        end
        conv_curve(iter) = alpha_fit;
    end
    best_fitness = alpha_fit; best_path = alpha; best_len = alpha_len;
end

function generate_figure_literature(best_paths, conv_curves, buildings, env, stats)
    pso_path = reshape(best_paths.pso, 3, [])';
    ipso_path = reshape(best_paths.ipso, 3, [])';
    spso_path = reshape(best_paths.spso, 3, [])';
    gwo_path = reshape(best_paths.gwo, 3, [])';
    pso_conv = conv_curves.pso;
    ipso_conv = conv_curves.ipso;
    spso_conv = conv_curves.spso;
    gwo_conv = conv_curves.gwo;
    
    [~, pso_len, ~] = calculate_fitness(best_paths.pso, env, buildings, struct('alpha1',0,'alpha2',1,'alpha3',0,'ipso',struct('coll_penalty',0),'safety_dist',0));
    [~, ipso_len, ~] = calculate_fitness(best_paths.ipso, env, buildings, struct('alpha1',0,'alpha2',1,'alpha3',0,'ipso',struct('coll_penalty',0),'safety_dist',0));
    [~, spso_len, ~] = calculate_fitness(best_paths.spso, env, buildings, struct('alpha1',0,'alpha2',1,'alpha3',0,'ipso',struct('coll_penalty',0),'safety_dist',0));
    [~, gwo_len, ~] = calculate_fitness(best_paths.gwo, env, buildings, struct('alpha1',0,'alpha2',1,'alpha3',0,'ipso',struct('coll_penalty',0),'safety_dist',0));
    
    figure('Position', [100, 100, 1200, 900], 'Color', 'w');
    
    % (a) 收敛曲线
    subplot(2,2,1);
    hold on; grid on;
    plot(pso_conv, 'b-', 'LineWidth', 2, 'DisplayName', 'PSO');
    plot(ipso_conv, 'r--', 'LineWidth', 2, 'DisplayName', 'IPSO');
    plot(spso_conv, 'g-.', 'LineWidth', 2, 'DisplayName', 'SPSO');
    plot(gwo_conv, 'm:', 'LineWidth', 2, 'DisplayName', 'GWO');
    xlabel('迭代次数'); ylabel('最优适应度');
    title('(a) 4算法收敛曲线对比');
    legend('Location', 'best');
    hold off;
    
    % (b) 三维路径
    subplot(2,2,2);
    hold on; grid on; view(30, 30);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('(b) 三维路径对比');
    for b = 1:min(100, size(buildings,1))
        x = buildings(b,1); y = buildings(b,2); z = buildings(b,3);
        w = buildings(b,4); l = buildings(b,5); h = buildings(b,6);
        draw_cuboid(x, y, z, w, l, h, [0.85,0.85,0.85], 0.6);
    end
    plot3(pso_path(:,1), pso_path(:,2), pso_path(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'PSO');
    plot3(ipso_path(:,1), ipso_path(:,2), ipso_path(:,3), 'r--', 'LineWidth', 2, 'DisplayName', 'IPSO');
    plot3(spso_path(:,1), spso_path(:,2), spso_path(:,3), 'g-.', 'LineWidth', 2, 'DisplayName', 'SPSO');
    plot3(gwo_path(:,1), gwo_path(:,2), gwo_path(:,3), 'm:', 'LineWidth', 2, 'DisplayName', 'GWO');
    hStart = plot3(env.start(1), env.start(2), env.start(3), 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    hGoal = plot3(env.goal(1), env.goal(2), env.goal(3), 'r*', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
    legend('Location', 'northeastoutside'); % 这里会自动显示所有 DisplayName
    hold off;
    
    % (c) 二维俯视图
    subplot(2,2,3);
    hold on; grid on; axis equal;
    xlim([0, env.map_size]); ylim([0, env.map_size]);
    xlabel('X (m)'); ylabel('Y (m)');
    title('(c) 二维俯视图');
    for b = 1:min(100, size(buildings,1))
        x = buildings(b,1); y = buildings(b,2);
        w = buildings(b,4); l = buildings(b,5);
        rectangle('Position', [x-w/2, y-l/2, w, l], 'FaceColor', [0.85,0.85,0.85], 'EdgeColor', 'k', 'LineWidth', 0.5);
    end
    plot(pso_path(:,1), pso_path(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'PSO');
    plot(ipso_path(:,1), ipso_path(:,2), 'r--', 'LineWidth', 2, 'DisplayName', 'IPSO');
    plot(spso_path(:,1), spso_path(:,2), 'g-.', 'LineWidth', 2, 'DisplayName', 'SPSO');
    plot(gwo_path(:,1), gwo_path(:,2), 'm:', 'LineWidth', 2, 'DisplayName', 'GWO');
    plot(env.start(1), env.start(2), 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    plot(env.goal(1), env.goal(2), 'r*', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
    legend('Location', 'best');
    hold off;
    
    % (d) 统计数据文本显示
    subplot(2,2,4);
    axis off;
    text(0.0, 1.0, '(d) 4算法10次实验统计结果 (Best/Mean/Std)', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 准备表格数据
    row_names = {'路径长度 (m)', '适应度值'};
    alg_names = {'PSO', 'IPSO', 'SPSO', 'GWO'};
    
    % 行1：长度
    y_pos = 0.85;
    text(0.0, y_pos, '路径长度:', 'FontWeight', 'bold');
    text(0.2, y_pos, sprintf('PSO: %.1f / %.1f / %.1f', stats.len.best(1), stats.len.mean(1), stats.len.std(1)));
    text(0.2, y_pos-0.08, sprintf('IPSO: %.1f / %.1f / %.1f', stats.len.best(2), stats.len.mean(2), stats.len.std(2)), 'Color', 'r', 'FontWeight', 'bold');
    text(0.2, y_pos-0.16, sprintf('SPSO: %.1f / %.1f / %.1f', stats.len.best(3), stats.len.mean(3), stats.len.std(3)));
    text(0.2, y_pos-0.24, sprintf('GWO: %.1f / %.1f / %.1f', stats.len.best(4), stats.len.mean(4), stats.len.std(4)));
    
    % 行2：适应度
    y_pos = 0.45;
    text(0.0, y_pos, '适应度值:', 'FontWeight', 'bold');
    text(0.2, y_pos, sprintf('PSO: %.1f / %.1f / %.1f', stats.fit.best(1), stats.fit.mean(1), stats.fit.std(1)));
    text(0.2, y_pos-0.08, sprintf('IPSO: %.1f / %.1f / %.1f', stats.fit.best(2), stats.fit.mean(2), stats.fit.std(2)), 'Color', 'r', 'FontWeight', 'bold');
    text(0.2, y_pos-0.16, sprintf('SPSO: %.1f / %.1f / %.1f', stats.fit.best(3), stats.fit.mean(3), stats.fit.std(3)));
    text(0.2, y_pos-0.24, sprintf('GWO: %.1f / %.1f / %.1f', stats.fit.best(4), stats.fit.mean(4), stats.fit.std(4)));
    
    % 提升率
    improvement = (stats.len.mean(1) - stats.len.mean(2)) / stats.len.mean(1) * 100;
    text(0.0, 0.1, sprintf('IPSO 相对 PSO 平均路径长度提升: %.2f%%', improvement), 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');
    
    print('-dpng', '-r300', 'literature_4alg_figure_final.png');
    fprintf('📊 最终论文图表已保存为 literature_4alg_figure_final.png\n');
end

function draw_cuboid(x, y, z, w, l, h, color, alpha)
    vertices = [
        x-w/2, y-l/2, z;    x+w/2, y-l/2, z;    x+w/2, y+l/2, z;    x-w/2, y+l/2, z;
        x-w/2, y-l/2, z+h;  x+w/2, y-l/2, z+h;  x+w/2, y+l/2, z+h;  x-w/2, y+l/2, z+h
    ];
    faces = [1 2 3 4; 5 6 2 1; 6 7 3 2; 7 8 4 3; 8 5 1 4; 5 8 7 6];
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', color, 'FaceAlpha', alpha, 'EdgeColor', 'k', 'LineWidth', 0.5);
end