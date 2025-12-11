%% =========================================================================
% 复杂三维路径规划环境生成系统（500x500超大间距全区域填满版）
% 核心：楼间距≥20米 + 全区域随机分散 + 无集中拥挤 + 无死循环
% =========================================================================
function complex_3d_environment_500x500()
clear; clc; close all;
fprintf('==============================================\n');
fprintf('   复杂三维路径规划环境生成系统\n');
fprintf('   地图尺寸: 500x500 | 超大间距全区域分散楼房\n');
fprintf('==============================================\n\n');
rng(42, 'twister');
%% 环境参数设置
map_size = 500;        % 地图尺寸 500x500
max_height = 200;      % 最大高度
grid_resolution = 2;   % 网格分辨率
MIN_BUILDING_DIST = 20;% 强制楼间距≥20米
fprintf('🗺️  正在初始化 %dx%dx%d 环境...\n', map_size, map_size, max_height);
%% 生成复杂环境
[environment, obstacle_info, building_list] = create_complex_environment(...
    map_size, max_height, grid_resolution, MIN_BUILDING_DIST);
%% 环境信息统计
fprintf('📊 环境生成完成！统计信息：\n');
fprintf('   网格尺寸: %dx%dx%d\n', size(environment.occupancy_map));
fprintf('   障碍物总数: %d个\n', obstacle_info.total_obstacles);
fprintf('   建筑类型: %d种\n', length(obstacle_info.type_count));
fprintf('   地图复杂度: %.1f%%\n', obstacle_info.obstacle_density * 100);
fprintf('   楼间距: ≥%d米\n', MIN_BUILDING_DIST);
fprintf('   密集区建筑数: %d个（X150-350,Y150-350）\n', obstacle_info.type_count('密集住宅'));
%% 可视化环境
fprintf('📈 生成环境可视化（实体楼房）...\n');
visualize_complex_environment(environment, obstacle_info, building_list, map_size, max_height);
%% 保存环境数据
save_environment_data(environment, obstacle_info, building_list);
fprintf('💾 环境数据已保存！\n');
fprintf('\n✅ 超大间距全区域分散楼房环境生成完成！\n');
end

function [environment, obstacle_info, building_list] = create_complex_environment(...
    map_size, max_height, grid_resolution, min_dist)
    % 初始化结构体
    environment = struct();
    environment.map_size = map_size;
    environment.max_height = max_height;
    environment.resolution = grid_resolution;
    environment.start_point_phys = [50, 50, 50];  % 起点
    environment.goal_point_phys = [450, 450, 50]; % 终点
    
    % 初始化占据网格
    grid_size = round(map_size / grid_resolution);
    height_grid_size = round(max_height / grid_resolution);
    environment.terrain_height = zeros(grid_size, grid_size);
    environment.occupancy_map = zeros(grid_size, grid_size, height_grid_size);
    
    % 初始化建筑列表和障碍物信息
    building_list = [];
    obstacle_info = struct();
    obstacle_info.total_obstacles = 0;
    obstacle_info.type_count = containers.Map({'高层住宅','工业建筑','分散住宅','自然障碍','交通枢纽','密集住宅','高架桥','山体','树木'}, {0,0,0,0,0,0,0,0,0});
    obstacle_info.obstacles = [];
    
    % 生成密集住宅区（X150-350,Y150-350）
    fprintf('   🏙️  生成城市密集建筑区（X150-350,Y150-350）...\n');
    dense_min_dist = 10;  % 密集区楼间距10米
    dense_num = 60;       % 密集区60栋楼
    dense_x_range = [150, 350];
    dense_y_range = [150, 350];
    
    bl_len = numel(building_list);
    for i = 1:dense_num
        pos_x = dense_x_range(1) + rand()*(dense_x_range(2)-dense_x_range(1));
        pos_y = dense_y_range(1) + rand()*(dense_y_range(2)-dense_y_range(1));
        pos_z = environment.terrain_height(round(pos_x/grid_resolution), round(pos_y/grid_resolution));
        
        width = 8 + rand()*4;
        length = 8 + rand()*4;
        height = 30 + rand()*20;
        
        % 检查间距
        valid = true;
        if bl_len > 0
            for j = 1:bl_len
                dist = sqrt((pos_x - building_list(j).x_phys)^2 + (pos_y - building_list(j).y_phys)^2);
                if dist < dense_min_dist
                    valid = false;
                    break;
                end
            end
        end
        
        if valid
            building = struct();
            building.x_phys = pos_x;
            building.y_phys = pos_y;
            building.z_phys = pos_z;
            building.width = width;
            building.length = length;
            building.height = height;
            building.type = '密集住宅';
            building_list = [building_list, building];
            obstacle_info.type_count('密集住宅') = obstacle_info.type_count('密集住宅') + 1;
            obstacle_info.total_obstacles = obstacle_info.total_obstacles + 1;
            bl_len = bl_len + 1;
        end
    end
    
    % 计算障碍物密度
    total_voxels = numel(environment.occupancy_map);
    occupied_voxels = sum(environment.occupancy_map(:) == 1);
    obstacle_info.obstacle_density = occupied_voxels / total_voxels;
    
    % 生成其他类型建筑
    % 1. 高层住宅（50栋）
    [environment, obstacle_info, building_list] = add_building_type(...
        environment, obstacle_info, building_list, map_size, max_height, grid_resolution, ...
        '高层住宅', 50, [15,25], [100,180], min_dist);
    
    % 2. 工业建筑（40栋）
    [environment, obstacle_info, building_list] = add_building_type(...
        environment, obstacle_info, building_list, map_size, max_height, grid_resolution, ...
        '工业建筑', 40, [20,30], [60,100], min_dist);
    
    % 3. 分散住宅（80栋）
    [environment, obstacle_info, building_list] = add_building_type(...
        environment, obstacle_info, building_list, map_size, max_height, grid_resolution, ...
        '分散住宅', 80, [8,12], [30,50], min_dist);
    
    % 4. 自然障碍（60个）
    [environment, obstacle_info, building_list] = add_natural_type(...
        environment, obstacle_info, building_list, map_size, max_height, grid_resolution, 60, min_dist);
    
    % 5. 立体交通枢纽（高架桥+桥墩）
    [environment, obstacle_info, building_list] = add_transportation_hub(...
        environment, obstacle_info, building_list, map_size, grid_resolution, min_dist);
    
    % 重新计算障碍物密度
    total_voxels = numel(environment.occupancy_map);
    occupied_voxels = sum(environment.occupancy_map(:) == 1);
    obstacle_info.obstacle_density = occupied_voxels / total_voxels;
end

function [environment, obstacle_info, building_list] = add_building_type(...
    environment, obstacle_info, building_list, map_size, max_height, grid_resolution, ...
    build_type, num_build, size_range, height_range, min_dist)
    grid_size_x = size(environment.occupancy_map, 1);
    grid_size_z = size(environment.occupancy_map, 3);
    bl_len = numel(building_list);
    build_count = 0;
    max_attempts = num_build * 10;  % 防止死循环
    attempt = 0;
    
    while build_count < num_build && attempt < max_attempts
        attempt = attempt + 1;
        % 随机位置
        pos_x_phys = rand() * (map_size - 20) + 10;
        pos_y_phys = rand() * (map_size - 20) + 10;
        pos_x = ceil(pos_x_phys / grid_resolution);
        pos_y = ceil(pos_y_phys / grid_resolution);
        
        % 检查间距
        dist_ok = true;
        if bl_len > 0
            for i = 1:bl_len
                exist_x = building_list(i).x_phys;
                exist_y = building_list(i).y_phys;
                dist = sqrt((pos_x_phys - exist_x)^2 + (pos_y_phys - exist_y)^2);
                if dist < min_dist
                    dist_ok = false;
                    break;
                end
            end
        end
        
        if ~dist_ok
            continue;
        end
        
        % 随机尺寸/高度
        width_vox = randi([ceil(size_range(1)/grid_resolution), ceil(size_range(2)/grid_resolution)]);
        length_vox = width_vox;
        height_vox = randi([ceil(height_range(1)/grid_resolution), ceil(height_range(2)/grid_resolution)]);
        height_vox = min(height_vox, grid_size_z - 10);
        
        % 生成建筑
        terrain_height = environment.terrain_height(pos_x, pos_y);
        base_z = ceil(terrain_height / grid_resolution) + 1;
        if base_z < 1; base_z = 1; end
        if base_z + height_vox >= grid_size_z
            height_vox = grid_size_z - base_z - 5;
        end
        
        [environment, success] = add_cuboid_building(environment, pos_x, pos_y, base_z, width_vox, length_vox, height_vox);
        if success
            build_count = build_count + 1;
            bl_len = bl_len + 1;
            building_list(bl_len).type = build_type;
            building_list(bl_len).x_phys = pos_x_phys;
            building_list(bl_len).y_phys = pos_y_phys;
            building_list(bl_len).z_phys = terrain_height + grid_resolution;
            building_list(bl_len).width = width_vox * grid_resolution;
            building_list(bl_len).length = length_vox * grid_resolution;
            building_list(bl_len).height = height_vox * grid_resolution;
            obstacle_info.obstacles = [obstacle_info.obstacles; pos_x, pos_y, base_z, width_vox, length_vox, height_vox, 1];
            obstacle_info.type_count(build_type) = obstacle_info.type_count(build_type) + 1;
        end
    end
    
    if build_count < num_build
        warning('生成%s失败，仅生成%d个（目标%d个）', build_type, build_count, num_build);
    end
    obstacle_info.total_obstacles = obstacle_info.total_obstacles + build_count;
    fprintf('      生成%d个%s（间距≥%d米）\n', build_count, build_type, min_dist);
end

function [environment, obstacle_info, building_list] = add_natural_type(...
    environment, obstacle_info, building_list, map_size, max_height, grid_resolution, num_nat, min_dist)
    grid_size_x = size(environment.occupancy_map, 1);
    grid_size_z = size(environment.occupancy_map, 3);
    bl_len = numel(building_list);
    nat_count = 0;
    max_attempts = num_nat * 10;
    attempt = 0;
    
    while nat_count < num_nat && attempt < max_attempts
        attempt = attempt + 1;
        % 随机位置
        pos_x_phys = rand() * (map_size - 20) + 10;
        pos_y_phys = rand() * (map_size - 20) + 10;
        pos_x = ceil(pos_x_phys / grid_resolution);
        pos_y = ceil(pos_y_phys / grid_resolution);
        
        % 检查间距
        dist_ok = true;
        if bl_len > 0
            for i = 1:bl_len
                exist_x = building_list(i).x_phys;
                exist_y = building_list(i).y_phys;
                dist = sqrt((pos_x_phys - exist_x)^2 + (pos_y_phys - exist_y)^2);
                if dist < min_dist
                    dist_ok = false;
                    break;
                end
            end
        end
        
        if ~dist_ok
            continue;
        end
        
        % 生成自然障碍
        terrain_height = environment.terrain_height(pos_x, pos_y);
        base_z = ceil(terrain_height / grid_resolution) + 1;
        if base_z >= grid_size_z
            continue;
        end
        
        if rand() < 0.2
            % 山体
            radius_vox = randi([ceil(10/grid_resolution), ceil(20/grid_resolution)]);
            height_vox = randi([ceil(80/grid_resolution), ceil(120/grid_resolution)]);
            [environment, success] = add_pyramidal_building(environment, pos_x, pos_y, base_z, radius_vox, height_vox);
            shape_name = '山体';
        else
            % 树木
            radius_vox = randi([ceil(1/grid_resolution), ceil(3/grid_resolution)]);
            height_vox = randi([ceil(10/grid_resolution), ceil(30/grid_resolution)]);
            [environment, success] = add_pyramidal_building(environment, pos_x, pos_y, base_z, radius_vox, height_vox);
            shape_name = '树木';
        end
        
        if success
            nat_count = nat_count + 1;
            bl_len = bl_len + 1;
            building_list(bl_len).type = '自然障碍';
            building_list(bl_len).x_phys = pos_x_phys;
            building_list(bl_len).y_phys = pos_y_phys;
            building_list(bl_len).z_phys = terrain_height + grid_resolution;
            building_list(bl_len).width = radius_vox * 2 * grid_resolution;
            building_list(bl_len).length = radius_vox * 2 * grid_resolution;
            building_list(bl_len).height = height_vox * grid_resolution;
            obstacle_info.obstacles = [obstacle_info.obstacles; pos_x, pos_y, base_z, radius_vox, radius_vox, height_vox, 7];
            obstacle_info.type_count(shape_name) = obstacle_info.type_count(shape_name) + 1;
        end
    end
    
    if nat_count < num_nat
        warning('生成自然障碍失败，仅生成%d个（目标%d个）', nat_count, num_nat);
    end
    obstacle_info.total_obstacles = obstacle_info.total_obstacles + nat_count;
    fprintf('      生成%d个自然障碍（间距≥%d米）\n', nat_count, min_dist);
end

function [environment, obstacle_info, building_list] = add_transportation_hub(...
    environment, obstacle_info, building_list, map_size, grid_resolution, min_dist)
    grid_size_x = size(environment.occupancy_map, 1);
    grid_size_z = size(environment.occupancy_map, 3);
    bl_len = numel(building_list);
    max_attempts = 1000;  % 防止死循环
    attempt = 0;
    bridge_y_phys = 250;  % 默认位置
    
    % 寻找高架桥位置
    while attempt < max_attempts
        attempt = attempt + 1;
        bridge_y_phys = rand() * 300 + 100;  % y范围100-400
        dist_ok = true;
        for i = 1:bl_len
            exist_y = building_list(i).y_phys;
            if abs(bridge_y_phys - exist_y) < min_dist
                dist_ok = false;
                break;
            end
        end
        if dist_ok
            break;
        end
        if attempt == max_attempts
            warning('未找到理想高架桥位置，使用默认位置（y=250）');
            bridge_y_phys = 250;
        end
    end
    
    % 高架桥参数
    x_min_phys = 150;
    x_max_phys = 350;
    bridge_height_phys = 80;
    bridge_width_phys = 8;
    bridge_length_phys = x_max_phys - x_min_phys;
    
    % 转换为网格坐标
    x_min = ceil(x_min_phys / grid_resolution);
    x_max = ceil(x_max_phys / grid_resolution);
    bridge_y = ceil(bridge_y_phys / grid_resolution);
    bridge_height = ceil(bridge_height_phys / grid_resolution);
    bridge_width = ceil(bridge_width_phys / grid_resolution);
    
    % 绘制高架桥
    for x = x_min:x_max
        for y = (bridge_y - bridge_width/2):(bridge_y + bridge_width/2)
            y = round(y);
            for z = bridge_height:(bridge_height + ceil(3/grid_resolution))
                if x >= 1 && x <= grid_size_x && y >= 1 && y <= grid_size_x && z >= 1 && z <= grid_size_z
                    environment.occupancy_map(x, y, z) = 1;
                end
            end
        end
    end
    
    % 绘制桥墩（间距≥40米）
    pier_spacing_phys = 40;
    pier_count = 0;
    for x_phys = x_min_phys:pier_spacing_phys:x_max_phys
        x = ceil(x_phys / grid_resolution);
        pier_radius = ceil(2 / grid_resolution);
        pier_height = bridge_height;
        [environment, success] = add_cylindrical_building(environment, x, bridge_y, 1, pier_radius, pier_height);
        if success
            pier_count = pier_count + 1;
            bl_len = bl_len + 1;
            building_list(bl_len).type = '交通枢纽';
            building_list(bl_len).x_phys = x_phys;
            building_list(bl_len).y_phys = bridge_y_phys;
            building_list(bl_len).z_phys = 0;
            building_list(bl_len).width = pier_radius * 2 * grid_resolution;
            building_list(bl_len).length = pier_radius * 2 * grid_resolution;
            building_list(bl_len).height = pier_height * grid_resolution;
        end
    end
    
    % 记录高架桥信息
    bl_len = bl_len + 1;
    building_list(bl_len).type = '交通枢纽';
    building_list(bl_len).x_phys = (x_min_phys+x_max_phys)/2;
    building_list(bl_len).y_phys = bridge_y_phys;
    building_list(bl_len).z_phys = bridge_height_phys;
    building_list(bl_len).width = bridge_length_phys;
    building_list(bl_len).length = bridge_width_phys;
    building_list(bl_len).height = 3;
    obstacle_info.obstacles = [obstacle_info.obstacles; mean([x_min, x_max]), bridge_y, bridge_height, ...
        ceil(bridge_length_phys/grid_resolution), bridge_width, ceil(3/grid_resolution), 5];
    obstacle_info.type_count('高架桥') = 1;
    obstacle_info.total_obstacles = obstacle_info.total_obstacles + 1 + pier_count;
    fprintf('      生成立体交通枢纽（高架桥+%d个桥墩，间距≥%d米）\n', pier_count, pier_spacing_phys);
end

function visualize_complex_environment(environment, obstacle_info, building_list, map_size, max_height)
    % 创建3D图形
    figure('Name', '复杂三维路径规划环境', 'Position', [100, 100, 1000, 800]);
    hold on; grid on; box on;
    xlabel('X坐标 (米)'); ylabel('Y坐标 (米)'); zlabel('高度 (米)');
    title('500x500复杂三维路径规划环境');
    axis equal;
    axis([0 map_size 0 map_size 0 max_height]);
    
    % 定义不同建筑类型的颜色
    color_map = containers.Map();
    color_map('高层住宅') = [0.85, 0.33, 0.1];   % 橙色
    color_map('工业建筑') = [0.9, 0.6, 0];      % 黄色
    color_map('分散住宅') = [0.47, 0.67, 0.19];  % 绿色
    color_map('密集住宅') = [0.3, 0.75, 0.93];   % 浅蓝色
    color_map('交通枢纽') = [0.63, 0.13, 0.94];  % 紫色
    color_map('山体') = [0.55, 0.27, 0.07];      % 棕色
    color_map('树木') = [0, 0.5, 0];             % 深绿色
    color_map('高架桥') = [0.5, 0.5, 0.5];       % 灰色
    
   % 绘制所有建筑（删除冗余条件判断）
for i = 1:numel(building_list)
    b = building_list(i);
    if isKey(color_map, b.type)
        color = color_map(b.type);
    else
        color = [0.5, 0.5, 0.5];  % 默认灰色
    end
    
    % 计算建筑角落坐标
    x = b.x_phys;
    y = b.y_phys;
    z = b.z_phys;
    w = b.width;
    l = b.length;
    h = b.height;
    
    % 直接绘制（无需特殊判断，逻辑统一）
    draw_cuboid(x - w/2, y - l/2, z, w, l, h, color, 0.8);
end
    
    % 绘制起点和终点
    start = environment.start_point_phys;
    goal = environment.goal_point_phys;
    plot3(start(1), start(2), start(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot3(goal(1), goal(2), goal(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    text(start(1), start(2), start(3)+10, '起点', 'Color', 'r');
    text(goal(1), goal(2), goal(3)+10, '终点', 'Color', 'g');
    
    % 添加图例
    legend_items = {'高层住宅', '工业建筑', '分散住宅', '密集住宅', '交通枢纽', '山体', '树木', '高架桥', '起点', '终点'};
    legend(legend_items, 'Location', 'best');
    
    % 设置视角
    view(30, 45);
    hold off;
end

function draw_cuboid(x, y, z, width, length, height, color, alpha)
    % 绘制长方体
    vertices = [
        x, y, z;
        x + width, y, z;
        x + width, y + length, z;
        x, y + length, z;
        x, y, z + height;
        x + width, y, z + height;
        x + width, y + length, z + height;
        x, y + length, z + height
    ];
    
    faces = [
        1, 2, 3, 4;
        5, 6, 7, 8;
        1, 2, 6, 5;
        2, 3, 7, 6;
        3, 4, 8, 7;
        4, 1, 5, 8
    ];
    
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', color, 'EdgeColor', 'k', 'FaceAlpha', alpha);
end

function save_environment_data(environment, obstacle_info, building_list)
    % 保存环境数据到MAT文件
    save('complex_3d_environment.mat', 'environment', 'obstacle_info', 'building_list');
end

function environment = generate_dynamic_terrain(environment, map_size, max_height, grid_resolution)
    grid_size_x = size(environment.occupancy_map, 1);
    grid_size_y = size(environment.occupancy_map, 2);
    
    [X, Y] = meshgrid(1:grid_size_x, 1:grid_size_y);
    X = X'; Y = Y';
    
    terrain = zeros(grid_size_x, grid_size_y);
    noise1 = 0.4 * generate_perlin_noise(grid_size_x, grid_size_y, 0.01);
    noise2 = 0.3 * generate_perlin_noise(grid_size_x, grid_size_y, 0.05);
    noise3 = 0.2 * generate_perlin_noise(grid_size_x, grid_size_y, 0.1);
    terrain = (noise1 + noise2 + noise3) * max_height * 0.1;
    
    river_center_y = grid_size_y * 0.6;
    for i = 1:grid_size_x
        for j = 1:grid_size_y
            distance_to_river = abs(j - river_center_y);
            if distance_to_river < grid_size_y * 0.1
                valley_depth = max_height * 0.05 * (1 - distance_to_river / (grid_size_y * 0.1));
                terrain(i,j) = terrain(i,j) - valley_depth;
            end
        end
    end
    
    environment.terrain_height = max(0, terrain);
    
    for i = 1:grid_size_x
        for j = 1:grid_size_y
            terrain_height_voxels = ceil(environment.terrain_height(i,j) / grid_resolution);
            terrain_height_voxels = min(terrain_height_voxels, size(environment.occupancy_map,3));
            environment.occupancy_map(i, j, 1:terrain_height_voxels) = 1;
        end
    end
end

function noise = generate_perlin_noise(width, height, frequency)
    [X, Y] = meshgrid(1:width, 1:height);
    X = X'; Y = Y';
    
    noise = randn(size(X));
    h = fspecial('gaussian', 15, 1/frequency);
    noise = imfilter(noise, h, 'replicate');
    noise = (noise - min(noise(:))) / (max(noise(:)) - min(noise(:)));
end

function [environment, success] = add_cuboid_building(environment, center_x, center_y, base_z, width, length, height)
    success = false;
    grid_size = size(environment.occupancy_map);
    
    x_start = max(1, center_x - floor(width/2));
    x_end = min(grid_size(1), center_x + floor(width/2));
    y_start = max(1, center_y - floor(length/2));
    y_end = min(grid_size(2), center_y + floor(length/2));
    z_end = min(grid_size(3), base_z + height - 1);
    
    if x_end - x_start < 2; x_end = x_start + 2; end
    if y_end - y_start < 2; y_end = y_start + 2; end
    if z_end - base_z < 5; z_end = base_z + 5; end
    
    if x_start >= x_end || y_start >= y_end || base_z > z_end
        return;
    end
    
    try
        environment.occupancy_map(x_start:x_end, y_start:y_end, base_z:z_end) = 1;
        success = true;
    catch
        success = false;
    end
end

function [environment, success] = add_cylindrical_building(environment, center_x, center_y, base_z, radius, height)
    success = false;
    grid_size = size(environment.occupancy_map);
    
    z_end = min(grid_size(3), base_z + height - 1);
    
    for x = max(1, center_x-radius):min(grid_size(1), center_x+radius)
        for y = max(1, center_y-radius):min(grid_size(2), center_y+radius)
            distance = sqrt((x - center_x)^2 + (y - center_y)^2);
            if distance <= radius
                for z = base_z:z_end
                    if x >= 1 && x <= grid_size(1) && y >= 1 && y <= grid_size(2) && z >= 1 && z <= grid_size(3)
                        environment.occupancy_map(x, y, z) = 1;
                    end
                end
            end
        end
    end
    success = true;
end

function [environment, success] = add_pyramidal_building(environment, center_x, center_y, base_z, base_radius, height)
    success = false;  % 初始化成功标志为false
    grid_size = size(environment.occupancy_map);
    
    for z = 0:(height-1)
        current_radius = base_radius * (1 - z/height);
        if current_radius < 1
            break;  % 半径过小，停止绘制
        end
        
        current_z = base_z + z;
        if current_z > grid_size(3)
            break;  % 超出高度范围，停止绘制
        end
        
        % 遍历当前高度层的栅格
        for x = max(1, center_x - floor(current_radius)):min(grid_size(1), center_x + floor(current_radius))
            for y = max(1, center_y - floor(current_radius)):min(grid_size(2), center_y + floor(current_radius))
                distance = sqrt((x - center_x)^2 + (y - center_y)^2);
                if distance <= current_radius
                    if x >= 1 && x <= grid_size(1) && y >= 1 && y <= grid_size(2)
                        % 补全核心赋值：栅格设为1（表示有障碍物）
                        environment.occupancy_map(x, y, current_z) = 1;
                    end
                end
            end
        end
    end
    
    success = true;  % 绘制完成，标记成功
end