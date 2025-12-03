
%  Author: Yiqing Zhi
%  Institution: University of Waterloo
%  Project: Four-Foot Spider Robot - Kinematic Analysis
%  Based on: Actual Engineering Drawings (Shanghai Yufei Network Technology)

clear; clc; close all;

%% Fancy Startup Banner
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                                                                ║\n');
fprintf('║        🕷️  SPIDER ROBOT KINEMATICS ANALYSIS SUITE 🕷️         ║\n');
fprintf('║                                                                ║\n');
fprintf('║                  University of Waterloo                        ║\n');
fprintf('║              Mechanical Engineering Project                    ║\n');
fprintf('║                                                                ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% Robot Parameters with Fancy Display
fprintf('📐 LOADING ROBOT PARAMETERS FROM CAD DRAWINGS...\n');
pause(0.5);

L1 = 45;   % Coxa length [mm]
L2 = 90;   % Femur length [mm]
L3 = 90;   % Tibia length [mm]

fprintf('✓ Coxa Length (L1):  %3d mm\n', L1);
fprintf('✓ Femur Length (L2): %3d mm\n', L2);
fprintf('✓ Tibia Length (L3): %3d mm\n', L3);
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('📊 Total Leg Reach:   %3d mm\n', L1+L2+L3);
fprintf('⚖️  Material: Carbon Steel\n');
fprintf('🏗️  Manufacturing: 3D Printed PPS-CF\n\n');

%% Joint Limits
theta1_limits = [-60, 60];
theta2_limits = [-90, 90];
theta3_limits = [-135, 30];

%% ========================================================================
%  ENHANCED INVERSE KINEMATICS WITH ERROR HANDLING
%% ========================================================================
function [theta1, theta2, theta3, reachable, error_msg] = spider_leg_IK_enhanced(x, y, z, L1, L2, L3)
    error_msg = '';
    
    % Coxa angle
    theta1 = atan2d(y, x);
    
    % Project into sagittal plane
    r_xy = sqrt(x^2 + y^2);
    r = r_xy - L1;
    
    % Check reachability
    reach_distance = sqrt(r^2 + z^2);
    max_reach = L2 + L3;
    min_reach = abs(L2 - L3);
    
    if r_xy < L1
        reachable = false;
        theta2 = NaN; theta3 = NaN;
        error_msg = 'Target too close to base';
        return;
    end
    
    if reach_distance > max_reach
        reachable = false;
        theta2 = NaN; theta3 = NaN;
        error_msg = sprintf('Target too far (%.1fmm > %.1fmm)', reach_distance, max_reach);
        return;
    end
    
    if reach_distance < min_reach
        reachable = false;
        theta2 = NaN; theta3 = NaN;
        error_msg = sprintf('Target too close (%.1fmm < %.1fmm)', reach_distance, min_reach);
        return;
    end
    
    reachable = true;
    
    % Law of cosines
    cos_theta3 = (r^2 + z^2 - L2^2 - L3^2) / (2 * L2 * L3);
    cos_theta3 = max(-1, min(1, cos_theta3));
    theta3 = -acosd(cos_theta3);
    
    % Calculate theta2
    alpha = atan2d(z, r);
    beta = atan2d(L3 * sind(theta3), L2 + L3 * cosd(theta3));
    theta2 = alpha - beta;
end

%% ========================================================================
%  FORWARD KINEMATICS
%% ========================================================================
function [x, y, z] = spider_leg_FK(theta1, theta2, theta3, L1, L2, L3)
    t1 = deg2rad(theta1);
    t2 = deg2rad(theta2);
    t3 = deg2rad(theta3);
    
    x = (L1 + L2*cos(t2) + L3*cos(t2+t3)) * cos(t1);
    y = (L1 + L2*cos(t2) + L3*cos(t2+t3)) * sin(t1);
    z = L2*sin(t2) + L3*sin(t2+t3);
end

%% ========================================================================
%  🎨 ULTRA FANCY 3D WORKSPACE VISUALIZATION
%% ========================================================================
function visualize_workspace_fancy(L1, L2, L3)
    fprintf('🎨 GENERATING ADVANCED WORKSPACE VISUALIZATION...\n');
    
    fig = figure('Name', 'Spider Leg Workspace - Professional Analysis', ...
                 'Position', [50 50 1400 900], ...
                 'Color', [0 0 0]);
    
    % Create tiled layout
    tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    % ===== TILE 1: 3D Workspace Cloud =====
    nexttile([2 2]);
    
    theta1_range = linspace(-60, 60, 15);
    theta2_range = linspace(-90, 90, 20);
    theta3_range = linspace(-135, 30, 20);
    
    workspace_points = [];
    colors_z = [];
    
    h_wait = waitbar(0, 'Computing 3D workspace...', 'Name', 'Analysis Progress');
    total_iterations = length(theta1_range) * length(theta2_range) * length(theta3_range);
    current_iter = 0;
    
    for t1 = theta1_range
        for t2 = theta2_range
            for t3 = theta3_range
                [x, y, z] = spider_leg_FK(t1, t2, t3, L1, L2, L3);
                workspace_points = [workspace_points; x, y, z];
                colors_z = [colors_z; z];
                
                current_iter = current_iter + 1;
                if mod(current_iter, 100) == 0
                    waitbar(current_iter / total_iterations, h_wait);
                end
            end
        end
    end
    close(h_wait);
    
    % Create beautiful 3D scatter with gradient
    scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), ...
             25, colors_z, 'filled', 'MarkerFaceAlpha', 0.4, 'MarkerEdgeAlpha', 0.6);
    
    hold on;
    
    % Add robot base representation
    [X_base, Y_base, Z_base] = sphere(20);
    surf(X_base*15, Y_base*15, Z_base*15, 'FaceColor', [0.3 0.3 0.3], ...
         'EdgeColor', 'none', 'FaceAlpha', 0.8);
    
    % Coordinate frame arrows
    quiver3(0, 0, 0, 80, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0, 80, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0, 0, 80, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    
    text(90, 0, 0, 'X', 'FontSize', 16, 'Color', 'r', 'FontWeight', 'bold');
    text(0, 90, 0, 'Y', 'FontSize', 16, 'Color', 'g', 'FontWeight', 'bold');
    text(0, 0, 90, 'Z', 'FontSize', 16, 'Color', 'b', 'FontWeight', 'bold');

    set(gca, 'Color', [0 0 0]);  % BLACK WORKSPACE BACKGROUND
    set(gca, 'XColor', 'white', 'YColor', 'white', 'ZColor', 'white');  % White axis labels
    set(gca, 'GridColor', [0.5 0.5 0.5]);  % Gray grid

    
    % Ground plane
    [X_ground, Y_ground] = meshgrid(-50:50:300, -150:50:150);
    Z_ground = -150 * ones(size(X_ground));
    surf(X_ground, Y_ground, Z_ground, 'FaceColor', [0.85 0.85 0.9], ...
         'EdgeColor', [0.7 0.7 0.75], 'FaceAlpha', 0.3, 'LineWidth', 0.5);
    
    % Calculate and display workspace volume
    [~, vol] = convhull(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3));
    
    % Styling
    xlabel('X Position [mm]', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('Y Position [mm]', 'FontSize', 13, 'FontWeight', 'bold');
    zlabel('Z Position [mm]', 'FontSize', 13, 'FontWeight', 'bold');
    title(sprintf('🕷️ Spider Leg Reachable Workspace | Volume: %.0f cm³', vol/1000), ...
          'FontSize', 15, 'FontWeight', 'bold');
    
    grid on;
    axis equal;
    colormap(jet);
    c = colorbar;
    c.Label.String = 'Height (Z) [mm]';
    c.Label.FontSize = 11;
    c.Label.FontWeight = 'bold';
    
    view(135, 25);
    lighting gouraud;
    camlight('headlight');
    set(gca, 'FontSize', 11, 'LineWidth', 1.5);
    
    % Add workspace statistics
    dim = [0.15 0.7 0.2 0.15];
    str = {sprintf('📊 Workspace Statistics:'), ...
           sprintf('• Max Reach: %.0f mm', max(sqrt(workspace_points(:,1).^2 + workspace_points(:,2).^2 + workspace_points(:,3).^2))), ...
           sprintf('• Volume: %.0f cm³', vol/1000), ...
           sprintf('• Points: %d', size(workspace_points, 1)), ...
           sprintf('• DOF: 3')};
    annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', ...
               'BackgroundColor', 'white', 'EdgeColor', [0.3 0.3 0.8], ...
               'LineWidth', 2, 'FontSize', 10, 'FontWeight', 'bold');
    
    hold off;
    
    fprintf('✓ Workspace visualization complete!\n');
    fprintf('  └─ Volume: %.0f cm³\n', vol/1000);
    fprintf('  └─ Points analyzed: %d\n\n', size(workspace_points, 1));
end

%% ========================================================================
%  🎬 CINEMATIC LEG ANIMATION WITH TRAILS
%% ========================================================================
function animate_spider_leg_cinematic(trajectory, L1, L2, L3, title_text)
    fig = figure('Name', 'Spider Leg Motion - Cinematic View', ...
                 'Position', [100 100 1200 800], ...
                 'Color', [0.05 0.05 0.15]);
   
    
    foot_trail = [];
    
    for i = 1:size(trajectory, 1)
        clf;
        
        theta1 = trajectory(i, 1);
        theta2 = trajectory(i, 2);
        theta3 = trajectory(i, 3);
        
        t1 = deg2rad(theta1);
        t2 = deg2rad(theta2);
        t3 = t2 + deg2rad(theta3);
        
        p0 = [0, 0, 0];
        p1 = [L1*cos(t1), L1*sin(t1), 0];
        
        p2_x = p1(1) + L2*cos(t2)*cos(t1);
        p2_y = p1(2) + L2*cos(t2)*sin(t1);
        p2_z = L2*sin(t2);
        p2 = [p2_x, p2_y, p2_z];
        
        p3_x = p2(1) + L3*cos(t3)*cos(t1);
        p3_y = p2(2) + L3*cos(t3)*sin(t1);
        p3_z = p2(3) + L3*sin(t3);
        p3 = [p3_x, p3_y, p3_z];
        
        foot_trail = [foot_trail; p3];

        hold on;
        
        plot3([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], ...
              'Color', [1 0.3 0], 'LineWidth', 8);
        
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
              'Color', [1 0.7 0], 'LineWidth', 8);
        
        plot3([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], ...
              'Color', [0 0.9 1], 'LineWidth', 8);
        
        scatter3([p0(1), p1(1), p2(1), p3(1)], ...
                 [p0(2), p1(2), p2(2), p3(2)], ...
                 [p0(3), p1(3), p2(3), p3(3)], ...
                 200, [1 0.2 0.2; 1 0.5 0; 1 0.8 0; 0 1 1], 'filled', ...
                 'MarkerEdgeColor', 'white', 'LineWidth', 2);
        
        % Foot trail with fading effect
        if size(foot_trail, 1) > 1
            trail_colors = linspace(0.3, 1, size(foot_trail, 1))';
            scatter3(foot_trail(:,1), foot_trail(:,2), foot_trail(:,3), ...
                     30, [trail_colors, zeros(size(trail_colors)), trail_colors], ...
                     'filled', 'MarkerFaceAlpha', 0.6);
            plot3(foot_trail(:,1), foot_trail(:,2), foot_trail(:,3), ...
                  'm--', 'LineWidth', 2, 'Color', [1 0 1 0.5]);
        end
        
        % Ground plane with grid
        [X_grid, Y_grid] = meshgrid(-50:25:300, -150:25:150);
        Z_grid = -130 * ones(size(X_grid));
        surf(X_grid, Y_grid, Z_grid, 'FaceColor', [0.1 0.1 0.2], ...
             'EdgeColor', [0.3 0.3 0.5], 'FaceAlpha', 0.4, 'LineWidth', 0.5);
        
        % Coordinate frame
        quiver3(0, 0, 0, 50, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.8);
        quiver3(0, 0, 0, 0, 50, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.8);
        quiver3(0, 0, 0, 0, 0, 50, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.8);
        
        % Labels and styling
        xlabel('X [mm]', 'Color', 'white', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('Y [mm]', 'Color', 'white', 'FontSize', 12, 'FontWeight', 'bold');
        zlabel('Z [mm]', 'Color', 'white', 'FontSize', 12, 'FontWeight', 'bold');
        
        title_str = sprintf('%s | Frame %d/%d\nθ₁=%.1f°  θ₂=%.1f°  θ₃=%.1f°', ...
                           title_text, i, size(trajectory,1), theta1, theta2, theta3);
        title(title_str, 'Color', 'white', 'FontSize', 14, 'FontWeight', 'bold');
        
        grid on;
        axis equal;
        xlim([-50 300]);
        ylim([-150 150]);
        zlim([-150 50]);
        view(45, 20);
        
        set(gca, 'Color', [0.05 0.05 0.15], 'XColor', 'white', 'YColor', 'white', ...
                 'ZColor', 'white', 'GridColor', [0.3 0.3 0.5], 'GridAlpha', 0.5);
        
        lighting gouraud;
        camlight('headlight');
        
        hold off;
        drawnow;
        
        % Write frame to video (if enabled)
        % frame = getframe(gcf);
        % writeVideo(v, frame);
        
        pause(0.04);
    end
    
    % close(v);  % Close video writer
    fprintf('✓ Animation complete!\n\n');
end

%% ========================================================================
%  📈 JOINT ANGLE TRAJECTORY PLOTS
%% ========================================================================
function plot_joint_trajectories(trajectory, title_text)
    figure('Name', 'Joint Angle Trajectories', 'Position', [100 100 1200 700]);
    
    time = linspace(0, 1, size(trajectory, 1));
    
    subplot(3, 1, 1);
    plot(time, trajectory(:,1), 'r-', 'LineWidth', 2.5);
    hold on;
    plot(time, trajectory(:,1), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r');
    grid on;
    ylabel('θ₁ - Coxa [°]', 'FontSize', 12, 'FontWeight', 'bold');
    title([title_text ' - Joint Angle Profiles'], 'FontSize', 14, 'FontWeight', 'bold');
    xlim([0 1]);
    set(gca, 'FontSize', 11, 'LineWidth', 1.5);
    
    subplot(3, 1, 2);
    plot(time, trajectory(:,2), 'g-', 'LineWidth', 2.5);
    hold on;
    plot(time, trajectory(:,2), 'go', 'MarkerSize', 4, 'MarkerFaceColor', 'g');
    grid on;
    ylabel('θ₂ - Femur [°]', 'FontSize', 12, 'FontWeight', 'bold');
    xlim([0 1]);
    set(gca, 'FontSize', 11, 'LineWidth', 1.5);
    
    subplot(3, 1, 3);
    plot(time, trajectory(:,3), 'b-', 'LineWidth', 2.5);
    hold on;
    plot(time, trajectory(:,3), 'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'b');
    grid on;
    xlabel('Normalized Time', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('θ₃ - Tibia [°]', 'FontSize', 12, 'FontWeight', 'bold');
    xlim([0 1]);
    set(gca, 'FontSize', 11, 'LineWidth', 1.5);
end

%% ========================================================================
%  🚶 ADVANCED GAIT GENERATION
%% ========================================================================
function trajectory = generate_walking_gait(L1, L2, L3, stride_length, step_height)
    num_points = 80;
    trajectory = zeros(num_points, 3);
    
    x_stand = 160;
    y_stand = 0;
    z_stand = -90;
    
    for i = 1:num_points
        phase = (i-1) / (num_points-1);
        
        if phase < 0.5
            swing_phase = phase / 0.5;
            x = x_stand + stride_length * (swing_phase - 0.5);
            y = y_stand;
            z = z_stand + step_height * sin(pi * swing_phase);
        else
            stance_phase = (phase - 0.5) / 0.5;
            x = x_stand + stride_length * (0.5 - stance_phase);
            y = y_stand;
            z = z_stand;
        end
        
        [theta1, theta2, theta3, reachable, ~] = spider_leg_IK_enhanced(x, y, z, L1, L2, L3);
        
        if reachable
            trajectory(i, :) = [theta1, theta2, theta3];
        else
            trajectory(i, :) = trajectory(max(1, i-1), :);
        end
    end
end

%% ========================================================================
%  🎯 MAIN EXECUTION WITH INTERACTIVE MENU
%% ========================================================================

% Test IK
fprintf('🎯 TESTING INVERSE KINEMATICS...\n');
x_target = 180;
y_target = 40;
z_target = -75;

[theta1, theta2, theta3, reachable, error_msg] = spider_leg_IK_enhanced(x_target, y_target, z_target, L1, L2, L3);

if reachable
    fprintf('✓ IK Solution Found!\n');
    fprintf('  Target: (%.1f, %.1f, %.1f) mm\n', x_target, y_target, z_target);
    fprintf('  ┌─ θ₁ (Coxa):  %7.2f°\n', theta1);
    fprintf('  ├─ θ₂ (Femur): %7.2f°\n', theta2);
    fprintf('  └─ θ₃ (Tibia): %7.2f°\n', theta3);
    
    [x_fk, y_fk, z_fk] = spider_leg_FK(theta1, theta2, theta3, L1, L2, L3);
    error = sqrt((x_fk-x_target)^2 + (y_fk-y_target)^2 + (z_fk-z_target)^2);
    fprintf('  Verification Error: %.4f mm ✓\n\n', error);
else
    fprintf('❌ Target unreachable: %s\n\n', error_msg);
end

% Interactive Menu
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║         SELECT ANALYSIS TO PERFORM             ║\n');
fprintf('╠════════════════════════════════════════════════╣\n');
fprintf('║  1️⃣  - Workspace Visualization                 ║\n');
fprintf('║  2️⃣  - Walking Gait Animation                  ║\n');
fprintf('║  3️⃣  - Joint Trajectory Analysis               ║\n');
fprintf('║  4️⃣  - Complete Analysis Suite (All Above)     ║\n');
fprintf('╚════════════════════════════════════════════════╝\n');

choice = input('\n👉 Enter your choice (1-4): ');

switch choice
    case 1
        fprintf('\n🎨 Running Workspace Analysis...\n');
        visualize_workspace_fancy(L1, L2, L3);
        
    case 2
        fprintf('\n🎬 Running Gait Animation...\n');
        stride = 60;
        height = 40;
        traj = generate_walking_gait(L1, L2, L3, stride, height);
        animate_spider_leg_cinematic(traj, L1, L2, L3, '🚶 Walking Gait');
        
    case 3
        fprintf('\n📈 Running Trajectory Analysis...\n');
        stride = 60;
        height = 40;
        traj = generate_walking_gait(L1, L2, L3, stride, height);
        plot_joint_trajectories(traj, '🚶 Walking Gait');
        
    case 4
        fprintf('\n🚀 Running COMPLETE Analysis Suite...\n\n');
        
        fprintf('Step 1/3: Workspace Visualization\n');
        visualize_workspace_fancy(L1, L2, L3);
        pause(1);
        
        % 2. Gait trajectory
        fprintf('Step 2/3: Generating Gait Trajectory\n');
        stride = 60;
        height = 40;
        traj = generate_walking_gait(L1, L2, L3, stride, height);
        plot_joint_trajectories(traj, '🚶 Walking Gait');
        pause(1);
        
        % 3. Animation
        fprintf('Step 3/3: Cinematic Animation\n');
        animate_spider_leg_cinematic(traj, L1, L2, L3, '🚶 Walking Gait Simulation');
        
    otherwise
        fprintf('❌ Invalid choice. Running default analysis...\n');
        visualize_workspace_fancy(L1, L2, L3);
end

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                                                                ║\n');
fprintf('║              ✨ ANALYSIS COMPLETE! ✨                          ║\n');
fprintf('║                                                                ║\n');
fprintf('║         Thank you for using Spider Robot IK Suite             ║\n');
fprintf('║                                                                ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');