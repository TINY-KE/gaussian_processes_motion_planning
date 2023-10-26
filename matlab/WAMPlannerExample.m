% 7DOF WAM arm example, with integrated planner
% @author Jing Dong
% @date Jun 15, 2015

close all
clear

import gtsam.*
import gpmp2.*


%% dataset
dataset = generate3Ddataset('WAMDeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% init sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

% arm: WAM arm
% arm = generateArm('WAMArm');
arm = generateArm('JACO2Arm');


% start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
% end_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';
% end_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';  %关节角度的向量
% start_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';   %关节角度的向量

%6 dof
start_conf = [0,0,0,0,0,0]';  %关节角度的向量
end_conf = [-0.0,0.94,1.6,0,-0.919,1.55]';   %关节角度的向量


start_vel = zeros(7,1);
end_vel = zeros(7,1);

% plot problem setting
figure(1), hold on
title('Problem Settings')
plotMap3D(dataset.corner_idx, origin, cell_size);
plotRobotModel(arm, start_conf);
plotRobotModel(arm, end_conf);
% plot config
set3DPlotRange(dataset)
grid on, view(3)
hold off


%% settings
total_time_sec = 2;
total_time_step = 10;
total_check_step = 100;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% GP
Qc = 1 * eye(7);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% algo settings
cost_sigma = 0.02;
epsilon_dist = 0.2;

% noise model
pose_fix_sigma = 0.0001;
vel_fix_sigma = 0.0001;

% plot settings
plot_inter_traj = false;
plot_inter = 4;
if plot_inter_traj
    total_plot_step = total_time_step * (plot_inter + 1);
else
    total_plot_step = total_time_step;
end
pause_time = total_time_sec / total_plot_step;


%% initial traj
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

% % plot initial traj
% if plot_inter_traj
%     plot_values = interpolateArmTraj(init_values, Qc_model, delta_t, plot_inter);
% else
%     plot_values = init_values;
% end
% 
% % plot init values
% figure(3),
% hold on
% title('Initial Values')
% % plot world
% plotMap3D(dataset.corner_idx, origin, cell_size);
% for i=0:total_plot_step
%     % plot arm
%     conf = plot_values.atVector(symbol('x', i));
%     plotRobotModel(arm, conf)
%     % plot config
%     set3DPlotRange(dataset)
%     grid on, view(2)
%     pause(pause_time)
% end
% hold off


%% optimize!
opt_setting = TrajOptimizerSetting(7);
opt_setting.set_total_step(total_time_step); %优化的状态/航点总数。在大多数情况下，>=10 是可以的。
opt_setting.set_total_time(total_time_sec);  %轨迹的总运行时间（以秒为单位）
opt_setting.set_epsilon(epsilon_dist);  %epsilon 是指“安全距离”，如果希望机器人远离障碍物，请使用更大的 \epsilon。
opt_setting.set_cost_sigma(cost_sigma); %\sigma_obs 控制轨迹的“平滑度”和“避障”之间的平衡。较小的\sigma_obs是指轨迹不太平滑，与障碍物碰撞的概率较低。越大\sigma_obs是指轨迹越平滑，但与障碍物碰撞的概率越高。
% ????  为您自己的机器人调整此值，这是唯一需要根据机器人进行调整的参数。

opt_setting.set_obs_check_inter(check_inter);  %通过GP插值检查状态数，0表示不使用GP插值。check_inter 每个机械臂轨迹点之间需要插值的个数
opt_setting.set_conf_prior_model(pose_fix_sigma);  %姿势 先验模型协方差。在大多数情况下，保留默认值 0.0001。
opt_setting.set_vel_prior_model(vel_fix_sigma);  %姿势 先验模型协方差。在大多数情况下，保留默认值 0.0001。
opt_setting.set_Qc_model(Qc);   %???  GP hyperparameter. If this is not known just use identity.????

%优化算法。在大多数情况下，LMA 工作正常。高斯牛顿也可以用于简单的问题。
opt_setting.setDogleg();
% opt_setting.setGaussNewton();
% opt_setting.setLM();
% opt_setting.setVerbosityError();


% other parameter
% rel_thresh：当相对误差减少小于此阈值时停止优化。在大多数情况下，保留默认值 1e-6。如果您希望使优化提前停止，并且可以使用次优解决方案，请将此值变大，例如 1e-3;
% max_iter：优化的最大迭代次数，默认为 100。如果要提前停止算法，或者具有运行时间限制，请将其设置为较小的值。请注意，迭代太少可能不允许收敛到正确的最小值。


tic
result = BatchTrajOptimize3DArm(arm, sdf, start_conf, start_vel, end_conf, ...
    end_vel, init_values, opt_setting);
fprintf('Optimization Time: %f\n', toc)

% result.print('Final results')


%% plot results
if plot_inter_traj
    plot_values = interpolateArmTraj(result, Qc_model, delta_t, plot_inter);
else
    plot_values = result;
end

% plot All Bones
figure(24)
hold on
title('All Bones')
grid on, view(-70, 20)
% plot world
plotMap3D(dataset.corner_idx, origin, cell_size);
for i=0:total_plot_step
    % plot arm
    conf = plot_values.atVector(symbol('x', i));  
    %这段代码中的plot_values是一个由优化器计算出的机械臂轨迹，它是一个GTSAM库中的Values对象，
    % 包含了轨迹在不同时间步骤中的关节角度向量和速度向量等信息。
    % atVector方法是Values对象中的一个方法，用于获取指定符号（symbol）对应的向量值。
    % 在这里，symbol('x', i)生成了一个符号，表示机械臂在第i个时间步骤的关节角度向量，通过atVector方法获取该时间步骤的关节角度向量值。
    % 具体来说，symbol('x', i)返回一个GTSAM库中的Symbol对象，表示符号x_i，然后使用atVector方法获取该符号对应的向量值。
    % 'x'表示机器人状态的类型，i表示时间步骤.
    plotArm(arm.fk_model(), conf, 'b', 2);
    % plot config
    set3DPlotRange(dataset)
    pause(pause_time)
end
hold off

% plot final values
figure(25)
for i=0:total_plot_step
    clf
    hold on, view(-5, 12)
    title('Real Robot')
    % plot world
    plotMap3D(dataset.corner_idx, origin, cell_size);
    % plot arm
    conf = plot_values.atVector(symbol( ...
        ...
        'x', i));
%     plotArm(arm, conf, 'b', 2)
    plotRobotModel(arm, conf);
    % plot config
    set3DPlotRange(dataset)
    grid on
    pause(0.01)
end
hold off



