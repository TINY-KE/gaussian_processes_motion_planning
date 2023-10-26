% 7DOF WAM arm example
% use workspace orientation constraint for end effector to stay horizontal
% use workspace pose constraint for end effector goal
% @author Mustafa Mukadam

close all
clear
clc

import gtsam.*
import gpmp2.*

%% arm: WAM arm
rot = Rot3(0, 0, 1, ...
            0, 1, 0, ...
            -1, 0, 0);         
base_pose = Pose3(Rot3(eye(3)), Point3([0,0,0]'));
arm = generateArm('WAMArm', base_pose);

%% 定义机械臂的起始和结束姿态 start_conf 和 end_conf，
% 以及起始和结束速度 start_vel 和 end_vel。
% start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';  %关节角度的向量
% end_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';   %关节角度的向量
end_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';  %关节角度的向量
start_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';   %关节角度的向量

start_vel = zeros(arm.dof(),1);
end_vel = zeros(arm.dof(),1);

%% 使用机械臂对象的正向运动学函数,计算 起始姿态 对应的末端执行器位置和方向jposes，
jposes = arm.fk_model().forwardKinematicsPose(start_conf);
%然后，通过旋转矩阵 Rot3.Ypr 和位置点 Point3 创建结束姿态的 Pose3 对象 end_pose，
% 计算末端执行器期望方向 traj_orien。目标是希望，方向始终与初始方向，相同
traj_orien = Rot3.Ypr(jposes(1,end),jposes(2,end),jposes(3,end));

%% 使用机械臂对象的正向运动学函数, 计算 结束姿态 对应的末端执行器位置和方向jposes，
jposes = arm.fk_model().forwardKinematicsPose(end_conf);
end_pose = Pose3(Rot3.Ypr(jposes(1,end),jposes(2,end),jposes(3,end)), ...
    Point3(jposes(4,end),jposes(5,end),jposes(6,end)));

%% settings 
% 设定总时间、总步数、每隔多少步检测一次障碍物、时间步长以及检查障碍物的总步数。
total_time_sec = 2;
total_time_step = 10;
check_inter = 5;
delta_t = total_time_sec / total_time_step;
total_check_step = (check_inter + 1)*total_time_step;

% GP. 
% 创建运动模型噪声协方差矩阵 Qc，并将其包装成一个高斯噪声模型 Qc_model。
Qc = 0.1 * eye(arm.dof());
Qc_model = noiseModel.Gaussian.Covariance(Qc);


% algo settings  

obs_sigma = 0.005;  %观测噪声方差，用于表示机械臂传感器测量噪声的大小 ????
epsilon_dist = 0.15;  %epsilon距离，是机械臂轨迹的一种限制条件，用于防止机械臂与障碍物过于接近；
fix_sigma = 1e-4; %固定噪声方差，用于表示机械臂起始状态的不确定性；
end_pose_sigma = 1e-4; %机械臂末端执行器姿态先验因子的噪声方差
orien_sigma = 1e-2; %机械臂执行器方向先验因子的噪声方差

%% sdf
% 加载包含三维地图和相关参数的数据集，并计算出这个数据集的有符号距离场（SDF SignedDistanceField）。
% SDF 是一种用于快速检测机器人与障碍物之间距离和碰撞的数据结构。
dataset = generate3Ddataset('WAMDeskDataset');
% 这个原点坐标是指3D数据集中的原点在世界坐标系中的位置。这个变量后面会被用来初始化SDF类的实例对象。
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z]; 
origin_point3 = Point3(origin');
%cell_size表示每个体素的大小，是3D数据集中体素的边长。
cell_size = dataset.cell_size;

% sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);  %gpmp2库中的signedDistanceField3D函数.
% field，表示每个体素到最近障碍物的有符号距离
disp('calculating signed distance field done');
sdf = SignedDistanceField(origin_point3, cell_size, ...
    ...   %实例对象中每个维度的体素数（x、y和z方向的体素数）
    size(field, 1),     size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');  %这里的z-1是因为initFieldData函数的第一个参数是层数，从0开始计数。
end

%% plot
plot_inter = check_inter;
if plot_inter
    total_plot_step = total_time_step * (plot_inter + 1);
else
    total_plot_step = total_time_step;
end
pause_time = total_time_sec / total_plot_step;  % no use

% plot environment and robot
h = figure(1); clf(1);
set(h, 'Position', [-1200, 100, 1100, 1200]);
hold on, view(-103, 60); %将图形窗口的绘图模式设置为“保持”，并设置视角；
title('Result Values')
plotMap3D(dataset.corner_idx, origin, cell_size);  %在图形窗口中绘制3D数据集的立方体边界；
set3DPlotRange(dataset);
hcp = plotRobotModel(arm, start_conf);
hold off

%% initial traj and construct factor graph
% init_values用于表示机械臂在轨迹规划开始之前的初始状态. 将init_values作为起点，通过优化算法逐步调整机械臂的关节角度，最终得到一条满足约束条件和目标函数的机械臂轨迹。
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);  %this parogram: total_time_step = 10;


%PriorFactorVector(size_t key, Vector prior, Base noiseModel);
% 它接受三个参数，分别是向量值变量的符号关键字 key、向量值变量的先验均值 prior，以及向量值变量的先验噪声模型 noiseModel。
% PriorFactor(Key key, const VALUE& prior, const Matrix& covariance) :
%       Base(noiseModel::Gaussian::Covariance(covariance), key), prior_(prior) {
%     }

% PriorFactor(Key key, const VALUE& prior, const SharedNoiseModel& model) :
%       Base(model, key), prior_(prior) {
%     }


% graph
graph = NonlinearFactorGraph;
for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);

    if i==0
        % fix start state
        graph.add(PriorFactorVector(key_pos, start_conf, ...
            noiseModel.Isotropic.Sigma(arm.dof(), fix_sigma)));
        
        graph.add(PriorFactorVector(key_vel, start_vel, ...
            noiseModel.Isotropic.Sigma(arm.dof(), fix_sigma)));
    elseif i==total_time_step
        % goal pose for end effector in workspace
        graph.add(GaussianPriorWorkspacePoseArm(key_pos, arm, arm.dof()-1, ...
            end_pose, noiseModel.Isotropic.Sigma(6, end_pose_sigma)));
        % fix goal velocity
        graph.add(PriorFactorVector(key_vel, end_vel, ...
            noiseModel.Isotropic.Sigma(arm.dof(), fix_sigma)));
    else
        % fix end effector orientation in workspace to be horizontal
        graph.add(GaussianPriorWorkspaceOrientationArm(key_pos, arm, arm.dof()-1, ...
            traj_orien, noiseModel.Isotropic.Sigma(3, orien_sigma)));
    end
    
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        % GP prior
        graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, ...
            key_pos2, key_vel2, delta_t, Qc_model));
        % unary obstacle factor
        graph.add(ObstacleSDFFactorArm( ...
            key_pos, arm, sdf, obs_sigma, epsilon_dist));
        % interpolated obstacle factor
        if check_inter
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstacleSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, sdf, obs_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end

%% optimize
use_LM = true;
use_trustregion_opt = false;
if use_LM
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('ERROR');
    parameters.setlambdaInitial(1000.0);
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
    % how to use optimizer in c++:  optimize(graph, init_values, setting);
elseif use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end
tic
optimizer.optimize();
toc
result = optimizer.values();

%% results
if plot_inter
    plot_values = interpolateArmTraj(result, Qc_model, delta_t, plot_inter);
else
    plot_values = result;
end
opt_setting = TrajOptimizerSetting(arm.dof());
opt_setting.set_total_step(total_time_step);
opt_setting.set_total_time(total_time_sec);
opt_setting.set_epsilon(epsilon_dist);
opt_setting.set_cost_sigma(obs_sigma);
opt_setting.set_obs_check_inter(check_inter);
opt_setting.set_conf_prior_model(noiseModel.Isotropic.Sigma(arm.dof(), fix_sigma));
opt_setting.set_vel_prior_model(noiseModel.Isotropic.Sigma(arm.dof(), fix_sigma));
opt_setting.set_Qc_model(Qc);
% check for collision
if CollisionCost3DArm(arm, sdf, plot_values, opt_setting)
    disp('Trajectory is in collision!');
else
    disp('Trajectory is collision free.');
end

% % plot final values
% for i=0:total_plot_step
%     figure(1);
%     hold on;
%     conf = plot_values.atVector(symbol('x', i));
%     delete(hcp); hcp = plotRobotModel(arm, conf);
%     hold off;
%     pause(1e-10);
% end

% plot All Bones
figure(4)
hold on
title('All Bones')
grid on, view(-70, 70)
% plot world
plotMap3D(dataset.corner_idx, origin, cell_size);
for i=0:total_plot_step
    % plot arm
    conf = plot_values.atVector(symbol('x', i));
    plotArm(arm.fk_model(), conf, 'b', 2);
    % plot config
    set3DPlotRange(dataset)
    pause(pause_time)
end
hold off

% plot final values
figure(5)
for i=0:total_plot_step
    clf
    hold on, view(-103, 60)
    title('Real Robot')
    % plot world
    plotMap3D(dataset.corner_idx, origin, cell_size);
    % plot arm
    conf = plot_values.atVector(symbol('x', i));
%     plotArm(arm, conf, 'b', 2)
    plotRobotModel(arm, conf);
    % plot config
    set3DPlotRange(dataset)
    grid on
    pause(1e-11)
end
hold off
