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
arm = generateArm('rm_75');

start_conf = [-3.084730575741016, -1.763304599691998, 1.8552083929655296, 0.43301604856981246, -2.672461979658843, 0.46925065047728776, 4.000864693936108]';
end_conf = [-2.018431036907354, -1.4999897089911451, 1.4046777996889483, -1.3707039693409548, -3.0999924865261397, -0.8560425214202461, 4.8622166345079165]';

start_vel = zeros(arm.dof(),1);
end_vel = zeros(arm.dof(),1);

jposes = arm.fk_model().forwardKinematicsPose(start_conf);
traj_orien = Rot3.Ypr(jposes(1,end),jposes(2,end),jposes(3,end));
jposes = arm.fk_model().forwardKinematicsPose(end_conf);
end_pose = Pose3(Rot3.Ypr(jposes(1,end),jposes(2,end),jposes(3,end)), ...
    Point3(jposes(4,end),jposes(5,end),jposes(6,end)));

%% settings
total_time_sec = 2;
total_time_step = 10;
check_inter = 10;
delta_t = total_time_sec / total_time_step;
total_check_step = (check_inter + 1)*total_time_step;

% GP
Qc = 0.1 * eye(arm.dof());
Qc_model = noiseModel.Gaussian.Covariance(Qc);

% algo settings
obs_sigma = 0.005;
epsilon_dist = 0.15;
fix_sigma = 1e-4;
end_pose_sigma = 1e-4;
orien_sigma = 1e-2;

%% sdf
dataset = generate3Ddataset('WAMDeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

% save sdf field to txt file
% % diary('/home/zhjd/ws_3d_vp/src/view_planning/txt_sdf/sdf.txt')
% filename = '/home/zhjd/ws_3d_vp/src/view_planning/txt_sdf/sdf.txt';
% fid = fopen('/home/zhjd/ws_3d_vp/src/view_planning/txt_sdf/sdf.txt', 'w');
% % dlmwrite(filename, data);
% 
% for i = 1:dataset.rows
%     for j = 1:dataset.cols
%         for k = 1:dataset.z
%             % formatString = ['%.', num2str(decimalPlaces), 'f\n'];
%             % fprintf(formatString, value);
%             % 指定输出的数字长度为8
%             numberLength = 8;
%             % 将value转换为字符串，并指定输出的长度
%             valueString = num2str(field(i,j,k), numberLength);
%             fprintf(fid, '%s\t', valueString);
%         end
%         fprintf(fid, '\n');  % mei shu pai, huang yi hang
%     end
% end

% sdf 2
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

% DEBUG
p = Point3(-0.13, -0.14, 0.06);
signed_distance = sdf.getSignedDistance(p)
point = [-0.13, -0.14, 0.06]; p = Point3(point');
signed_distance = sdf.getSignedDistance(p)
p = Point3(0.18, 0.12, 0.01);
% grad_act = [0,0,0];
% signed_distance = sdf.getSignedDistance(p,grad_act)
% grad_act


%% plot
plot_inter = check_inter;
if plot_inter
    total_plot_step = total_time_step * (plot_inter + 1);
else
    total_plot_step = total_time_step;
end
pause_time = total_time_sec / total_plot_step;

% plot environment and robot
h = figure(1); clf(1);
set(h, 'Position', [-1200, 100, 1100, 1200]);
hold on, view(-103, 90);
title('Result Values')
plotMap3D(dataset.corner_idx, origin, cell_size);
set3DPlotRange(dataset);
hcp = plotRobotModel(arm, start_conf);
hold off

%% initial traj and construct factor graph
% init_values = initArmTrajStraightLine(start_conf, start_conf, total_time_step);
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
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
% elseif use_trustregion_opt
%     parameters = DoglegParams;
%     parameters.setVerbosity('ERROR');
%     optimizer = DoglegOptimizer(graph, init_values, parameters);
% else
%     parameters = GaussNewtonParams;
%     parameters.setVerbosity('ERROR');
%     optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end
tic
optimizer.optimize();
toc
result = optimizer.values()

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

% plot final values
for i=0:total_plot_step
    figure(1);
    hold on;
    conf = plot_values.atVector(symbol('x', i));
    delete(hcp); hcp = plotRobotModel(arm, conf);
    hold off;
    pause(1e-10);
end
