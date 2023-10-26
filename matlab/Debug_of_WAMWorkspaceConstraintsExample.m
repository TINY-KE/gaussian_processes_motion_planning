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
arm = generateArm('WAMArm');

end_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
start_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';
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
check_inter = 5;
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
disp('sdf dateset ...');
dataset.map(170,220,130:150)
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

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
hold on, view(-103, 60);
title('Result Values')
plotMap3D(dataset.corner_idx, origin, cell_size);
set3DPlotRange(dataset);
hcp = plotRobotModel(arm, start_conf);
hold off


%% debug Point3(0, 0, 0)
origin = [0,0,0];
% origin_point3 = Point3(origin');
origin_point3 = Point3(0,0,0);
% sdf.convertPoint3toCell(origin_point3)

test = [-0.13, -0.14, 0.06];
test_point3 = Point3(test');
test_point3 = Point3(-0.13, -0.14, 0.06);
aaa = sdf.getSignedDistance(test_point3);
aaa;
bbb = sdf.getSignedDistance(Point3(0, 0, 0))
% grad_act


