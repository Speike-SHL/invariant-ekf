%% Set sample times

% Don't change
EtherCat_SampleTime = 1/2000;
VectorNav_SampleTime = 1/800;
EKF_SampleTime = 1/2000;

%% Injected Noise Parameters

% Set inital IMU biases
% 初始IMU bias
gyro_true_bias_init = [0; 0; 0];
% 初始加速度计bias
accel_true_bias_init = [0; 0; 0];

% Set gyroscope noise std
% 增加到陀螺仪观测上的噪声的标准偏差
gyro_true_noise_std = 0.00 * ones(3, 1);
% 增加到陀螺仪bias上的噪声的标准偏差
gyro_true_bias_noise_std = 0.000 * ones(3, 1);

% Set accelerometer noise std
% 增加到加速度计观测上的噪声的标准偏差
accel_true_noise_std = 0.0 * ones(3, 1);
% 增加到加速度计bias上的噪声的标准偏差
accel_true_bias_noise_std = 0.000 * ones(3, 1);

% Landmark position measurement noise
% 增加到地标观测上的噪声的标准偏差
landmark_true_noise_std = 0.00 * ones(3, 1);

% Position of landmark in world frame
% 带有ID的地标位置列表
landmark_positions = [[1; 0; -1; 0], ... % [Landmark_ID; {W}_p_{WL}]
                           [2; 1; 1; -0.5], ...
                           [3; 2; -1; 0.5]];
% landmark_positions = [[1; 0;0;0],... % [Landmark_ID; {W}_p_{WL}]
%                       [2; 1;2;3],...
%                       [3; 4;5;6],...
%                       [4; 7;8;9],...
%                       [5; -7;-8;-9],...
%                       [6; -4;-5;-6],...
%                       [7; -1;-2;-3]];
% 地标观测频率
landmark_measurement_frequency = 1/20;

%% Filter Parameters

% Enable bias estimation and measurement updates
% 标志，使能static bias初始化，其中初始bias从假定base pose保持固定的前几秒数据中获得。对于包含的数据集，将此标志设置为false。
static_bias_initialization = false;
% 标志，使能卡尔曼滤波的更新阶段
ekf_update_enabled = true;
% 标志，使能运动学观测
enable_kinematic_measurements = true;
% 标志，使能地标观测
enable_landmark_measurements = false;
% 标志，使能静态地标。如果为假，则地标位置将与其他状态量一起估计。
enable_static_landmarks = false;

% Set inital IMU biases
% 初始陀螺仪bias估计
gyro_bias_init = [0; 0; 0];
% 初始加速度计bias估计
accel_bias_init = [0; 0; 0];

% Set gyroscope noise std
% 陀螺仪观测噪声的标准偏差
gyro_noise_std = 0.002 * ones(3, 1);
% 陀螺仪bias噪声的标准偏差
gyro_bias_noise_std = 0.001 * ones(3, 1);

% Set accelerometer noise std
% 加速度计观测噪声的标准偏差
accel_noise_std = 0.04 * ones(3, 1);
% 加速度计bias噪声的标准偏差
accel_bias_noise_std = 0.001 * ones(3, 1);

% Set contact and encoder noise std
% contact frame 速度观测噪声的标准偏差
contact_noise_std = 0.05 * ones(3, 1);
% 关节编码器观测噪声的标准偏差
encoder_noise_std = deg2rad(0.5) * ones(14, 1);

% Set landmark measurement noise std
% 地标观测噪声的标准偏差
landmark_noise_std = 0.1 * ones(3, 1);

% Priors
% 初始基座位姿（方向和位置）的标准偏差
prior_base_pose_std = [0.01 * ones(3, 1); 0.01 * ones(3, 1)];
% 初始基座速度的标准偏差
prior_base_velocity_std = 0.1 * ones(3, 1);
% 初始接触位置的标准偏差
prior_contact_position_std = 1.0 * ones(3, 1);
% 初始陀螺仪bias的标准偏差
prior_gyro_bias_std = 0.01 * ones(3, 1);
% 初始加速度计bias的标准偏差
prior_accel_bias_std = 0.1 * ones(3, 1);
% 增加到正向运动学观测协方差的附加噪声项
prior_forward_kinematics_std = 0.03 * ones(3, 1);
