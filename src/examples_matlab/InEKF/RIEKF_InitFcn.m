%% Set sample times

% Don't change
EtherCat_SampleTime = 1/2000;
VectorNav_SampleTime = 1/800;
EKF_SampleTime = 1/2000;

%% Injected Noise Parameters

% Set inital IMU biases
% ��ʼIMU bias
gyro_true_bias_init = [0; 0; 0];
% ��ʼ���ٶȼ�bias
accel_true_bias_init = [0; 0; 0];

% Set gyroscope noise std
% ���ӵ������ǹ۲��ϵ������ı�׼ƫ��
gyro_true_noise_std = 0.00 * ones(3, 1);
% ���ӵ�������bias�ϵ������ı�׼ƫ��
gyro_true_bias_noise_std = 0.000 * ones(3, 1);

% Set accelerometer noise std
% ���ӵ����ٶȼƹ۲��ϵ������ı�׼ƫ��
accel_true_noise_std = 0.0 * ones(3, 1);
% ���ӵ����ٶȼ�bias�ϵ������ı�׼ƫ��
accel_true_bias_noise_std = 0.000 * ones(3, 1);

% Landmark position measurement noise
% ���ӵ��ر�۲��ϵ������ı�׼ƫ��
landmark_true_noise_std = 0.00 * ones(3, 1);

% Position of landmark in world frame
% ����ID�ĵر�λ���б�
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
% �ر�۲�Ƶ��
landmark_measurement_frequency = 1/20;

%% Filter Parameters

% Enable bias estimation and measurement updates
% ��־��ʹ��static bias��ʼ�������г�ʼbias�Ӽٶ�base pose���̶ֹ���ǰ���������л�á����ڰ��������ݼ������˱�־����Ϊfalse��
static_bias_initialization = false;
% ��־��ʹ�ܿ������˲��ĸ��½׶�
ekf_update_enabled = true;
% ��־��ʹ���˶�ѧ�۲�
enable_kinematic_measurements = true;
% ��־��ʹ�ܵر�۲�
enable_landmark_measurements = false;
% ��־��ʹ�ܾ�̬�رꡣ���Ϊ�٣���ر�λ�ý�������״̬��һ����ơ�
enable_static_landmarks = false;

% Set inital IMU biases
% ��ʼ������bias����
gyro_bias_init = [0; 0; 0];
% ��ʼ���ٶȼ�bias����
accel_bias_init = [0; 0; 0];

% Set gyroscope noise std
% �����ǹ۲������ı�׼ƫ��
gyro_noise_std = 0.002 * ones(3, 1);
% ������bias�����ı�׼ƫ��
gyro_bias_noise_std = 0.001 * ones(3, 1);

% Set accelerometer noise std
% ���ٶȼƹ۲������ı�׼ƫ��
accel_noise_std = 0.04 * ones(3, 1);
% ���ٶȼ�bias�����ı�׼ƫ��
accel_bias_noise_std = 0.001 * ones(3, 1);

% Set contact and encoder noise std
% contact frame �ٶȹ۲������ı�׼ƫ��
contact_noise_std = 0.05 * ones(3, 1);
% �ؽڱ������۲������ı�׼ƫ��
encoder_noise_std = deg2rad(0.5) * ones(14, 1);

% Set landmark measurement noise std
% �ر�۲������ı�׼ƫ��
landmark_noise_std = 0.1 * ones(3, 1);

% Priors
% ��ʼ����λ�ˣ������λ�ã��ı�׼ƫ��
prior_base_pose_std = [0.01 * ones(3, 1); 0.01 * ones(3, 1)];
% ��ʼ�����ٶȵı�׼ƫ��
prior_base_velocity_std = 0.1 * ones(3, 1);
% ��ʼ�Ӵ�λ�õı�׼ƫ��
prior_contact_position_std = 1.0 * ones(3, 1);
% ��ʼ������bias�ı�׼ƫ��
prior_gyro_bias_std = 0.01 * ones(3, 1);
% ��ʼ���ٶȼ�bias�ı�׼ƫ��
prior_accel_bias_std = 0.1 * ones(3, 1);
% ���ӵ������˶�ѧ�۲�Э����ĸ���������
prior_forward_kinematics_std = 0.03 * ones(3, 1);
