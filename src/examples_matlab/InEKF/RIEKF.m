%--------------------------------------------------------------------------
%  Copyright 2018, Ross Hartley
%  All Rights Reserved
%  See LICENSE for the license information
%--------------------------------------------------------------------------

%   @file   RIEKF.m
%   @author Ross Hartley
%   @brief  Matlab system object to implement contact-aided invariant extended Kalman filtering
%   @date   September 25, 2018

% State Estimator
% classdef RIEKF < matlab.System & matlab.system.mixin.Propagates %#codegen
classdef RIEKF < matlab.System
    %% Properties =========================================================

    % PUBLIC PROPERTIES
    properties

        % 以下参数将影响过滤器的运行方式:
        % //HACK 为什么初始化为false,这里又变为了true
        static_bias_initialization = true; % 标志，使能static bias初始化，其中初始bias从假定base pose保持固定的前几秒数据中获得。对于包含的数据集，将此标志设置为false。
        ekf_update_enabled = true; % 标志，使能卡尔曼滤波的更新阶段
        enable_kinematic_measurements = false; % 标志，使能运动学观测
        enable_landmark_measurements = false; % 标志，使能地标观测
        enable_static_landmarks = false; % 标志，使能静态地标。如果为假，则地标位置将与其他状态量一起估计。

        % 以下参数将影响用于过程和测量模型的初始条件和协方差：
        gyro_bias_init = zeros(3, 1); % 初始陀螺仪bias估计
        accel_bias_init = zeros(3, 1); % 初始加速度计bias估计
        gyro_noise_std = 0.1 * ones(3, 1); % 陀螺仪观测噪声的标准偏差
        gyro_bias_noise_std = 0.1 * ones(3, 1); % 陀螺仪bias噪声的标准偏差
        accel_noise_std = 0.1 * ones(3, 1); % 加速度计观测噪声的标准偏差
        accel_bias_noise_std = 0.1 * ones(3, 1); % 加速度计bias噪声的标准偏差
        contact_noise_std = 0.1 * ones(3, 1); % contact frame 速度观测噪声的标准偏差
        encoder_noise_std = 0.1 * ones(14, 1); % 关节编码器观测噪声的标准偏差
        landmark_noise_std = 0.1 * ones(3, 1); % 地标观测噪声的标准偏差

        %以下参数设置状态估计的初始协方差:
        prior_base_pose_std = 0.1 * ones(6, 1); % 初始基座位姿（方向和位置）的标准偏差
        prior_base_velocity_std = 0.1 * ones(3, 1); % 初始基座速度的标准偏差
        prior_contact_position_std = 0.1 * ones(3, 1); % 初始接触位置的标准偏差
        prior_gyro_bias_std = 0.1 * ones(3, 1); % 初始陀螺仪bias的标准偏差
        prior_accel_bias_std = 0.1 * ones(3, 1); % 初始加速度计bias的标准偏差
        prior_forward_kinematics_std = 0.1 * ones(3, 1); % 增加到正向运动学观测协方差的附加噪声项

        % //HACK 为什么不带id了
        landmark_positions = [0; 0; 0]; % 静态地标位置
    end

    % PRIVATE PROPERTIES
    properties (Access = private)
        X; % 状态
        theta; % bias状态
        P; % 误差协方差
        filter_enabled; % 使能滤波器
        bias_initialized;
        ba0 = zeros(3, 1); % 加速度计bias
        bg0 = zeros(3, 1); % 陀螺仪bias
        a_init_vec; % 累计前imu_init_total_count帧的加速度数据
        w_init_vec; % 累计前imu_init_total_count帧的角速度数据
        imu_init_count = 1; % imu初始化帧计数,是否达到imu_init_total_count
        w_prev; % 上一步传入的角速度值
        a_prev; % 上一步传入的加速度值
        encoders_prev; % 上一步传入的编码器值
        contact_prev; % 上一步传入的contact值
        t_prev; % 上一步的仿真时间,由EtherCat_SampleTime确定
        landmark_ids; % 地标序号

        % Sensor Covariances
        Qg; % Gyro Covariance Matrix 3x3
        Qbg; % Gyro bias Covariance Matrix 3x3
        Qa; % Accel Covariance Matrix 3x3
        Qba; % Accel Bias Covariance Matrix 3x3
        Qc; % Contact Covariance Matrix 3x3
        Qe; % Encoder Covariance Matrix 14x14
        Ql; % Landmark Distance Covariance Matrix 3x3
        Np; % Prior Forward Kinematics Covariance Matrix 3x3
        P_prior; % 增广的先验误差协方差矩阵,diag([R,v,p,d1,d2,bg,ba]),21x21

    end

    % PRIVATE CONSTANTS
    properties (Access = private, Constant)
        % EKF Noise Parameters
        g = [0; 0; -9.81]; % Gravity
        imu_init_total_count = 1000; % 使用EtherCat_SampleTime的前1000步用来初始化imu bias
    end

    %% PROTECTED METHODS ==================================================
    methods (Access = protected)

        % 初始化 System object
        function setupImpl(obj)
            obj.filter_enabled = false;
            obj.bias_initialized = false;
            obj.a_init_vec = zeros(3, obj.imu_init_total_count);
            obj.w_init_vec = zeros(3, obj.imu_init_total_count);

            % Initialze State and Covariance
            obj.X = eye(7); % R+v+p+d1+d2 = [3x3 3x1 3x1 3x1 3x1]
            obj.theta = zeros(6, 1); % bg+ba = [3x1 ; 3x1]
            obj.P = eye(21);

            % 初始化传感器协方差, 系统噪声协方差为Q, 观测噪声协方差为N, 误差协方差为P
            obj.Qg = diag(obj.gyro_noise_std .^ 2); % Gyro Covariance Matrix        3x3
            obj.Qbg = diag(obj.gyro_bias_noise_std .^ 2); % Gyro bias Covariance Matrix   3x3
            obj.Qa = diag(obj.accel_noise_std .^ 2); % Accel Covariance Matrix       3x3
            obj.Qba = diag(obj.accel_bias_noise_std .^ 2); % Accel Bias Covariance Matrix  3x3
            obj.Qc = diag(obj.contact_noise_std .^ 2); % Contact Covariance Matrix     3x3
            obj.Qe = diag(obj.encoder_noise_std .^ 2); % Encoder Covariance Matrix     14x14
            obj.Ql = diag(obj.landmark_noise_std .^ 2); % Landmark Covariance Matrix    3x3
            obj.Np = diag(obj.prior_forward_kinematics_std .^ 2); % Prior Forward Kinematics Covariance Matrix 3x3
            obj.P_prior = blkdiag(diag(obj.prior_base_pose_std(1:3) .^ 2), ... % base orientation Covariance 3x3
                diag(obj.prior_base_velocity_std .^ 2), ... % base velocity Covariance 3x3
                diag(obj.prior_base_pose_std(4:6) .^ 2), ... % base position Covariance 3x3
                diag(obj.prior_contact_position_std .^ 2), ... % 同时有两个足端接触 6x6
                diag(obj.prior_contact_position_std .^ 2), ...
                diag(obj.prior_gyro_bias_std .^ 2), ... % 陀螺仪和加速度计bias 6x6
                diag(obj.prior_accel_bias_std .^ 2)); % 共21X21

            % Initialize bias estimates
            obj.bg0 = obj.gyro_bias_init;
            obj.ba0 = obj.accel_bias_init;

            % Variables to store previous measurements
            obj.w_prev = zeros(3, 1);
            obj.a_prev = zeros(3, 1);
            obj.encoders_prev = zeros(14, 1);
            obj.contact_prev = zeros(2, 1);
            obj.t_prev = 0;

            % Landmark
            obj.landmark_ids = [];

        end % setupImpl

        % 系统输出和状态更新方程
        function [X, theta, P, enabled, landmark_ids] = stepImpl(obj, enable, t, w, a, encoders, contact, landmarks, X_init)
            % 执行初始化,运行滤波器并保存结果
            %
            %   Inputs:
            %       enable      - flag to enable/disable the filter
            %       t           - time
            %       w           - angular velocity, {I}_w_{WI}
            %       a           - linear acceleration, {I}_a_{WI}
            %       encoders    - joint encoder positions
            %       contact     - contact indicator
            %       landmarks   - vector of landmark IDs along with distance
            %       X_init      - initial state
            %
            %   Outputs:
            %       X           - current state estimate
            %       theta       - current parameter estimates
            %       P           - current covariance estimate
            %       enabled     - flag indicating when the filter is enabled
            %       landmark_ids-
            %
            %   Author: Ross Hartley
            %   Date:   1/19/2018

            %             %%% FILTER TEST
            %             obj.X = eye(7);
            %             obj.P = eye(21);
            %
            %             landmarks = zeros(4,3);
            %             landmarks(:,1) = [1; 1;2;3];
            %             landmarks(:,2) = [2; 4;5;6];
            %             landmarks(:,3) = [3; 7;8;9];
            %             obj.Ql = 0.1^2*eye(3);
            % %             obj.Update_StaticLandmarks(landmarks);
            %             obj.Update_Landmarks(landmarks);
            %
            %             m = [1;2;3;4;5;6];
            %             dt = 0.1;
            %             obj.Qg = 0.01^2*eye(3);
            %             obj.Qa = 0.1^2*eye(3);
            %             obj.Qbg = 0.00001^2*eye(3);
            %             obj.Qba = 0.0001^2*eye(3);
            %             obj.Predict_State(m(1:3), m(4:6), zeros(14,1), zeros(2,1), dt);
            %             obj.Predict_State(m(1:3), m(4:6), zeros(14,1), zeros(2,1), dt);
            %
            % %             obj.Update_StaticLandmarks(landmarks);
            %             obj.Update_Landmarks(landmarks);
            %             %%%%%%

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %simulink中固定了landmarks的大小，这里是删除landmarks中的全零列
            landmarks(:, ~all(landmarks)) = [];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Initialize bias
            % (does nothing if bias is already initialized)
            obj.InitializeBias(w, a, X_init)

            % Initiaze filter
            % (does nothing if filter is already initialized)
            % any(contact == 1) 任意一只脚触地时初始化
            if t > 0.01 && any(contact == 1)
                obj.InitializeFilter(enable, X_init);
            end

            % Only run if filter is enabled
            if obj.filter_enabled

                % Predict state using IMU and contact measurements
                obj.Predict_State(obj.w_prev, obj.a_prev, obj.encoders_prev, obj.contact_prev, t - obj.t_prev);

                % Update using other measurements
                if obj.ekf_update_enabled

                    % Update state using forward kinematic measurements
                    if obj.enable_kinematic_measurements
                        obj.Update_ForwardKinematics(encoders, contact);
                    end

                    % Update state using landmark position measurements
                    if obj.enable_landmark_measurements && ~isempty(landmarks)

                        if obj.enable_static_landmarks
                            obj.Update_StaticLandmarks(landmarks);
                        else
                            obj.Update_Landmarks(landmarks);
                        end

                    end

                end

            end

            % 储存上步传入的值
            obj.w_prev = w;
            obj.a_prev = a;
            obj.encoders_prev = encoders;
            obj.contact_prev = contact;
            obj.t_prev = t;

            % 输出结果到输出端口
            X = obj.X;
            theta = obj.theta;
            P = obj.P(1:21, 1:21);
            enabled = double(obj.filter_enabled);
            landmark_ids = obj.landmark_ids;

        end % stepImpl

        % 重置 System object 状态
        function resetImpl(~)
        end % resetImpl

        % 输出端口的大小
        function [X, theta, P, enabled, landmark_ids] = getOutputSizeImpl(~)
            X = [7 + 10, 7 + 10]; % //HACK R+v+p+d1+d2 = 3x3+3x1+3x1+3x1+3x1, 为什么加10, 地标？
            theta = [6, 1]; % ba+bg = [3x1;3x1]
            P = [21 + 3 * 10, 21 + 3 * 10]; % //HACK 为什么加10
            enabled = [1, 1]; % //HACK enabled是什么,是否使用了滤波器？
            landmark_ids = [1, 10]; % landmarks 10个
        end % getOutputSizeImpl

        % 输出端口的数据类型
        function [X, theta, P, enabled, landmark_ids] = getOutputDataTypeImpl(~)
            X = 'double';
            theta = 'double';
            P = 'double';
            enabled = 'double';
            landmark_ids = 'double';
        end % getOutputDataTypeImpl

        % 输出端口的复/实性
        function [X, theta, P, enabled, landmark_ids] = isOutputComplexImpl(~)
            X = false;
            theta = false;
            P = false;
            enabled = false;
            landmark_ids = false;
        end % isOutputComplexImpl

        % 固定大小或可变大小输出端口
        function [X, theta, P, enabled, landmark_ids] = isOutputFixedSizeImpl(~)
            X = false;              % 可变大小
            theta = true;           % 固定大小
            P = false;              % 可变大小
            enabled = true;         % 固定大小
            landmark_ids = false;   % 可变大小
        end % isOutputFixedSizeImpl

    end

    %% PRIVATE METHODS(自定义方法) ========================================
    methods (Access = private)

        % 分离状态向量X=[R,v,p,dr,dl,(lm)]和theta=[bg;ba]
        function [R, v, p, dR, dL, lm, bg, ba] = Separate_State(~, X, theta)
            R = X(1:3, 1:3);    % Orientation
            v = X(1:3, 4);      % Base Velocity
            p = X(1:3, 5);      % Base Position
            dR = X(1:3, 6);     % Right Foot Position
            dL = X(1:3, 7);     % Left Foot Position

            % Landmark Positions (if they exist, 即X的维度大于7)
            if size(X, 2) > 7
                lm = X(1:3, 8:end);
            else
                lm = [];
            end

            bg = theta(1:3); % Gyroscope Bias
            ba = theta(4:6); % Accelerometer Bias
        end

        % 构建状态向量X,theta
        function [X, theta] = Construct_State(~, R, v, p, dR, dL, lm, bg, ba)
            X = eye(7 + size(lm, 2));
            X(1:7, 1:7) = [R, v, p, dR, dL; 0, 0, 0, 1, 0, 0, 0; 0, 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 0, 1];
            % Add Landmarks
            if ~isempty(lm)
                X(1:3, 8:end) = lm;
            end

            theta = [bg; ba];
        end

        % 构建3x1向量的反对称阵
        function [A] = skew(~, v)
            % Convert from vector to skew symmetric matrix
            A = [0, -v(3), v(2);
                 v(3), 0, -v(1);
                 -v(2), v(1), 0];
        end

        % 指数映射//HACK
        function [dX, dtheta] = exp(obj, v)
            % Exponential map of SE_N(3)
            N = (length(v) - 3) / 3;
            Lg = zeros(N + 3);
            Lg(1:3, :) = [obj.skew(v(1:3)), reshape(v(4:end), 3, [])];
            dX = expm(Lg);
            dtheta = v(end - 5:end);
        end

        % 构造X的反对称矩阵[R v p dr dl] ->
        % [R 0 0 0 0; v^R R 0 0 0; p^R 0 R 0 0; dr^R 0 0 R 0; dl^R 0 0 0 R]
        function AdjX = Adjoint(obj, X)
            % Adjoint of SE_N(3)
            N = size(X, 2) - 3;
            R = X(1:3, 1:3);
            R_cell = repmat({R}, 1, N + 1);
            AdjX = blkdiag(R_cell{:});

            for i = 1:N
                AdjX(3 * i + 1:3 * i + 3, 1:3) = obj.skew(X(1:3, i + 3)) * R;
            end

        end

        % 前imu_init_total_count步累积imu数据来计算bias
        function [] = InitializeBias(obj, w, a, X_init)
            % Function to initiaze IMU bias
            if obj.static_bias_initialization

                if ~obj.bias_initialized

                    if norm(a) > 0 && norm(w) > 0 % Wait for valid signal
                        % 前imu_init_total_count帧累积imu数据
                        if obj.imu_init_count <= obj.imu_init_total_count
                            Rwi = X_init(1:3, 1:3);
                            % a_i = R_wi'*(R_wi*a_i + g)得到去除重力加速度的imu系下的加速度值
                            obj.a_init_vec(:, obj.imu_init_count) = Rwi' * (Rwi * a + obj.g);
                            obj.w_init_vec(:, obj.imu_init_count) = w;
                            obj.imu_init_count = obj.imu_init_count + 1;
                        else
                            % 使用前imu_init_total_count步累计的imu数据初始化bias,计算均值作为bias,因为bias在均值附近游走
                            obj.ba0 = mean(obj.a_init_vec, 2);
                            obj.bg0 = mean(obj.w_init_vec, 2);
                            obj.bias_initialized = true;
                        end

                    end

                end

            else
                obj.bias_initialized = true;
            end

        end

        % 初始化滤波器的X,theta和P,若enable为0,则用默认0和对角阵1
        function [] = InitializeFilter(obj, enable, X_init)
            % Attempt to enable filter (successful if enable is true, and
            % at least one foot is on the ground)
            if enable && ~obj.filter_enabled
                obj.X = X_init;
                obj.theta = [obj.bg0; obj.ba0];
                obj.P = obj.P_prior;
                obj.filter_enabled = true;
            end

            % If filter is disabled, zero everything
            if ~enable || ~obj.filter_enabled
                obj.X = eye(7);
                obj.theta = zeros(6, 1);
                obj.P = eye(21);
                obj.filter_enabled = false;
            end

        end

        % 右不变扩展卡尔曼滤波器预测步骤
        function [] = Predict_State(obj, w, a, encoders, contact, dt)

            [R, v, p, dR, dL, lm, bg, ba] = obj.Separate_State(obj.X, obj.theta);

            % bias修正后的IMU的角速度和加速度
            w_k = w - bg; % {I}_w_{WI}
            a_k = a - ba; % {I}_a_{WI}

            % Base Pose Dynamics
            R_pred = R * expm(obj.skew(w_k * dt));
            v_pred = v + (R * a_k + obj.g) * dt;
            p_pred = p + v * dt + 0.5 * (R * a_k + obj.g) * dt ^ 2;

            % Foot Position Dynamics //HACK
            dR_off = p_pred + R_pred * p_VectorNav_to_RightToeBottom(encoders); % {W}_p_{WR}
            dL_off = p_pred + R_pred * p_VectorNav_to_LeftToeBottom(encoders); % {W}_p_{WL}
            dR_pred = contact(2) * dR + (1 - contact(2)) * dR_off;
            dL_pred = contact(1) * dL + (1 - contact(1)) * dL_off;

            % Landmark Dynamics
            lm_pred = lm;

            % Bias Dynamics
            bg_pred = bg;
            ba_pred = ba;

            % -- Linearized invariant error dynamics --

            % Base
            Fc = [zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
                  obj.skew(obj.g), zeros(3), zeros(3), zeros(3), zeros(3);
                  zeros(3), eye(3), zeros(3), zeros(3), zeros(3);
                  zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
                  zeros(3), zeros(3), zeros(3), zeros(3), zeros(3)];
            % Landmarks
            Fc = blkdiag(Fc, zeros(3 * length(obj.landmark_ids)));
            % Parameters
            Fc = blkdiag(Fc, zeros(6));
            Fc(1:15, end - 5:end) = [-R, zeros(3);
                                     -obj.skew(v) * R, -R;
                                     -obj.skew(p) * R, zeros(3);
                                     -obj.skew(dR) * R, zeros(3);
                                     -obj.skew(dL) * R, zeros(3)];

            for i = 1:length(obj.landmark_ids)
                Fc(15 + 3 * (i - 1) + 1:15 + 3 * i, end - 5:end) = [-obj.skew(lm(:, i)) * R, zeros(3)];
            end

            % //INFO Discretize,下面公式推导在附录A.2给出
            Fk = eye(size(Fc)) + Fc * dt;

            Lc = blkdiag(obj.Adjoint(obj.X), eye(6));
            hR_R = R_VectorNav_to_RightToeBottom(encoders);
            hR_L = R_VectorNav_to_LeftToeBottom(encoders);
            % //HACK hR_R * (obj.Qc + (1e4 * eye(3) .* (1 - contact(2)))) * hR_R'
            Q = blkdiag(obj.Qg, obj.Qa, zeros(3), hR_R * (obj.Qc + (1e4 * eye(3) .* (1 - contact(2)))) * hR_R', hR_L * (obj.Qc + (1e4 * eye(3) .* (1 - contact(1)))) * hR_L', zeros(3 * length(obj.landmark_ids)), obj.Qbg, obj.Qba);
            Qk = Fk * Lc * Q * Lc' * Fk' * dt; % Discretized 

            % Construct predicted state
            [obj.X, obj.theta] = obj.Construct_State(R_pred, v_pred, p_pred, dR_pred, dL_pred, lm_pred, bg_pred, ba_pred);

            % Predict Covariance
            obj.P = Fk * obj.P * Fk' + Qk;
        end
        
        % 根据论文中公式(29)更新状态
        function [] = Update_State(obj, Y, b, H, N, PI)
            % Update State and Covariance from a measurement
            % Compute Kalman Gain
            S = H * obj.P * H' + N;
            K = (obj.P * H') / S;

            % Copy X along the diagonals if more than one measurement
            % 因为可能会有多个观测量叠在一起
            X_cell = repmat({obj.X}, 1, length(Y) / size(obj.X, 1));
            Z = blkdiag(X_cell{:}) * Y - b;

            % //INFO Update State 论文公式(29)
            delta = K * PI * Z;
            dX = obj.exp(delta(1:end - 6));
            dtheta = delta(end - 5:end);
            obj.X = dX * obj.X;
            obj.theta = obj.theta + dtheta;

            % Update Covariance 论文公式(29)
            I = eye(size(obj.P));
            obj.P = (I - K * H) * obj.P * (I - K * H)' + K * N * K'; % Joseph update form

        end
        
        % 分为两脚同时触地，一只脚触地的情况，调用Update_State函数进行状态更新
        function [] = Update_ForwardKinematics(obj, encoders, contact)
            % Function to perform Right-Invariant EKF update from forward
            % kinematic measurements

            R_pred = obj.X(1:3, 1:3);
            M = length(obj.landmark_ids);
            
            % 两个脚都触地
            if contact(2) == 1 && contact(1) == 1
                % Double Support
                s_pR = p_VectorNav_to_RightToeBottom(encoders);
                s_pL = p_VectorNav_to_LeftToeBottom(encoders);
                JR = J_VectorNav_to_RightToeBottom(encoders);
                JL = J_VectorNav_to_LeftToeBottom(encoders);

                % Measurement Model 论文第十页，公式(17)和公式(20)
                Y = [s_pR; 0; 1; -1; 0; zeros(M, 1);
                     s_pL; 0; 1; 0; -1; zeros(M, 1)];
                b = zeros(size(Y));  % b设为全零是因为后面会乘上一个选择矩阵PI,b其实没用,见论文P10左下角
                H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3, 3 * M), zeros(3, 6);
                     zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3, 3 * M), zeros(3, 6)];
                N = blkdiag(R_pred * JR * obj.Qe * JR' * R_pred' + obj.Np, ...
                    R_pred * JL * obj.Qe * JL' * R_pred' + obj.Np);
                % PI的每行与矩阵Y整体相乘,第一行选取s_pR所在行,第二行选取s_pL所在行,其余都为0
                PI = [eye(3), zeros(3, 4), zeros(3, M), zeros(3, 7), zeros(3, M); 
                      zeros(3, 7), zeros(3, M), eye(3), zeros(3, 4), zeros(3, M)];

                % Update State
                obj.Update_State(Y, b, H, N, PI);

            elseif contact(2) == 1
                % Single Support Right
                s_pR = p_VectorNav_to_RightToeBottom(encoders);
                JR = J_VectorNav_to_RightToeBottom(encoders);

                % Measurement Model
                Y = [s_pR; 0; 1; -1; 0; zeros(M, 1)];
                b = zeros(size(Y));
                H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3, 3 * M), zeros(3, 6)];
                N = R_pred * JR * obj.Qe * JR' * R_pred' + obj.Np;
                PI = [eye(3), zeros(3, 4), zeros(3, M)];

                % Update State
                obj.Update_State(Y, b, H, N, PI);

            elseif contact(1) == 1
                % Single Support Left
                s_pL = p_VectorNav_to_LeftToeBottom(encoders);
                JL = J_VectorNav_to_LeftToeBottom(encoders);

                % Measurement Model
                Y = [s_pL; 0; 1; 0; -1; zeros(M, 1)];
                b = zeros(size(Y));
                H = [zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3, 3 * M), zeros(3, 6)];
                N = R_pred * JL * obj.Qe * JL' * R_pred' + obj.Np;
                PI = [eye(3), zeros(3, 4), zeros(3, M)];

                % Update State
                obj.Update_State(Y, b, H, N, PI);

            end

        end
        
        % 静态地标的观测模型
        function [] = Update_StaticLandmarks(obj, landmarks)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %simulink中固定了landmarks的大小，这里是删除landmarks中的全零列
            landmarks(:, ~all(landmarks)) = [];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Function to perform Right-Invariant EKF update from static
            % landmark distance measurements

            R_pred = obj.X(1:3, 1:3);

            % Stack landmark measurements
            Y = []; b = []; H = []; N = []; PI = [];

            for i = 1:size(landmarks, 2)
                % Search to see if measured landmark id is in the list of
                % static landmarks
                % //HACK 这里landmark_positions全为0？RIEKF_InitFcn中有值
                id = find(obj.landmark_positions(1, :) == landmarks(1, i));

                if ~isempty(id)
                    % 创建地标观测模型
                    Y = vertcat(Y, [landmarks(2:end, i); 0; 1; 0; 0]);
                    b = vertcat(b, [obj.landmark_positions(2:end, id); 0; 1; 0; 0]);
                    H = vertcat(H, [obj.skew(obj.landmark_positions(2:end, id)), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), zeros(3)]);
                    N = blkdiag(N, R_pred * obj.Ql * R_pred');
                    PI = blkdiag(PI, [eye(3), zeros(3, 4)]);
                end

            end

            % Update State
            if ~isempty(Y)
                obj.Update_State(Y, b, H, N, PI);
            end

        end

        function [] = Update_Landmarks(obj, landmarks)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %simulink中固定了landmarks的大小，这里是删除landmarks中的全零列
            landmarks(:, ~all(landmarks)) = [];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Function to perform Right-Invariant EKF update from estimated
            % landmark distance measurements

            R_pred = obj.X(1:3, 1:3);

            % Stack landmark measurements
            Y = []; H = []; N = []; PI = []; new_landmarks = [];

            for i = 1:size(landmarks, 2)
                % Search to see if measured landmark id is in the list of
                % static landmarks
                id = find(obj.landmark_ids == landmarks(1, i));

                if isempty(id)
                    new_landmarks = horzcat(new_landmarks, landmarks(:, i));
                else
                    % Create measurement model
                    Y1 = [landmarks(2:end, i); 0; 1; 0; 0];
                    Y2 = zeros(length(obj.landmark_ids), 1);
                    Y2(id) = -1;
                    Y = vertcat(Y, [Y1; Y2]);

                    H1 = [zeros(3), zeros(3), -eye(3), zeros(3), zeros(3)];
                    H2 = zeros(3, 3 * length(obj.landmark_ids));
                    H2(:, 3 * (id - 1) + 1:3 * (id - 1) + 3) = eye(3);
                    H3 = zeros(3, 6);
                    H = vertcat(H, [H1, H2, H3]);

                    N = blkdiag(N, R_pred * obj.Ql * R_pred');
                    PI = blkdiag(PI, [eye(3), zeros(3, 4), zeros(3, length(obj.landmark_ids))]);
                end

            end

            % Update State
            if ~isempty(Y)
                b = zeros(size(Y));
                obj.Update_State(Y, b, H, N, PI);
            end

            % Augment State for new landmarks
            % 为新地标更新状态向量和协方差矩阵
            if ~isempty(new_landmarks)
                [R, ~, p, ~, ~, ~, ~, ~] = obj.Separate_State(obj.X, obj.theta);

                for i = 1:size(new_landmarks, 2)
                    % Initialize new landmark mean
                    obj.landmark_ids = horzcat(obj.landmark_ids, new_landmarks(1, i));
                    obj.X = blkdiag(obj.X, 1);
                    % {W}_p_{WL} = {W}_p_{WB} + R_{WB}*{B}_p_{BL}
                    obj.X(1:3, end) = p + R * new_landmarks(2:end, i);

                    % Initialize new landmark covariance
                    F = eye(size(obj.P, 1) - 6); % Start with I with state dim
                    F = vertcat(F, [zeros(3, 6), eye(3), zeros(3, 3 * (size(obj.X, 2) - 6))]); % Add row to increase dimension
                    F = blkdiag(F, eye(6)); % Add block I for parameters
                    G = zeros(size(F, 1), 3);
                    G(end - 8:end - 6, :) = R_pred;
                    obj.P = F * obj.P * F' + G * obj.Ql * G';
                end

            end

        end

    end % methods

end % classdef
