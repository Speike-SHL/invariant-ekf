### Tunable Parameters
以下参数将影响进入滤波器的实际噪声测量值::
* `gyro_true_bias_init` - Initial gyroscope bias

  初始陀螺仪bias

* `accel_true_bias_init` - Initial accelerometer bias

  初始加速度计bias

* `gyro_true_noise_std` - Standard deviation of noise added to the gyroscope measurement

  增加到陀螺仪观测上的噪声的标准偏差

* `gyro_true_bias_noise_std` - Standard deviation of noise added to the gyroscope bias

  增加到陀螺仪bias上的噪声的标准偏差

* `accel_true_noise_std` - Standard deviation of noise added to the accelerometer measurement

  增加到加速度计观测上的噪声的标准偏差

* `accel_true_bias_noise_std` - Standard deviation of noise added to the accelerometer bias

  增加到加速度计bias上的噪声的标准偏差

* `landmark_true_noise_std` - Standard deviation of noise added to the land mark measurements

  增加到地标观测上的噪声的标准偏差

* `landmark_positions` - List of landmark positions with an associated ID

  带有ID的地标位置列表

* `landmark_measurement_frequency` - Frequency of incoming landmark measurements

  进来的地标观测频率

以下参数将影响过滤器的运行方式::
* `static_bias_initialization` - Flag that enables static bias initialization, where the initial bias estimate is obtained from the first few seconds of data assuming the base pose remains fixed. Keep this flag to false for the included dataset.

  标志，使能static bias初始化，其中初始bias从假定base pose保持固定的前几秒数据中获得。对于包含的数据集，将此标志设置为false。

* `ekf_update_enabled` - Flag that enables the update phase of the Kalman filter.

  标志，使能卡尔曼滤波的更新阶段

* `enable_kinematic_measurements` - Flag that enables kinematic measurements.

  标志，使能运动学观测

* `enable_landmark_measurements` - Flag that enables landmark measurements.

  标志，使能地标观测

* `enable_static_landmarks` - Flag that enables static landmarks. If false, the landmark positions will be estimated along with the rest of the state variables.

  标志，使能静态地标。如果为假，则地标位置将与其他状态量一起估计。

以下参数将影响用于过程和测量模型的初始条件和协方差：
* `gyro_bias_init` - Initial gyroscope bias estimate

  初始陀螺仪bias估计

* `accel_bias_init` - Initial accelerometer bias estimate

  初始加速度计bias估计

* `gyro_noise_std` - Standard deviation of the gyroscope measurement noise

  陀螺仪观测噪声的标准偏差

* `gyro_bias_noise_std` - Standard deviation of the gyroscope bias noise

  陀螺仪bias噪声的标准偏差

* `accel_noise_std` - Standard deviation of the accelerometer measurement noise

  加速度计观测噪声的标准偏差

* `accel_bias_noise_std` - Standard deviation the accelerometer bias noise

  加速度计bias噪声的标准偏差

* `contact_noise_std` - Standard deviation of the contact frame velocity measurement noise

  contact frame 速度观测噪声的标准偏差

* `encoder_noise_std` - Standard deviation of the joint encoder measurement noise

  关节编码器观测噪声的标准偏差

* `landmark_noise_std` - Standard deviation of the landmark measurement noise

  地标观测噪声的标准偏差

以下参数设置状态估计的初始协方差:
* `prior_base_pose_std` - Initial base orientation and position standard deviation

  初始基座位姿（方向和位置）的标准偏差

* `prior_base_velocity_std` - Initial base velocity standard deviation

  初始基座速度的标准偏差

* `prior_contact_position_std` - Initial contact position standard deviation

  初始接触位置的标准偏差

* `prior_gyro_bias_std` - Initial gyroscope bias standard deviation

  初始陀螺仪bias的标准偏差

* `prior_accel_bias_std` - Initial accelerometer bias standard deviation

  初始加速度计bias的标准偏差

* `prior_forward_kinematics_std` - Additional noise term that is added to increase the forward kinematics measurement covariance

  增加到正向运动学观测协方差的附加噪声项
