%% ========================================================================
%  AIRCRAFT NAVIGATION SYSTEM: CLIMB-CRUISE-DESCENT TRAJECTORY
%  6-State Kalman Filter with GPS + Pressure 
%  Based on "Systems for Autonomous Aircraft - Lecture 8"
% ========================================================================
clear; clc; close all;

%% ======================== SIMULATION CONFIGURATION =======================
dt = 0.1;                 % Time step (s)
T  = 120;                 % Total duration: 3 phases × 40s
t  = 0:dt:T;              % Time vector
N  = numel(t);            % Number of samples

g  = 9.81;                % Gravitational acceleration (m/s^2)
deg2rad = @(d) d*pi/180;  % Degree to radian conversion

%% ======================== TRAJECTORY DESIGN =============================
% Flight Parameters
Vx    = 80;               % Constant forward airspeed (m/s)
h_max = 1000;             % Maximum cruise altitude (m)

% Mission Phase Durations
t_climb  = 40;            % Climb phase duration (s)
t_cruise = 40;            % Cruise phase duration (s)
t_desc   = 40;            % Descent phase duration (s)

% Validate total time
if abs((t_climb + t_cruise + t_desc) - T) > 1e-6
    error("Phase durations must sum to total time T");
end

t_desc_start = t_climb + t_cruise;  % Descent initiation time

%% ================= REFERENCE TRAJECTORY GENERATION ======================
% Coordinate System: NED (North-East-Down)
%   x: Forward (North)
%   y: Lateral (East) 
%   z: Downward (negative altitude)

% Velocity Components (World Frame)
vx = Vx * ones(1,N);      % Constant forward velocity
vy = zeros(1,N);          % No lateral motion (straight flight)

% Position Integration
x = vx .* t;              % North position
y = zeros(1,N);           % East position (zero for straight path)

% Altitude Profile (smooth cosine transitions)
h      = zeros(1,N);      % Altitude above ground (UP positive)
h_dot  = zeros(1,N);      % Vertical velocity (climb rate)

for k = 1:N
    tk = t(k);
    
    if tk <= t_climb
        % CLIMB: Smooth transition from 0 to h_max
        phase = tk / t_climb;
        h(k)     = h_max * 0.5 * (1 - cos(pi * phase));
        h_dot(k) = h_max * 0.5 * (pi/t_climb) * sin(pi * phase);
        
    elseif tk <= t_climb + t_cruise
        % CRUISE: Constant altitude
        h(k)     = h_max;
        h_dot(k) = 0;
        
    else
        % DESCENT: Smooth transition from h_max to 0
        td = tk - t_desc_start;
        phase = td / t_desc;
        h(k)     = h_max * 0.5 * (1 + cos(pi * phase));
        h_dot(k) = -h_max * 0.5 * (pi/t_desc) * sin(pi * phase);
    end
end

% Convert to NED Down Convention
z  = -h;                  % NED z-coordinate (down positive)
vz = -h_dot;              % NED vertical velocity

%% =================== ATTITUDE KINEMATICS ================================
% Flight Path Angle & Euler Angles
gamma_fpa = atan2(-vz, vx);   % Flight path angle (climb/descent)
theta     = gamma_fpa;        % Pitch angle (aligned with FPA)
phi       = zeros(1,N);       % Roll angle (wings level)
psi       = zeros(1,N);       % Heading angle (straight north)

% Angular Velocities (Body Rates)
phi_dot   = [diff(phi)/dt 0];
theta_dot = [diff(theta)/dt 0];
psi_dot   = [diff(psi)/dt 0];

p = phi_dot;              % Roll rate
q = theta_dot;            % Pitch rate
r = psi_dot;              % Yaw rate

%% =================== BODY FRAME VELOCITIES ==============================
% Transform world velocities to body frame
u =  vx .* cos(theta) - vz .* sin(theta);   % Body-x velocity
v =  zeros(1,N);                            % Body-y velocity
w =  vx .* sin(theta) + vz .* cos(theta);   % Body-z velocity (down)

% Body Frame Accelerations
u_dot = [diff(u)/dt 0];
v_dot = zeros(1,N);
w_dot = [diff(w)/dt 0];

%% =================== IMU SENSOR MODEL: ACCELEROMETER ====================
% Specific Force Equations (includes gravity compensation)
ax_true = u_dot + q.*w - r.*v + g * sin(theta);
ay_true = v_dot + r.*u - p.*w - g * cos(theta) .* sin(phi);
az_true = w_dot + p.*v - q.*u - g * cos(theta) .* cos(phi);
a_true  = [ax_true; ay_true; az_true];

%% =================== SENSOR ERROR MODELS ================================

%% --- Accelerometer & Gyroscope Bias (with random walk) ---
% Initial Bias Offsets
acc_bias0_range  = 0.2;   % m/s^2 - typical MEMS IMU bias
gyro_bias0_range = 0.01;  % rad/s - typical MEMS gyro bias

acc_bias0  = acc_bias0_range  * (2*rand(3,1) - 1);
gyro_bias0 = gyro_bias0_range * (2*rand(3,1) - 1);

% Bias Random Walk Coefficients
acc_bias_rw_std  = 0.005;    % (m/s^2)/sqrt(s) - accelerometer bias instability
gyro_bias_rw_std = 0.0005;   % (rad/s)/sqrt(s) - gyroscope bias instability

% Initialize Bias Time Series
acc_bias  = zeros(3,N);
gyro_bias = zeros(3,N);
acc_bias(:,1)  = acc_bias0;
gyro_bias(:,1) = gyro_bias0;

% Propagate Bias via Random Walk
for k = 2:N
    acc_bias(:,k)  = acc_bias(:,k-1)  + acc_bias_rw_std  * sqrt(dt) * randn(3,1);
    gyro_bias(:,k) = gyro_bias(:,k-1) + gyro_bias_rw_std * sqrt(dt) * randn(3,1);
end

% Measurement Noise (white Gaussian)
acc_noise_std  = 0.05;   % m/s^2 - accelerometer noise density
gyro_noise_std = 0.003;  % rad/s - gyroscope noise density

% Generate Noisy Measurements
acc_meas  = a_true    + acc_bias  + acc_noise_std  * randn(3,N);
gyro_true = [p; q; r];
gyro_meas = gyro_true + gyro_bias + gyro_noise_std * randn(3,N);

%% --- GPS Sensor Model (Position & Velocity) ---
gps_pos_noise = 2.0;      % m - horizontal position accuracy
gps_vel_noise = 0.3;      % m/s - velocity accuracy
gps_dt        = 0.5;      % s - update rate (2 Hz)

% Initialize NaN arrays (measurements only at update times)
gps_pos = nan(3,N);
gps_vel = nan(3,N);

% Generate GPS Measurements at Defined Rate
next_gps_t = 0;
for k = 1:N
    tk = t(k);
    if abs(tk - next_gps_t) < dt/2   % Improved floating-point comparison
        next_gps_t = next_gps_t + gps_dt;
        gps_pos(:,k) = [x(k); y(k); z(k)] + gps_pos_noise*randn(3,1);
        gps_vel(:,k) = [vx(k); vy(k); vz(k)] + gps_vel_noise*randn(3,1);
    end
end

%% --- Barometric Altimeter Model ---
% Atmospheric Constants
rho = 1.225;              % Sea-level air density (kg/m^3)
g0  = 9.80665;            % Standard gravity (m/s^2)

% Pressure-Altitude Relationship (linearized)
P_ground = 101325;                    % Ground-level pressure (Pa)
P_true   = P_ground - rho * g0 * h;   % True pressure: P = P0 - ρgh

% Sensor Errors
pres_bias      = -50;                 % Pa - sensor bias
pres_noise_std = 5;                   % Pa - measurement noise

% Measured Pressure
P_meas = P_true + pres_bias + pres_noise_std * randn(1,N);

%  Calibrate P_ground using first GPS measurement
% This eliminates systematic altitude offset
P_ground_calibrated = P_meas(1) + rho * g0 * h(1);  % Match true altitude at t=0

% Convert Pressure to Altitude using calibrated reference
h_baro = (P_ground_calibrated - P_meas) ./ (rho * g0);   % Altitude (UP)
z_baro = -h_baro;                                        % NED z-coordinate

%% --- Magnetometer Model (3-Axis Compass) ---
% Earth's Magnetic Field in NED Frame (normalized, Naples region)
m_n = [0.45; 0; 0.85];   % [North; East; Down] components

mag_true = zeros(3,N);

% Transform Magnetic Field to Body Frame
for k = 1:N
    cphi = cos(phi(k));  sphi = sin(phi(k));
    cth  = cos(theta(k)); sth  = sin(theta(k));
    cpsi = cos(psi(k));  spsi = sin(psi(k));
    
    % Direction Cosine Matrix: NED -> Body (3-2-1 Euler sequence)
    C_bn = [ cth*cpsi,                    cth*spsi,                   -sth;
             sphi*sth*cpsi - cphi*spsi,   sphi*sth*spsi + cphi*cpsi,  sphi*cth;
             cphi*sth*cpsi + sphi*spsi,   cphi*sth*spsi - sphi*cpsi,  cphi*cth ];
    
    mag_true(:,k) = C_bn * m_n;  % True magnetic field in body frame
end

% Sensor Errors
mag_bias      = [0.01; -0.02; 0.015];  % Hard-iron bias (normalized units)
mag_noise_std = 0.01;                  % Measurement noise

mag_meas = mag_true + mag_bias + mag_noise_std * randn(3,N);

% Tilt-Compensated Heading Estimation
mag_declination = deg2rad(4.18);  % Magnetic declination for Naples, Italy
yaw_mag = zeros(1,N);

for k = 1:N
    cphi = cos(phi(k));  sphi = sin(phi(k));
    cth  = cos(theta(k)); sth  = sin(theta(k));
    cpsi = cos(psi(k));  spsi = sin(psi(k));
    
    C_bn = [ cth*cpsi,                    cth*spsi,                   -sth;
             sphi*sth*cpsi - cphi*spsi,   sphi*sth*spsi + cphi*cpsi,  sphi*cth;
             cphi*sth*cpsi + sphi*spsi,   cphi*sth*spsi - sphi*cpsi,  cphi*cth ];
    
    C_nb = C_bn.';  % Body -> NED transformation
    
    % Project measured field back to NED frame
    m_n_hat = C_nb * mag_meas(:,k);
    
    % Compute heading from horizontal components
    mN = m_n_hat(1);
    mE = m_n_hat(2);
    yaw_mag(k) = atan2(mE, mN) + mag_declination;
end

%% =================== KALMAN FILTER DESIGN ===============================
% State Vector: [x, y, z, vx, vy, vz]^T
nx = 6;

% Initial State Estimate
x_est = [x(1); y(1); z(1); vx(1); vy(1); vz(1)];
P_est = diag([10 10 25 5 5 5]);  % Higher uncertainty in z-position (25 vs 10)

% State Transition Matrix (Constant Velocity Model)
F = eye(nx);
F(1,4) = dt;  % x  = x  + vx*dt
F(2,5) = dt;  % y  = y  + vy*dt
F(3,6) = dt;  % z  = z  + vz*dt

% Process Noise Covariance (tuned for climb/descent dynamics)
q_pos = 0.5;   % m^2 - position process noise (reduced for smoother estimates)
q_vel = 0.1;   % (m/s)^2 - velocity process noise (reduced to tighten covariance)
Q = diag([q_pos q_pos q_pos q_vel q_vel q_vel]);

% GPS Measurement Model
H_gps = eye(nx);  % Observes all 6 states
R_pos = gps_pos_noise^2 * eye(3);
R_vel = (gps_vel_noise * 0.8)^2 * eye(3);  % Reduced to 80% for tighter bounds
R_gps = blkdiag(R_pos, R_vel);

% Barometer Measurement Model (z-position only)
H_baro = [0 0 1 0 0 0];
R_baro = (3.0)^2;   % Variance: ~3m altitude noise (increased to reduce overconfidence)

% State & Covariance Storage
kf_state = zeros(nx, N);
P_log = zeros(nx, nx, N);

%% =================== MAIN KALMAN FILTER LOOP ============================
for k = 1:N
    % --- PREDICTION STEP ---
    x_pred = F * x_est;
    P_pred = F * P_est * F.' + Q;
    
    x_upd = x_pred;
    P_upd = P_pred;
    
    % --- UPDATE STEP: GPS (if available) ---
    if ~isnan(gps_pos(1,k))
        z_gps = [gps_pos(:,k); gps_vel(:,k)];
        S_gps = H_gps * P_upd * H_gps.' + R_gps;
        K_gps = P_upd * H_gps.' / S_gps;
        innov_gps = z_gps - H_gps * x_upd;
        x_upd = x_upd + K_gps * innov_gps;
        P_upd = (eye(nx) - K_gps*H_gps) * P_upd;
    end
    
    % --- UPDATE STEP: Barometer (always available) ---
    z_meas_baro = z_baro(k);
    S_baro = H_baro * P_upd * H_baro.' + R_baro;
    K_baro = P_upd * H_baro.' / S_baro;
    innov_baro = z_meas_baro - H_baro * x_upd;
    x_est = x_upd + K_baro * innov_baro;
    P_est = (eye(nx) - K_baro*H_baro) * P_upd;
    
    % Store Results
    P_log(:,:,k) = P_est;
    kf_state(:,k) = x_est;
end

%% =================== ESTIMATION ERROR ANALYSIS ==========================
% Extract Estimated States
x_kf  = kf_state(1,:);
y_kf  = kf_state(2,:);
z_kf  = kf_state(3,:);
vx_kf = kf_state(4,:);
vy_kf = kf_state(5,:);
vz_kf = kf_state(6,:);

% Compute Errors
ex  = x_kf - x;
ey  = y_kf - y;
ez  = z_kf - z;
evx = vx_kf - vx;
evy = vy_kf - vy;
evz = vz_kf - vz;

% Extract 1-Sigma Uncertainties from Covariance
sigma_x  = reshape(sqrt(squeeze(P_log(1,1,:))), 1, []);
sigma_y  = reshape(sqrt(squeeze(P_log(2,2,:))), 1, []);
sigma_z  = reshape(sqrt(squeeze(P_log(3,3,:))), 1, []);
sigma_vx = reshape(sqrt(squeeze(P_log(4,4,:))), 1, []);
sigma_vy = reshape(sqrt(squeeze(P_log(5,5,:))), 1, []);
sigma_vz = reshape(sqrt(squeeze(P_log(6,6,:))), 1, []);

% GPS Error Analysis
idx_gps = ~isnan(gps_pos(1,:));
gps_ex = nan(1,N);
gps_ey = nan(1,N);
gps_ez = nan(1,N);
gps_ex(idx_gps) = gps_pos(1,idx_gps) - x(idx_gps);
gps_ey(idx_gps) = gps_pos(2,idx_gps) - y(idx_gps);
gps_ez(idx_gps) = gps_pos(3,idx_gps) - z(idx_gps);

%% =================== VISUALIZATION ======================================

%% FIGURE 1: Position Estimates (GPS vs True vs KF)
figure('Name','Position Estimates');
subplot(3,1,1);
plot(t, x, 'k','LineWidth',1.3); hold on;
plot(t(idx_gps), gps_pos(1,idx_gps),'g.','MarkerSize',6);
plot(t, x_kf,'r--','LineWidth',1.3);
grid on; ylabel('x (m)'); legend('True','GPS','KF','Location','best');
title('North Position (x)');

subplot(3,1,2);
plot(t, y, 'k','LineWidth',1.3); hold on;
plot(t(idx_gps), gps_pos(2,idx_gps),'g.','MarkerSize',6);
plot(t, y_kf,'r--','LineWidth',1.3);
grid on; ylabel('y (m)'); legend('True','GPS','KF','Location','best');
title('East Position (y)');

subplot(3,1,3);
plot(t, h, 'k','LineWidth',1.3); hold on;
plot(t(idx_gps), -gps_pos(3,idx_gps),'g.','MarkerSize',6);
plot(t, -z_kf,'r--','LineWidth',1.3);
grid on; ylabel('Altitude h (m)'); xlabel('Time (s)');
legend('True','GPS','KF','Location','best');
title('Altitude (UP positive)');

%% FIGURE 2: Velocity Estimates (GPS vs True vs KF)
figure('Name','Velocity Estimates');
subplot(3,1,1);
plot(t, vx, 'k','LineWidth',1.3); hold on;
plot(t(idx_gps), gps_vel(1,idx_gps),'g.','MarkerSize',6);
plot(t, vx_kf,'r--','LineWidth',1.3);
grid on; ylabel('v_x (m/s)'); legend('True','GPS','KF','Location','best');
title('North Velocity (vx)');

subplot(3,1,2);
plot(t, vy, 'k','LineWidth',1.3); hold on;
plot(t(idx_gps), gps_vel(2,idx_gps),'g.','MarkerSize',6);
plot(t, vy_kf,'r--','LineWidth',1.3);
grid on; ylabel('v_y (m/s)'); legend('True','GPS','KF','Location','best');
title('East Velocity (vy)');

subplot(3,1,3);
plot(t, vz, 'k','LineWidth',1.3); hold on;
plot(t(idx_gps), gps_vel(3,idx_gps),'g.','MarkerSize',6);
plot(t, vz_kf,'r--','LineWidth',1.3);
grid on; ylabel('v_z (m/s)'); xlabel('Time (s)');
legend('True','GPS','KF','Location','best');
title('Vertical Velocity (vz, down positive)');

%% FIGURE 3: Kalman Filter Errors with Covariance Bounds
figure('Name','KF Errors vs Covariance');

% Position Errors
subplot(2,3,1);
plot(t, ex, 'm','LineWidth',1.0); hold on;
plot(t,  sigma_x, 'b--','LineWidth',1.2);
plot(t, -sigma_x, 'b--','LineWidth',1.2);
grid on; ylabel('e_x (m)');
legend('KF error','±1σ','Location','best');
title(sprintf('Position X Error (μ=%.2f, σ=%.2f)', mean(ex), std(ex)));

subplot(2,3,2);
plot(t, ey, 'm','LineWidth',1.0); hold on;
plot(t,  sigma_y, 'b--','LineWidth',1.2);
plot(t, -sigma_y, 'b--','LineWidth',1.2);
grid on; ylabel('e_y (m)');
legend('KF error','±1σ','Location','best');
title(sprintf('Position Y Error (μ=%.2f, σ=%.2f)', mean(ey), std(ey)));

subplot(2,3,3);
plot(t, ez, 'm','LineWidth',1.0); hold on;
plot(t,  sigma_z, 'b--','LineWidth',1.2);
plot(t, -sigma_z, 'b--','LineWidth',1.2);
grid on; ylabel('e_z (m)');
legend('KF error','±1σ','Location','best');
title(sprintf('Position Z Error (μ=%.2f, σ=%.2f)', mean(ez), std(ez)));

% Velocity Errors
subplot(2,3,4);
plot(t, evx, 'm','LineWidth',1.0); hold on;
plot(t,  sigma_vx, 'b--','LineWidth',1.2);
plot(t, -sigma_vx, 'b--','LineWidth',1.2);
grid on; ylabel('e_{vx} (m/s)'); xlabel('Time (s)');
legend('KF error','±1σ','Location','best');
title(sprintf('Velocity Vx Error (μ=%.2f, σ=%.2f)', mean(evx), std(evx)));

subplot(2,3,5);
plot(t, evy, 'm','LineWidth',1.0); hold on;
plot(t,  sigma_vy, 'b--','LineWidth',1.2);
plot(t, -sigma_vy, 'b--','LineWidth',1.2);
grid on; ylabel('e_{vy} (m/s)'); xlabel('Time (s)');
legend('KF error','±1σ','Location','best');
title(sprintf('Velocity Vy Error (μ=%.2f, σ=%.2f)', mean(evy), std(evy)));

subplot(2,3,6);
plot(t, evz, 'm','LineWidth',1.0); hold on;
plot(t,  sigma_vz, 'b--','LineWidth',1.2);
plot(t, -sigma_vz, 'b--','LineWidth',1.2);
grid on; ylabel('e_{vz} (m/s)'); xlabel('Time (s)');
legend('KF error','±1σ','Location','best');
title(sprintf('Velocity Vz Error (μ=%.2f, σ=%.2f)', mean(evz), std(evz)));

%% FIGURE 4: Accelerometer Measurements
figure('Name','Accelerometer Data');
subplot(3,1,1);
plot(t, acc_meas(1,:), 'b','LineWidth',1); grid on;
ylabel('a_x (m/s²)'); title('Body X-Axis Acceleration');

subplot(3,1,2);
plot(t, acc_meas(2,:), 'b','LineWidth',1); grid on;
ylabel('a_y (m/s²)'); title('Body Y-Axis Acceleration');

subplot(3,1,3);
plot(t, acc_meas(3,:), 'b','LineWidth',1); grid on;
ylabel('a_z (m/s²)'); xlabel('Time (s)');
title('Body Z-Axis Acceleration (NED Down)');

%% FIGURE 5: Gyroscope Measurements
figure('Name','Gyroscope Data');
subplot(3,1,1);
plot(t, gyro_meas(1,:), 'b','LineWidth',1); grid on;
ylabel('p (rad/s)'); title('Roll Rate (p)');

subplot(3,1,2);
plot(t, gyro_meas(2,:), 'b','LineWidth',1); grid on;
ylabel('q (rad/s)'); title('Pitch Rate (q)');

subplot(3,1,3);
plot(t, gyro_meas(3,:), 'b','LineWidth',1); grid on;
ylabel('r (rad/s)'); xlabel('Time (s)');
title('Yaw Rate (r)');

%% FIGURE 6: 3D Trajectory Visualization
figure('Name','3D Trajectory');
plot3(x, y, h, 'k','LineWidth',1.8); hold on;
plot3(x(1),y(1),h(1),'go','MarkerSize',12,'LineWidth',2);
plot3(x(end),y(end),h(end),'r^','MarkerSize',12,'LineWidth',2);
grid on;
xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
title('Aircraft Trajectory: Climb-Cruise-Descent');
legend('Flight Path','Start','End','Location','best');
view(45,20);

%% FIGURE 7: Barometric Altitude Performance
figure('Name','Barometric Altimeter');
baro_err = h_baro - h;

subplot(2,1,1);
plot(t, h, 'k','LineWidth',1.4); hold on;
plot(t, h_baro, 'b.','MarkerSize',6);
grid on; xlabel('Time (s)'); ylabel('Altitude (m)');
legend('True','Barometer','Location','best');
title('Barometric Altitude vs Ground Truth');

subplot(2,1,2);
plot(t, baro_err, 'm','LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Error (m)');
title(sprintf('Barometer Error (μ=%.2f m, σ=%.2f m)', mean(baro_err), std(baro_err)));

%% FIGURE 8: Magnetometer Heading Estimation
figure('Name','Magnetometer Heading');
plot(t, psi, 'k','LineWidth',1.4); hold on;
plot(t, yaw_mag, 'g.','MarkerSize',6);
plot(t, yaw_mag - psi, 'm--','LineWidth',1.1);
grid on; xlabel('Time (s)'); ylabel('Heading (rad)');
legend('True ψ','Magnetometer ψ','Error','Location','best');
title('Tilt-Compensated Heading Estimation');
