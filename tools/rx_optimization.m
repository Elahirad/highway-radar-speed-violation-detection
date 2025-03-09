%% INIT
close all; clc; clearvars;

fc = 2e9;
c = 3e8;
lambda = c / fc;
theta = -90:1:90;
theta_rad = deg2rad(theta);
d_min = 0;
d_max = 2;
N = 500;
d = (d_max-d_min) / N;


theta_diff = abs(repmat(theta, length(theta), 1) - ...
    repmat(theta', 1, length(theta)));
theta_diff = reshape(theta_diff, length(theta)^2, 1);
steering_matrix = zeros(length(theta)^2, N);
for n = 1:N
    steering_temp = exp(1j * 2 * pi * d * (n - 1) * ...
        sin(theta_rad) / lambda)' * exp(1j * 2 * pi * d * (n - 1) * ...
        sin(theta_rad) / lambda);
    steering_matrix(:, n) = reshape(steering_temp, length(theta)^2, 1);
end

rows_to_process = find(theta_diff > 5.73);

%% CVX Optimization
cvx_begin
    variable v(N) nonnegative
    variable t

    % Objective: minimize t
    minimize(t)

    % Constraints
    subject to

        abs(v' * steering_matrix(rows_to_process, :)') <= t;

        % Total position normalization
        sum(v) == 1; % Normalize total length
        v >= 0; % Non-negative positions
cvx_end
% %% Load V.MAT
% load v;
%% Choose antennas
antenna_count = 11;
antenna_d = zeros(1, antenna_count);
for x=1:antenna_count
    antenna_d(x) = d_min + d * (find(v==max(v)) - 1);
    v(v==max(v)) = 0;
end

%% Compute array pattern
steering_matrix = zeros(length(theta), antenna_count);
for n = 1:antenna_count
    steering_matrix(:, n) = exp(-1j * 2 * pi * antenna_d(n) * ...
        sin(theta_rad) / lambda);
end
pattern = abs(sum(steering_matrix, 2)).^2;
pattern = 10 * log10(pattern / max(pattern));

%% Plot the array pattern
figure;
plot(theta, pattern, 'LineWidth', 1.5);
xlabel('Angle (degrees)');
ylabel('Array Pattern (dB)');
title('Array Pattern with Optimized Antenna Spacing');
grid on;

%%
plot(antenna_d, zeros(size(antenna_d)), 'ob', 'DisplayName', 'Receivers');
hold on;
plot(ones(2, 1), [-0.04, 0.04], 'or', 'DisplayName', 'Transmitters');

legend;
ylim([-0.5, 0.5]);
grid on;