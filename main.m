%% INIT
clc;
clearvars;
% System Parameters
c = 3e8;
fc = 2e9;
lambda = c / fc;
tau = 50e-9;
PRI = 400e-6;
PRF = 1/PRI;
T_recording = 0.4;
N_pulses = floor(T_recording / PRI);
% Antenna gain and noise properties
G = 10;
System_Loss = 10;
k = 1.38e-23;
T = 290;
B = 1 / tau;
noise_figure = 5;
noise_power = k * T * B * G * 10^(noise_figure/10);
SNR_min = 30;
% Transmitter and Receiver parameters
h = 5;
tx_positions = [-0.04, 0.04];
tx_phases = [0, 1.5];
M = 10;
rx_positions = [0.0720, 0.3680, 0.5160, 0.6640, 0.8120, ...
    1.1840, 1.3320, 1.48, 1.628, 1.7760, 1.924] - 1;

%% SIMULATE TARGETS
target_distances = [5, 15, 30, 11, 36]; % Distances
target_velocities = [-210, 70, -140, -60, 195]; % Dopplers
target_azimuths = [30, -60, 15, 20, -12]; % Azimuth Angles
RCS = 10; % Radar Cross Section

% Signal parameters
fs = 40e6;
t = 0:1/fs:T_recording-1/fs;
% Generate baseband pulse waveform
A = 1.44;
pulse_PRI = A*[ones(1, round(tau * fs)),zeros(1, round((PRI - tau) * fs))];
pulse = repmat(pulse_PRI, 1, N_pulses);

% Generate the reflected signal
received_signal = generateReceivedSignal(lambda, fs, pulse, ...
    h, G, System_Loss, noise_power, tx_positions, tx_phases, ...
    rx_positions, target_distances, target_velocities, target_azimuths, RCS);

%% SPEED VIOLATOR DETECTION AND PROCESSING
% Initialize parameters
speed_limit_km = 110;
speed_max_km = 300;
speed_sweep_step = 5;
[est_target_snrs, est_target_ranges, est_target_dopplers, est_target_azimuths] ...
    = processReceivedSignal(fs, pulse, pulse_PRI, lambda, ...
    tau, rx_positions, noise_power, SNR_min, received_signal, ...
    speed_limit_km, speed_max_km, speed_sweep_step, 0);

for i=1:length(est_target_snrs)
    fprintf("----------------------------------------------\n");
    fprintf("Speed violating vehicle detected !\nSpeed: %d Km/h\n",...
        est_target_dopplers(i) * 3.6);
    fprintf("SNR: %.2f\n", est_target_snrs(i));
    % Distance calculation
    distance = sqrt(est_target_ranges(i)^2 - h^2) * cosd(est_target_azimuths(i));
    % Elevation calculation
    elevation = -asind(h / est_target_ranges(i));
    fprintf("Distance: %.3f Meters\n", distance);
    fprintf("Azimuth Angle: %.1f Degrees\n", est_target_azimuths(i));
    fprintf("Elevation Angle: %.1f Degrees\n", elevation);
end