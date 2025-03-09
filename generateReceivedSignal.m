function received_signal = generateReceivedSignal(lambda, fs, pulse, ...
    h, G, System_Loss, noise_power, tx_positions, tx_phases, ...
    rx_positions, target_distances, target_velocities_km_h, ...
    target_azimuths, RCS)


    c = 3e8;
    % Calculate range and elevation
    target_dopplers = target_velocities_km_h / 3.6 * 2 / lambda;
    target_ranges = sqrt(h^2 + (target_distances./cosd(target_azimuths)).^2);
    target_elevations = -asind(h ./ target_ranges);
    
    t = 0:1/fs:length(pulse)/fs - 1/fs;
    % Initialize the received signal matrix
    received_signal = zeros(length(rx_positions), length(t));
    % Simulate received signal for each transmitter-receiver-target combination
    for rx_idx = 1:length(rx_positions)
        rx_pos = rx_positions(rx_idx); % Receiver position
        for tx_idx = 1:length(tx_positions)
            tx_pos = tx_positions(tx_idx); % Transmitter position
            for target_idx = 1:length(target_ranges)
                % Target parameters
                target_range = target_ranges(target_idx);
                target_doppler = target_dopplers(target_idx);
                azimuth_angle = target_azimuths(target_idx);
                elevation_angle = target_elevations(target_idx);
                % Signal reaching target
                target_reached_gain = G * RCS / (4*pi*target_range^2);
                target_reached_pulse = pulse ...
                    * cosd(azimuth_angle) * cosd(elevation_angle) ... % Antenna pattern
                    * sqrt(target_reached_gain)  ... % Transmitter to target Loss
                    * exp(1j * 2*pi / lambda * tx_pos * ... % Elevation angle effect
                    sind(elevation_angle)) * exp(1j * tx_phases(tx_idx));
                % Target reflected signal
                reflected_pulse = circshift(target_reached_pulse, ... % Range effect
                    round((2 * target_range / c) * fs));
                reflected_pulse = reflected_pulse ...
                    *exp(1j * 2*pi / lambda * rx_pos * sind(azimuth_angle)) ... % Azimuth effect
                    .*exp(1j * 2*pi * target_doppler * t); % Doppler effect
                % Received signal
                received_pulse_gain = 1/(System_Loss * 4*pi*target_range^2) ...
                    * ((G * lambda^2)/(4*pi));
                received_pulse = sqrt(received_pulse_gain) ... % Target to receiver Loss
                    * cosd(azimuth_angle) * cosd(elevation_angle) ... % Antenna pattern
                    * reflected_pulse;
                % Add to the received signal for the current receiver
                received_signal(rx_idx, :) = received_signal(rx_idx, :) ...
                    + received_pulse;
            end
        end
    end
    % Add thermal noise to the received signal
    noise = sqrt(noise_power) * randn(size(received_signal)); % Gaussian noise
    received_signal = received_signal + noise;
end