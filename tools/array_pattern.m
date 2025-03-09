%% Initialize params
fc = 2e9;
lambda = 3e8/fc;
d_antennas = [-0.04, 0.04];
phi_antennas = [0, 1.5];
% d_antennas = [0];
% phi_antennas = [0];
theta = -pi/2:0.01:pi/2;

antenna_pattern = cos(theta);

%% Calculate g_Theta
g_theta = g(theta, antenna_pattern, d_antennas, phi_antennas, fc);

%% Plot
figure;
subplot(1, 2, 1);
polarplot(theta, g_theta);
title("Polar Array Pattern");
grid on;
subplot(1, 2, 2);
polarplot(theta, g_theta / max(g_theta));
title("Polar Array Pattern (Normalized)");
grid on;

%% Function to calculate g
function g_theta = g(theta, antenna_pattern, d_antennas, phi_antennas, fc)
    c = 3e8; 
    k = 2 * pi * fc / c;
    g_theta = zeros(1, length(theta));
    for i=1:length(theta)
        result = 0;
        for j=1:length(d_antennas)
            result = result + antenna_pattern(i) * exp(1i*k*d_antennas(j)*sin(theta(i))) * exp(1i*phi_antennas(j));
        end
        result = abs(result)^2 / length(d_antennas);
        g_theta(i) = result;
    end
end

