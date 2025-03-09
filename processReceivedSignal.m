function [target_snrs, target_ranges, target_dopplers, target_azimuths] = ...
    processReceivedSignal(fs, pulse, pulse_PRI, lambda, ...
    tau, rx_positions, noise_power, SNR_min, received_signal, ...
    speed_limit_km, speed_max_km, speed_sweep_step, do_plot)



    c = 3e8;
    % Calculate received signal FFT
    Yf = fft(received_signal.').';
    deltaFd = fs / length(pulse);
    PRF = fs / length(pulse_PRI);
    Yf=circshift(Yf,PRF/2/deltaFd,2);
    Fd = [-speed_max_km:speed_sweep_step:-speed_limit_km, ...
        speed_limit_km:speed_sweep_step:speed_max_km] ...
        / 3.6 * 2 / lambda;
    Fdlen = length(Fd);
    Theta=-90:0.1:90;
    MAP=exp(1j*2*pi/lambda*rx_positions'*sind(Theta));
    sl_PRI_f=fft(pulse_PRI);
    Rmax=c*(length(pulse_PRI)/fs)/2;
    deltaR=tau*c/2/tau/fs;
    R=0:deltaR:Rmax-deltaR;
    cc = 1;
    % Doppler, azimuth and range processing
    for i=1:Fdlen
        % Doppler filtering
        Z=Yf(:,round((PRF/2+Fd(i))/deltaFd)+1:PRF/deltaFd:end);
        SNR = 10 * log(mean(abs(Z(1, :)).^2) / (size(Yf, 2) * noise_power));
        if SNR < SNR_min
            continue;
        end
    
        % Angle estimation with MUSIC
        [U,~,~]=svd(Z);
        Unull=U(:,2:end); 
        ro=sum(abs(Unull'*MAP).^2);
        [~, ind] = max(-db(ro));
        azimuth = Theta(ind);
        % Range estimation
        [~,ind]=max(-db(ro));
        z=MAP(:,ind)'*Z;
        zif=ifft(z.*conj(sl_PRI_f));
        [~, ind] = max(abs(zif));
        range = R(ind);
        target_snrs(cc) = SNR;
        target_ranges(cc) = range;
        target_dopplers(cc) = Fd(i) * lambda / 2;
        target_azimuths(cc) = azimuth;
        cc = cc + 1;

        if do_plot
            Ns = size(Yf, 2);
            NCh = size(Yf, 1);
            NPRI = length(sl_PRI_f);
            NTr = Ns / NPRI;
            Yff = reshape(Yf.', NTr, NPRI, NCh);
            RD = ifft(Yff .* conj(sl_PRI_f), NPRI, 2);
            figure(22);
            [x, y] = meshgrid(R, -PRF/2:deltaFd:PRF/2-deltaFd);
            mesh(x, y, db(RD(:, :, 1)));
            title('Range Doppler Heatmap');
            xlabel('Range (m)');
            ylabel('Doppler (Hz)');
            figure(23);
            plot(Theta, -db(ro));
            title('AoA estimation with MUSIC');
            xlabel('Azimuth Angle $\phi$', Interpreter='latex');
            pause(0.5);
        end
    end
end

