function [reconstructed_signal, reconstruction_error, best_imfs, blind_snr] = part4_signal_reconstruction(...
    time, signal, IMFS_EMD, signal_imfs, mixed_imfs, filepath, sig_idx, segment_times, car_segment_indices)
% Reconstruct retained IMFs, estimate blind SNR, and optionally denoise.

fprintf('=== Part 4: signal reconstruction ===\n');

pure_imfs = setdiff(signal_imfs, mixed_imfs);

% Use noisy IMFs directly without searching combinations
best_imfs = unique([pure_imfs(:)', mixed_imfs(:)']);
if isempty(best_imfs)
    best_raw_reconstructed = zeros(1, size(IMFS_EMD, 2));
else
    best_raw_reconstructed = sum(IMFS_EMD(best_imfs, :), 1);
end

% ====== Compute blind SNR for logging ======
raw_recon_trunc = best_raw_reconstructed(1:min(length(time), length(best_raw_reconstructed)))';
time_col = time(:);
n_pts = min(length(time_col), length(raw_recon_trunc));
time_local = time_col(1:n_pts);
recon_local = raw_recon_trunc(1:n_pts);

num_segments = max(0, length(segment_times) - 1);
vehicle_mask = false(n_pts, 1);
for j = 1:num_segments
    st_time = segment_times(j);
    ed_time = segment_times(j + 1);
    if j < num_segments
        seg_mask = (time_local >= st_time) & (time_local < ed_time);
    else
        seg_mask = (time_local >= st_time) & (time_local <= ed_time);
    end
    if ismember(j, car_segment_indices)
        vehicle_mask = vehicle_mask | seg_mask;
    end
end
interf_mask = ~vehicle_mask;

if ~any(vehicle_mask) || ~any(interf_mask)
    best_snr = -Inf;
else
    baseline = safe_median(recon_local(interf_mask));
    recon_ac = recon_local - baseline;
    V_energy = safe_mean_square(recon_ac(vehicle_mask));
    I_energy = safe_mean_square(recon_ac(interf_mask));
    eps_val = 1e-12;
    best_snr = 10 * log10(V_energy / (I_energy + eps_val));
end

blind_snr = best_snr;

% This printed Blind SNR is computed on baseline-removed fluctuations.

fprintf('Reconstruction using unfiltered IMFs: %s, Blind SNR: %.2f dB\n', mat2str(best_imfs), blind_snr);

raw_reconstructed = best_raw_reconstructed;

dt = median(diff(time(:)));
fs_est = 1 / max(dt, eps);

% Denoising mode: 0=none, 1=wavelet, 2=moving_average, 3=moving_median
denoise_choice = 0;
switch denoise_choice
    case 0
        denoise_method = 'none';
    case 1
        denoise_method = 'wavelet';
    case 2
        denoise_method = 'moving_average';
    case 3
        denoise_method = 'moving_median';
    otherwise
        error('denoise_choice must be 0, 1, 2, or 3.');
end

% Apply denoising through denoise_signal.m
if denoise_choice == 0
    denoised = raw_reconstructed(:);
elseif exist('denoise_signal', 'file')
    denoised = denoise_signal(raw_reconstructed(:), denoise_method, fs_est);
else
    warning('denoise_signal.m not found; skipping built-in denoising');
    denoised = raw_reconstructed(:);
end
reconstructed_signal = denoised.';

% figure;
% subplot(2,1,1);
% plot(time(:), signal(:), 'k');
% % axis off;
% % title('Original Signal');
% % xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontWeight', 'bold');
% ylabel('Amplitude', 'FontName', 'Times New Roman', 'FontWeight', 'bold');
% set(gca, 'FontName', 'Times New Roman', 'FontWeight', 'bold');
% % grid on;

% % figure;
% subplot(2,1,2);
% % plot(time(:), raw_reconstructed(:), 'r--', 'LineWidth', 1.0, 'DisplayName', 'Reconstructed (Raw)');
% % hold on;
% plot(time(:), reconstructed_signal(:), 'b-', 'LineWidth', 1.2);
% % title('Reconstructed Signal (Raw vs Denoised)');
% xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontWeight', 'bold');
% ylabel('Amplitude', 'FontName', 'Times New Roman', 'FontWeight', 'bold');
% set(gca, 'FontName', 'Times New Roman', 'FontWeight', 'bold');
% % grid on;
% % legend('Location','best');
% % hold off;


% Compute reconstruction error
reconstruction_error = mean((signal - reconstructed_signal').^2);


% Save final results
save('final_results.mat', 'signal_imfs', 'reconstructed_signal', 'reconstruction_error');

end

function val = safe_mean_square(x)
    x = x(:);
    x = x(isfinite(x));
    if isempty(x)
        val = 0;
    else
        val = mean(x.^2);
    end
end

function val = safe_median(x)
    x = x(:);
    x = x(isfinite(x));
    if isempty(x)
        val = 0;
    else
        val = median(x);
    end
end
