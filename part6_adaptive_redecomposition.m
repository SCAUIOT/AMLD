function [IMFS_EMD, signal_imfs, noise_imfs, mixed_imfs, reconstructed_signal, similarity, current_sim, current_snr, history_str, opt_status] = ...
    part6_adaptive_redecomposition(time, signal, segment_times, IMFS_EMD, signal_imfs, noise_imfs, mixed_imfs, ...
    reconstructed_signal, similarity, filepath, sig_idx, alpha, tau, DC, init, tol, ~, car_segment_indices)

fprintf('\n=== Part 6: adaptive IMF redecomposition (blind SNR criterion)===\n');

if isstruct(similarity) && isfield(similarity, 'Best')
    current_sim = similarity.Best.Result.Corr;
else
    current_sim = 0;
end
if ~isfinite(current_sim)
    current_sim = 0;
end

if nargin < 18 || isempty(car_segment_indices)
    car_segment_indices = get_part1_car_segment_indices(segment_times);
end
eps_blind_snr = 1e-12;
[current_snr, current_Ev, current_EI] = calc_blind_snr_by_segments( ...
    time, reconstructed_signal, segment_times, car_segment_indices, eps_blind_snr);

fprintf('Current best similarity: %.4f, Blind SNR: %.4f dB (Ev=%.6g, EI=%.6g)\n', ...
    current_sim, current_snr, current_Ev, current_EI);

history_str = sprintf('Initial(Sim:%.4f, BlindSNR:%.4f)', current_sim, current_snr);

target_imfs = mixed_imfs;

opt_status = 0; % 0: unchanged, 1: improved, -1: rolled back

max_recomp_iter = 5;
iter_recomp = 0;

while iter_recomp < max_recomp_iter
    iter_recomp = iter_recomp + 1;
    fprintf('\n--- Redecomposition iteration %d / %d ---\n', iter_recomp, max_recomp_iter);

    if isempty(target_imfs)
        fprintf('  No mixed signal/noise IMFs require redecomposition; stopping.\n');
        break;
    end

    fprintf('  Target mixed IMF indices: %s\n', mat2str(target_imfs));

    IMFS_EMD_bak     = IMFS_EMD;
    signal_imfs_bak  = signal_imfs;
    noise_imfs_bak   = noise_imfs;
    mixed_imfs_bak   = mixed_imfs;
    rec_sig_bak      = reconstructed_signal;
    similarity_bak   = similarity;

    new_IMFS_rows = {};
    for i = 1:size(IMFS_EMD, 1)
        if ismember(i, target_imfs)
            fprintf('  >> Redecomposing mixed IMF %d...\n', i);
            orig_imf_signal = IMFS_EMD(i, :).';
            optimal_K_sub = 15;
            [sub_u, ~, sub_omega] = VMD(orig_imf_signal, alpha, tau, optimal_K_sub, DC, init, tol);
            if size(sub_u, 2) ~= numel(orig_imf_signal)
                sub_u = sub_u.';
            end
            [sub_u, sub_order, ~] = sort_imfs_by_dominant_frequency(sub_u, sub_omega, 'descend');
            fprintf('     [VMD] Sub-modes reordered high-to-low frequency. Original order: %s\n', mat2str(sub_order));

            fprintf('     [Local selection] Analyzing %d submodes...\n', size(sub_u, 1));
            [sub_sig_idxs, ~, ~, ~] = part3_correlation_analysis( ...
                time, signal, sub_u, segment_times, car_segment_indices);

            if isempty(sub_sig_idxs)
                fprintf('     [Warning] No signal submodes detected; retaining all submodes.\n');
                recons_imf = sum(sub_u, 1);
            else
                fprintf('     [Local selection] Selected submode indices: %s\n', mat2str(sub_sig_idxs(:)'));
                recons_imf = sum(sub_u(sub_sig_idxs, :), 1);
            end

            new_IMFS_rows{end+1} = recons_imf; %#ok<AGROW>
        else
            new_IMFS_rows{end+1} = IMFS_EMD(i, :); %#ok<AGROW>
        end
    end

    IMFS_EMD = cat(1, new_IMFS_rows{:});
    fprintf('  IMF count after this iteration: %d\n', size(IMFS_EMD, 1));

    [signal_imfs, noise_imfs, mixed_imfs, ~] = part3_correlation_analysis( ...
        time, signal, IMFS_EMD, segment_times, car_segment_indices);

    [reconstructed_signal, ~, used_best_imfs] = part4_signal_reconstruction( ...
        time, signal, IMFS_EMD, signal_imfs, mixed_imfs, filepath, sig_idx, segment_times, car_segment_indices);
    mixed_imfs = intersect(mixed_imfs, used_best_imfs);

    new_sim = current_sim;
    if ~isstruct(similarity) || ~isfield(similarity, 'Best') || ~isfield(similarity.Best, 'Result')
        similarity = struct('Best', struct('Result', struct( ...
            'Signal', reconstructed_signal, 'Corr', current_sim, 'SNR', current_snr)));
    else
        similarity.Best.Result.Signal = reconstructed_signal;
    end
    [new_snr, new_Ev, new_EI] = calc_blind_snr_by_segments( ...
        time, reconstructed_signal, segment_times, car_segment_indices, eps_blind_snr);
    similarity.Best.Result.SNR = new_snr;

    snr_gain = new_snr - current_snr;
    vehicle_energy_retained = current_Ev <= 0 || new_Ev >= 0.90 * current_Ev;
    snr_improved = isfinite(new_snr) && (~isfinite(current_snr) || snr_gain > 0) ...
        && vehicle_energy_retained;
    if ~snr_improved
        fprintf('  [Rollback] Blind SNR did not improve or retained vehicle energy was insufficient (%.4f -> %.4f dB, Ev=%.6g, EI=%.6g). Restoring the previous result and stopping.\n', ...
            current_snr, new_snr, new_Ev, new_EI);
        history_str = [history_str, sprintf(' -> Rollback(Sim:%.4f, BlindSNR:%.4f)', current_sim, current_snr)]; %#ok<AGROW>

        IMFS_EMD             = IMFS_EMD_bak;
        signal_imfs          = signal_imfs_bak;
        noise_imfs           = noise_imfs_bak;
        mixed_imfs           = mixed_imfs_bak;
        reconstructed_signal = rec_sig_bak;
        similarity           = similarity_bak;

        if opt_status == 0
            opt_status = -1;
        end
        break;
    else
        fprintf('  [Accepted] Blind SNR improved (%.4f -> %.4f dB, Ev=%.6g, EI=%.6g). Continuing.\n', ...
            current_snr, new_snr, new_Ev, new_EI);
        history_str = [history_str, sprintf(' -> Redecomp%d(Sim:%.4f, BlindSNR:%.4f)', iter_recomp, new_sim, new_snr)]; %#ok<AGROW>

        current_sim = new_sim;
        current_snr = new_snr;
        target_imfs = mixed_imfs;
        opt_status = 1;

        if isempty(target_imfs)
            fprintf('  No mixed IMFs remain; stopping.\n');
            break;
        end
    end
end

end

function car_segment_indices = get_part1_car_segment_indices(segment_times)
% Prefer car_segment_indices generated by part1. If unavailable, fall back
% to part1's usual alternating segments: background/vehicle/background/...

    num_segments = max(0, numel(segment_times) - 1);
    car_segment_indices = [];

    try
        has_caller_var = evalin('caller', 'exist(''car_segment_indices'', ''var'')');
        if has_caller_var
            car_segment_indices = evalin('caller', 'car_segment_indices');
        end
    catch
        car_segment_indices = [];
    end

    if isempty(car_segment_indices)
        try
            has_base_var = evalin('base', 'exist(''car_segment_indices'', ''var'')');
            if has_base_var
                car_segment_indices = evalin('base', 'car_segment_indices');
            end
        catch
            car_segment_indices = [];
        end
    end

    car_segment_indices = car_segment_indices(:).';
    car_segment_indices = car_segment_indices( ...
        isfinite(car_segment_indices) & car_segment_indices >= 1 & car_segment_indices <= num_segments);
    car_segment_indices = unique(round(car_segment_indices));

    if isempty(car_segment_indices)
        car_segment_indices = 2:2:num_segments;
    end
end

function [blind_snr, Ev, EI] = calc_blind_snr_by_segments(time, recon_signal, segment_times, car_segment_indices, eps_val)
% AC Blind SNR = 10*log10(Ev / (EI + eps)).
% Estimate the baseline from pure-interference segments, then compare
% baseline-removed fluctuation energy between vehicle and interference parts.

    if nargin < 5 || isempty(eps_val)
        eps_val = 1e-12;
    end

    time = time(:);
    recon_signal = recon_signal(:);
    n = min(numel(time), numel(recon_signal));
    time = time(1:n);
    recon_signal = recon_signal(1:n);

    [vehicle_mask, interference_mask] = build_part1_segment_masks(time, segment_times, car_segment_indices);

    if ~any(vehicle_mask) || ~any(interference_mask)
        warning('part6:BlindSNRMaskEmpty', ...
            'Vehicle or interference-only segments are empty; blind SNR set to -Inf.');
        baseline = safe_median(recon_signal);
        recon_ac = recon_signal - baseline;
        Ev = 0;
        EI = safe_mean_square(recon_ac);
        blind_snr = NaN;
        return;
    end

    baseline = safe_median(recon_signal(interference_mask));
    recon_ac = recon_signal - baseline;
    Ev = safe_mean_square(recon_ac(vehicle_mask));
    EI = safe_mean_square(recon_ac(interference_mask));

    if Ev <= 0
        blind_snr = NaN;
    else
        blind_snr = 10 * log10(Ev / (EI + eps_val));
    end
    if ~isfinite(blind_snr)
        blind_snr = NaN;
    end
end

function [vehicle_mask, interference_mask] = build_part1_segment_masks(time, segment_times, car_segment_indices)
    time = time(:);
    n = numel(time);
    vehicle_mask = false(n, 1);
    interference_mask = false(n, 1);

    segment_times = segment_times(:).';
    if numel(segment_times) < 2
        interference_mask(:) = true;
        return;
    end

    num_segments = numel(segment_times) - 1;
    car_segment_indices = unique(round(car_segment_indices(:).'));
    car_segment_indices = car_segment_indices( ...
        car_segment_indices >= 1 & car_segment_indices <= num_segments);

    for j = 1:num_segments
        start_time = segment_times(j);
        end_time = segment_times(j + 1);

        if j < num_segments
            seg_mask = (time >= start_time) & (time < end_time);
        else
            seg_mask = (time >= start_time) & (time <= end_time);
        end

        if ismember(j, car_segment_indices)
            vehicle_mask = vehicle_mask | seg_mask;
        else
            interference_mask = interference_mask | seg_mask;
        end
    end
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
