function [time, signal, segment_times, entropy_times, entropy_curve, car_segment_indices] = ...
    part1_sliding_entropy_analysis(time, signal)
% Compute window features, cluster vehicle intervals, and return segment boundaries.

fprintf('=== Part 1: sliding entropy and energy clustering ===\n');

fs = 1 / (time(2) - time(1));
N = length(signal);

% 1. Compute sliding entropy, local energy, and local variance
win_len_auto = max(20, round(N * 0.10));
step_auto = 1;
num_bins = 20;

entropy_curve = zeros(N, 1);
energy_curve  = zeros(N, 1);
var_curve     = zeros(N, 1);
fprintf('Computing sliding entropy, local energy, and local variance for %d samples...\n', N);

for i = 1:step_auto:N
    start_idx = max(1, i - floor(win_len_auto/2));
    end_idx   = min(N, i + floor(win_len_auto/2));

    win_data = signal(start_idx:end_idx);

    energy_curve(i) = mean(win_data.^2);

    var_curve(i) = var(win_data);

    win_data = win_data - mean(win_data);
    win_std = std(win_data);
    if win_std > 1e-6
        win_data = win_data / win_std;
    end

    [counts, ~] = histcounts(win_data, num_bins);
    p = counts / sum(counts);
    p = p(p > 0);

    entropy_curve(i) = -sum(p .* log2(p));
end

% ====== Validate feature data ======
if any(~isfinite(signal))
    fprintf('[Warning] The input contains NaN/Inf; features may become NaN.\n');
end
if all(energy_curve == 0) || std(energy_curve) < 1e-12
    fprintf('[Warning] Energy is nearly constant; normalization may produce NaN.\n');
end
if all(var_curve == 0) || std(var_curve) < 1e-12
    fprintf('[Warning] Variance is nearly constant; normalization may produce NaN.\n');
end

entropy_std = std(entropy_curve);
if entropy_std < 1e-12
    entropy_std = 1;
end
norm_entropy = (entropy_curve - mean(entropy_curve)) / entropy_std;

energy_fluct = abs(energy_curve - median(energy_curve));
energy_std = std(energy_fluct);
if energy_std < 1e-12
    energy_std = 1;
end
norm_energy  = (energy_fluct - mean(energy_fluct)) / energy_std;

var_fluct    = abs(var_curve - median(var_curve));
var_std = std(var_fluct);
if var_std < 1e-12
    var_std = 1;
end
norm_var     = (var_fluct - mean(var_fluct)) / var_std;
feature_matrix = [norm_entropy, norm_energy, norm_var];
% disp(feature_matrix(1:5, :));
% 2. Segment the signal by clustering
% fprintf('Clustering multiple features with 2-means to extract vehicle intervals...\n');
[idx, C] = kmeans(feature_matrix, 2, 'Replicates', 5);

if C(1, 1) < C(2, 1)
    car_cluster_id = 1;
    bg_cluster_id  = 2;
else
    car_cluster_id = 2;
    bg_cluster_id  = 1;
end
is_car_base = (idx == car_cluster_id);

D_car = sum((feature_matrix - C(car_cluster_id, :)).^2, 2);
D_bg  = sum((feature_matrix - C(bg_cluster_id,  :)).^2, 2);

max_relax_attempts = 15;
relax_multiplier = 1.0;
var_relax_multiplier = 1.0;

for attempt = 1:max_relax_attempts
    if attempt == 1
        is_car = is_car_base;
    else
        var_relax_multiplier = var_relax_multiplier - 0.05;
        if var_relax_multiplier < 0.1
            var_relax_multiplier = 0.1;
        end

        var_threshold_dist = abs(norm_var - C(bg_cluster_id, 3));
        bg_var_center = C(bg_cluster_id, 3);
        car_var_center = C(car_cluster_id, 3);
        var_delta = abs(car_var_center - bg_var_center);

        relax_var_condition = (var_threshold_dist > (var_delta * 0.5 * var_relax_multiplier));
        relax_multiplier = relax_multiplier + 0.3;
        is_car = (D_car < D_bg * relax_multiplier) | relax_var_condition;
    end

    merge_gap = round(N * 0.10);
    min_len   = round(N * 0.05);
    edge_margin = round(N * 0.05);

    diff_is_car = diff([0; is_car; 0]);
    start_indices = find(diff_is_car == 1);
    end_indices = find(diff_is_car == -1) - 1;

    if ~isempty(start_indices)
        merged_start = start_indices(1);
        merged_end   = end_indices(1);
        for k = 2:length(start_indices)
            if start_indices(k) - merged_end(end) < merge_gap
                merged_end(end) = end_indices(k);
            else
                merged_start = [merged_start; start_indices(k)];
                merged_end   = [merged_end; end_indices(k)];
            end
        end
        start_indices = merged_start;
        end_indices   = merged_end;

        valid_mask = (end_indices - start_indices + 1) >= min_len;
        start_indices = start_indices(valid_mask);
        end_indices   = end_indices(valid_mask);

        edge_mask = (start_indices == 1 & end_indices >= edge_margin) | ...
                    (start_indices <= (N - edge_margin) & end_indices == N);
        start_indices(edge_mask) = [];
        end_indices(edge_mask)   = [];
    end

    temp_boundaries = unique([1; start_indices(:); end_indices(:); N]);
    num_segments_temp = length(temp_boundaries) - 1;

    if num_segments_temp >= 5
        break;
    end
end

if num_segments_temp < 5
    var_hard_thresh = mean(var_curve);
    is_car_hard = (var_curve > var_hard_thresh);

    diff_is_car = diff([0; is_car_hard; 0]);
    start_indices = find(diff_is_car == 1);
    end_indices = find(diff_is_car == -1) - 1;

    if ~isempty(start_indices)
        merged_start = start_indices(1);
        merged_end   = end_indices(1);
        for k = 2:length(start_indices)
            if start_indices(k) - merged_end(end) < merge_gap
                merged_end(end) = end_indices(k);
            else
                merged_start = [merged_start; start_indices(k)];
                merged_end   = [merged_end; end_indices(k)];
            end
        end
        start_indices = merged_start;
        end_indices   = merged_end;

        valid_mask = (end_indices - start_indices + 1) >= min_len;
        start_indices = start_indices(valid_mask);
        end_indices   = end_indices(valid_mask);

        edge_mask = (start_indices == 1 & end_indices >= edge_margin) | ...
                    (start_indices <= (N - edge_margin) & end_indices == N);
        start_indices(edge_mask) = [];
        end_indices(edge_mask)   = [];
    end
end

% Discard vehicle intervals shorter than 10 percent of the signal
min_car_len = round(N * 0.10);
if ~isempty(start_indices)
    seg_lengths = end_indices - start_indices + 1;
    keep_mask = seg_lengths >= min_car_len;
    start_indices = start_indices(keep_mask);
    end_indices = end_indices(keep_mask);
end

is_car(:) = 0;
for k = 1:length(start_indices)
    is_car(start_indices(k):end_indices(k)) = 1;
end

% 3. Build the output segments
segment_times = [time(1)];
car_segment_indices = [];
current_seg_idx = 0;

for k = 1:length(start_indices)
    c_start = time(start_indices(k));
    c_end = time(end_indices(k));

    if c_start > segment_times(end)
        segment_times(end+1) = c_start;
        current_seg_idx = current_seg_idx + 1;
    end

    segment_times(end+1) = c_end;
    current_seg_idx = current_seg_idx + 1;
    car_segment_indices(end+1) = current_seg_idx;
end

if segment_times(end) < time(end)
    segment_times(end+1) = time(end);
end
segment_times = segment_times(:)';

entropy_times = time;
num_segments = length(segment_times) - 1;

fprintf('Final segment count: %d; vehicle interval indices: %s\n', num_segments, mat2str(car_segment_indices));

% plot_new_segment_results(time, signal, segment_times, car_segment_indices, is_car);

end

%% ====== Helper functions ======

function entropy = calc_entropy(segment, num_bins)
    if nargin < 2
        num_bins = 30;
    end
    [hist_counts, bin_edges] = histcounts(segment, num_bins, 'Normalization', 'pdf');
    prob = hist_counts .* diff(bin_edges);
    prob = prob(prob > 0);
    entropy = -sum(prob .* log(prob));
end

function [positions, entropies, window_info] = sliding_entropy(signal, time, win_len, step, num_bins)
    if nargin < 5
        num_bins = 30;
    end
    entropies = [];
    positions = [];
    window_info = struct([]);
    for start = 1:step:(length(signal) - win_len + 1)
        seg = signal(start:start + win_len - 1);
        H = calc_entropy(seg, num_bins);
        entropies = [entropies; H];
        center_index = start + floor(win_len / 2);
        center_time = time(center_index);
        positions = [positions; center_time];
        win_info = struct(...
            'start_index', start, ...
            'end_index', start + win_len - 1, ...
            'start_time', time(start), ...
            'end_time', time(start + win_len - 1), ...
            'center_time', center_time, ...
            'entropy', H);
        window_info = [window_info; win_info];
    end
end

function plot_new_segment_results(time, signal, segment_times, car_segment_indices, is_car)
    figure('Position', [100, 100, 1200, 400]);

    plot(time, signal, 'k-', 'LineWidth', 0.8);
    hold on;

    y_min = min(signal);
    y_max = max(signal);

    for i = 1:length(segment_times)
        xline(segment_times(i), 'b--', 'Alpha', 0.5, 'LineWidth', 1.2);
    end

    for i = 1:length(car_segment_indices)
        seg_idx = car_segment_indices(i);
        t_start = segment_times(seg_idx);
        t_end   = segment_times(seg_idx+1);
        patch([t_start t_end t_end t_start], [y_min y_min y_max y_max], 'r', ...
          'FaceAlpha', 0.2, 'EdgeColor', 'none');
    end

    ylabel('Signal amplitude');
    xlabel('Time (s)');
    title('Signal waveform and boundaries (red: vehicle intervals detected by clustering)');
    grid on;
end

function [signal_imfs, noise_imfs, analysis_results] = advanced_imf_identification(S, min_correlation_threshold)
    [M, K] = size(S);

    mean_corr = mean(S, 2);
    std_corr = std(S, 0, 2);
    max_corr = max(S, [], 2);

    z_scores = (max_corr - mean(max_corr)) / std(max_corr);
    high_corr_outliers = find(z_scores > 0.5);

    stability_scores = 1 - (std_corr ./ (mean_corr + eps));

    pattern_scores = zeros(M, 1);
    for i = 1:M
        if K > 1
            acf = autocorr(S(i, :), 1);
            pattern_scores(i) = abs(acf(2));
        else
            pattern_scores(i) = 0;
        end
    end

    combined_scores = zeros(M, 1);
    for i = 1:M
        if ismember(i, high_corr_outliers)
            penalty = 0.5;
        else
            penalty = 1;
        end
        combined_scores(i) = penalty * (0.6 * mean_corr(i) + 0.2 * stability_scores(i) + 0.2 * pattern_scores(i));
    end

    normalized_scores = (combined_scores - min(combined_scores)) / (max(combined_scores) - min(combined_scores));
    threshold = graythresh(normalized_scores);

    signal_candidates = find(combined_scores >= threshold);

    if ~ismember(1, signal_candidates)
        signal_candidates = [1; signal_candidates];
    end
    if ~ismember(2, signal_candidates)
        signal_candidates = [2; signal_candidates];
    end

    signal_imfs = unique(signal_candidates);
    noise_imfs = setdiff(1:M, signal_imfs);

    analysis_results.mean_corr = mean_corr;
    analysis_results.std_corr = std_corr;
    analysis_results.stability_scores = stability_scores;
    analysis_results.pattern_scores = pattern_scores;
    analysis_results.combined_scores = combined_scores;
    analysis_results.high_corr_outliers = high_corr_outliers;
    analysis_results.threshold = threshold;

    visualize_advanced_analysis(S, analysis_results, signal_imfs, noise_imfs);

    fprintf('Identified signal IMFs: %s\n', mat2str(signal_imfs'));
    fprintf('Identified noise IMFs: %s\n', mat2str(noise_imfs'));
end

function visualize_advanced_analysis(S, results, signal_imfs, noise_imfs)
    [M, K] = size(S);

    figure('Position', [100, 100, 1400, 800]);

    subplot(2, 3, 1);
    imagesc(S);
    colorbar;
    xlabel('Segment Index');
    ylabel('IMF Index');
    title('Correlation Matrix Heatmap');

    hold on;
    for i = 1:length(signal_imfs)
        plot([0, K+1], [signal_imfs(i)-0.5, signal_imfs(i)-0.5], 'g-', 'LineWidth', 2);
    end
    for i = 1:length(noise_imfs)
        plot([0, K+1], [noise_imfs(i)-0.5, noise_imfs(i)-0.5], 'r-', 'LineWidth', 2);
    end

    subplot(2, 3, 2);
    errorbar(1:M, results.mean_corr, results.std_corr, 'o', 'LineWidth', 1.5);
    hold on;

    for i = 1:length(signal_imfs)
        idx = signal_imfs(i);
        text(idx, results.mean_corr(idx), 'S', 'Color', 'green', 'FontWeight', 'bold', 'FontSize', 12);
    end
    for i = 1:length(noise_imfs)
        idx = noise_imfs(i);
        text(idx, results.mean_corr(idx), 'N', 'Color', 'red', 'FontWeight', 'bold', 'FontSize', 12);
    end

    xlabel('IMF Index');
    ylabel('Correlation');
    title('Mean Correlation ± Std Dev');
    grid on;

    subplot(2, 3, 3);
    bar(results.combined_scores);
    hold on;
    plot(xlim, [results.threshold results.threshold], 'r--', 'LineWidth', 2);

    for i = 1:length(signal_imfs)
        idx = signal_imfs(i);
        text(idx, results.combined_scores(idx), 'S', 'Color', 'green', 'FontWeight', 'bold', 'FontSize', 12);
    end
    for i = 1:length(noise_imfs)
        idx = noise_imfs(i);
        text(idx, results.combined_scores(idx), 'N', 'Color', 'red', 'FontWeight', 'bold', 'FontSize', 12);
    end

    xlabel('IMF Index');
    ylabel('Combined Score');
    title('Combined Scores with Threshold');
    grid on;

    subplot(2, 3, 4);
    bar(results.stability_scores);
    xlabel('IMF Index');
    ylabel('Stability Score');
    title('Stability Scores (1 - CV)');
    grid on;

    subplot(2, 3, 5);
    bar(results.pattern_scores);
    xlabel('IMF Index');
    ylabel('Pattern Score');
    title('Pattern Scores (Autocorrelation)');
    grid on;

    subplot(2, 3, 6);
    if ~isempty(signal_imfs)
        signal_correlations = reshape(S(signal_imfs, :), [], 1);
        histogram(signal_correlations, 20, 'FaceColor', 'green', 'FaceAlpha', 0.5);
        hold on;
    end
    if ~isempty(noise_imfs)
        noise_correlations = reshape(S(noise_imfs, :), [], 1);
        histogram(noise_correlations, 20, 'FaceColor', 'red', 'FaceAlpha', 0.5);
    end
    xlabel('Correlation Value');
    ylabel('Frequency');
    title('Correlation Distribution: Signal vs Noise IMFs');
    legend('Signal IMFs', 'Noise IMFs');
    grid on;
end
