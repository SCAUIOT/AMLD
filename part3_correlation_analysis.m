function [signal_imfs, noise_imfs, mixed_imfs, correlation_matrix] = part3_correlation_analysis(...
    time, signal, IMFS_EMD, segment_times, car_segment_indices)
% Classify IMFs using global and segment-wise correlation features.


%AMLD% Correlation analysis and IMF selection
fprintf('=== Part 3: correlation analysis and IMF selection ===\n');

num_imfs = size(IMFS_EMD, 1);
num_segments = length(segment_times) - 1;
if nargin < 5 || isempty(car_segment_indices)
    car_segment_indices = 2:2:num_segments;
end
car_segment_indices = unique(round(car_segment_indices(:)'));
car_segment_indices = car_segment_indices( ...
    car_segment_indices >= 1 & car_segment_indices <= num_segments);
background_segment_indices = setdiff(1:num_segments, car_segment_indices);

% Compute the correlation matrix
correlation_matrix = zeros(num_imfs, num_segments);

for j = 1:num_segments
    start_time = segment_times(j);
    end_time = segment_times(j+1);

    start_idx = find(time >= start_time, 1, 'first');
    end_idx = find(time <= end_time, 1, 'last');

    mixed_segment = signal(start_idx:end_idx);

    for i = 1:num_imfs
        imf_segment = IMFS_EMD(i, start_idx:end_idx);
        cval = corr(imf_segment', mixed_segment);
        if ~isfinite(cval)
            cval = 0;
        end
        correlation_matrix(i, j) = abs(cval);
    end
end

% Save the correlation matrix
save('correlation_matrix.mat', 'correlation_matrix');
% fprintf('Correlation matrix computed; dimensions: %d x %d\n', size(correlation_matrix, 1), size(correlation_matrix, 2));

% Compute correlation features
correlations = zeros(num_imfs, 1);
seg_diff_score_array = zeros(num_imfs, 1);

for i = 1:num_imfs
    imf_i = IMFS_EMD(i, :)';

    cval = corr(imf_i, signal(:));
    if ~isfinite(cval)
        cval = 0;
    end
    correlations(i) = abs(cval);

    if ~isempty(car_segment_indices) && ~isempty(background_segment_indices)
        vehicle_corr = mean(correlation_matrix(i, car_segment_indices));
        background_corr = mean(correlation_matrix(i, background_segment_indices));
        seg_score = vehicle_corr - background_corr;
    else
        % Use global correlation when too few segments are available for vehicle/background contrast
        seg_score = correlations(i);
    end
    seg_diff_score_array(i) = seg_score;
end

% Cluster IMF features using k-means
imf_features = [correlations(:), seg_diff_score_array(:)];
norm_imf_features = (imf_features - mean(imf_features, 1)) ./ (std(imf_features, 0, 1) + 1e-8);

% Replace nonfinite values before k-means
norm_imf_features(~isfinite(norm_imf_features)) = 0;

% Limit the number of clusters to the available samples
K = min(3, max(1, size(norm_imf_features, 1) - 1));

if K >= 2
    try
        [imf_idx, C] = kmeans(norm_imf_features, K, 'Replicates', 10);
    catch
        [~, order] = sort(correlations, 'ascend');
        imf_idx = ones(num_imfs, 1);
        if K == 2
            cut = max(1, floor(num_imfs/2));
            imf_idx(order(cut+1:end)) = 2;
            C = [mean(norm_imf_features(imf_idx==1, :), 1); mean(norm_imf_features(imf_idx==2, :), 1)];
        else
            C = mean(norm_imf_features, 1);
        end
    end
else
    imf_idx = ones(num_imfs, 1);
    C = mean(norm_imf_features, 1);
end

mean_corrs = -inf(1, K);
for c = 1:K
    mean_corrs(c) = mean(correlations(imf_idx == c));
    if isnan(mean_corrs(c))
        mean_corrs(c) = -inf;
    end
end

[~, sorted_idx] = sort(mean_corrs, 'ascend');

re_imf_idx = imf_idx;

final_class = zeros(num_imfs, 1);
if K >= 3
    final_class(re_imf_idx == sorted_idx(1)) = 0;
    final_class(re_imf_idx == sorted_idx(2)) = 1;
    final_class(re_imf_idx == sorted_idx(3)) = 2;
elseif K == 2
    final_class(re_imf_idx == sorted_idx(1)) = 0;
    final_class(re_imf_idx == sorted_idx(2)) = 2;
else
    final_class(:) = 2;
end

mean_global_corr = mean(correlations);
mean_seg_score = mean(seg_diff_score_array);

for i = 1:num_imfs
    if correlations(i) > mean_global_corr && seg_diff_score_array(i) < mean_seg_score
        final_class(i) = 0;
    end
end

signal_imfs = find(final_class == 1 | final_class == 2)';
noise_imfs = find(final_class == 0)';
mixed_imfs = find(final_class == 1)';

% Retain the highest-scoring IMF if clustering would discard every mode
if isempty(signal_imfs) && num_imfs > 0
    [~, best_idx] = max(correlations + max(seg_diff_score_array, 0));
    final_class(best_idx) = 2;
    signal_imfs = best_idx;
    noise_imfs = setdiff(1:num_imfs, best_idx);
    mixed_imfs = [];
end


fprintf('Selected signal IMFs: %s\n', mat2str(signal_imfs));
fprintf('Included mixed signal/noise IMFs: %s\n', mat2str(mixed_imfs));
fprintf('Noise IMFs: %s\n', mat2str(noise_imfs));


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

function plot_entropy_results(time, signal, segment_times, entropy_times, entropy_values, low_point_indices)
    figure('Position', [100, 100, 1200, 800]);

    subplot(2, 1, 1);
    plot(time, signal, 'b-', 'LineWidth', 0.8);
    hold on;
    for i = 1:length(segment_times)
        xline(segment_times(i), 'r--', 'Alpha', 0.7, 'LineWidth', 1.2);
    end
    ylabel('Signal amplitude');
    title('Signal waveform and boundaries based on entropy minima');
    grid on;
    legend('Original signal', 'Segment boundaries', 'Location', 'northeast');

    subplot(2, 1, 2);
    plot(entropy_times, entropy_values, 'g-', 'LineWidth', 1.2);
    hold on;

    plot(entropy_times(low_point_indices), entropy_values(low_point_indices), ...
        'ro', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'r');

    xlabel('Time');
    ylabel('Entropy');
    title('Sliding-window entropy (red circles indicate local minima)');
    grid on;
    legend('Entropy curve', 'Local minima', 'Location', 'northeast');
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
