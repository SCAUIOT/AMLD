function [seg_info] = part7_denoised_window_features(time1, sig1, time2, sig2, sample_filename)
% Inputs: two timestamp vectors and denoised signals, plus a sample filename.
% Output: segment information, DTW re-identification metrics, and available speed estimates.
% ====== Part 7: denoised window features, vehicle detection and plots ======

fprintf('=== Part 7: denoised window features and vehicle detection ===\n');

seg_info = struct();
seg_info.signal1 = analyze_one_signal(time1, sig1, 'Signal 1');
seg_info.signal2 = analyze_one_signal(time2, sig2, 'Signal 2');

% ====== DTW re-identification evaluation (ReID-ACC) ======
[reid_acc, dtw_info] = evaluate_reid_dtw(sig1, sig2, seg_info.signal1, seg_info.signal2);
seg_info.reid_acc = reid_acc;
seg_info.reid = dtw_info;
if isfinite(reid_acc)
    fprintf('ReID-ACC = %.2f%%  (N_matched=%d, N_pairs=%d)\n', ...
        reid_acc, dtw_info.N_matched, dtw_info.N_pairs);
else
    fprintf('ReID-ACC unavailable: at least 2 vehicle-segment pairs are required; N_pairs=%d.\n', dtw_info.N_pairs);
end
% if dtw_info.N_pairs > 0
%     plot_reid_dtw_reference(sig1, sig2, seg_info.signal1, seg_info.signal2, dtw_info, sample_filename);
% end

% ====== Speed estimation and comparison ======
detected_kmh = NaN;
if ~isempty(seg_info.signal1.segment_times) && ~isempty(seg_info.signal2.segment_times)
    if ~isempty(seg_info.signal1.car_segment_indices) && ~isempty(seg_info.signal2.car_segment_indices)
        s1_idx = seg_info.signal1.car_segment_indices(1);
        s2_idx = seg_info.signal2.car_segment_indices(1);
        t1 = seg_info.signal1.segment_times(s1_idx);
        t2 = seg_info.signal2.segment_times(s2_idx);
        % Convert timestamps from milliseconds to seconds
        dt = abs(t1 - t2) / 1000;
        if dt > 0
            detected_kmh = (8 / dt) * 3.6;
        end
    end
end

% Read reference speeds from Speed_label.xlsx for comparison
speed_label_file = 'E:\Postdata\yan2\datasets\speed_detection\clean\new\split\Speed_label.xlsx';
if nargin >= 5 && isfile(speed_label_file)
    T = readtable(speed_label_file);
    raw_names = string(T.Properties.VariableNames);
    norm_names = lower(strrep(strtrim(raw_names), ' ', ''));

    filename_col_idx = find(contains(norm_names, char([25991 20214 21517])) | contains(norm_names, "filename"), 1);
    speed_col_idx = find(contains(norm_names, char([34892 39542 36895 24230])) | contains(norm_names, "kmh") | contains(norm_names, char([36895 24230])), 1);

    if ~isempty(filename_col_idx) && ~isempty(speed_col_idx)
        filename_col = raw_names(filename_col_idx);
        speed_col = raw_names(speed_col_idx);

        [~, base_name, ext] = fileparts(sample_filename);
        key1 = [base_name, ext];
        key2 = [regexprep(base_name, '\+.*(?=_[xyz]$)', ''), ext];
        key3 = [regexprep(regexprep(base_name, '\+.*(?=_[xyz]$)', ''), '(_[xyz])$', ''), ext];

        match_idx = find(strcmp(T.(filename_col), key1), 1);
        if isempty(match_idx)
            match_idx = find(strcmp(T.(filename_col), key2), 1);
        end
        if isempty(match_idx)
            match_idx = find(strcmp(T.(filename_col), key3), 1);
        end
        if ~isempty(match_idx)
            true_kmh = T.(speed_col)(match_idx);
            seg_info.detected_kmh = detected_kmh;
            seg_info.true_kmh = true_kmh;
            seg_info.kmh_error = detected_kmh - true_kmh;
            % fprintf('Detected speed: %.3f km/h, Labeled speed: %.3f km/h, Difference: %.3f km/h\n', ...
            %     detected_kmh, true_kmh, detected_kmh - true_kmh);
        else
            fprintf('[Warning] No matching filename in Speed_label.xlsx: %s\n', base_name);
        end
    else
        fprintf('[Warning] Filename/speed columns not found in Speed_label.xlsx; check the column names.\n');
    end
else
    if nargin >= 5
        fprintf('[Warning] Speed_label.xlsx not found; speed comparison is unavailable.\n');
    end
end

% figure('Name', 'Denoised Signal - Car Segments', 'Position', [100, 100, 1200, 600]);

% subplot(2,1,1);
% plot(time1, sig1, 'k-', 'LineWidth', 0.8);
% hold on;
% plot_segments(sig1, seg_info.signal1.segment_times, seg_info.signal1.car_segment_indices);
% title('Signal 1 - Car Segments');
% xlabel('Time'); ylabel('Amplitude');
% grid off;
% apply_axis_style(gca);

% subplot(2,1,2);
% plot(time2, sig2, 'k-', 'LineWidth', 0.8);
% hold on;
% plot_segments(sig2, seg_info.signal2.segment_times, seg_info.signal2.car_segment_indices);
% title('Signal 2 - Car Segments');
% xlabel('Time'); ylabel('Amplitude');
% grid off;
% apply_axis_style(gca);

end

%% ====== Helper: process one signal ======
function info = analyze_one_signal(time, signal, label)
    N = length(signal);
    win_len = max(20, round(N * 0.10));
    step = 1;
    % num_bins = 20;

    energy_curve  = zeros(N, 1);
    var_curve     = zeros(N, 1);

    for i = 1:step:N
        s = max(1, i - floor(win_len/2));
        e = min(N, i + floor(win_len/2));
        win = signal(s:e);

        energy_curve(i) = mean(win.^2);
        var_curve(i) = var(win);

    end

    energy_fluct = abs(energy_curve - median(energy_curve));
    energy_std = std(energy_fluct);
    if energy_std < 1e-12, energy_std = 1; end
    norm_energy  = (energy_fluct - mean(energy_fluct)) / energy_std;

    var_fluct = abs(var_curve - median(var_curve));
    var_std = std(var_fluct);
    if var_std < 1e-12, var_std = 1; end
    norm_var = (var_fluct - mean(var_fluct)) / var_std;

    energy_thr = mean(norm_energy) + 0.5 * std(norm_energy);
    var_thr = mean(norm_var) + 0.5 * std(norm_var);
    is_car = (norm_energy > energy_thr) | (norm_var > var_thr);

    merge_gap = round(N * 0.10);
    min_len   = round(N * 0.05);

    diff_is_car = diff([0; is_car; 0]);
    start_idx = find(diff_is_car == 1);
    end_idx = find(diff_is_car == -1) - 1;

    if ~isempty(start_idx)
        merged_start = start_idx(1);
        merged_end   = end_idx(1);
        for k = 2:length(start_idx)
            if start_idx(k) - merged_end(end) < merge_gap
                merged_end(end) = end_idx(k);
            else
                merged_start = [merged_start; start_idx(k)];
                merged_end   = [merged_end; end_idx(k)];
            end
        end
        start_idx = merged_start;
        end_idx   = merged_end;

        valid_mask = (end_idx - start_idx + 1) >= min_len;
        start_idx = start_idx(valid_mask);
        end_idx   = end_idx(valid_mask);
    end

    % Discard vehicle intervals shorter than 10 percent of the signal
    min_car_len = round(N * 0.10);
    if ~isempty(start_idx)
        seg_lengths = end_idx - start_idx + 1;
        keep_mask = seg_lengths >= min_car_len;
        start_idx = start_idx(keep_mask);
        end_idx   = end_idx(keep_mask);
    end

    segment_times = [time(1)];
    car_segment_indices = [];
    current_seg_idx = 0;

    for k = 1:length(start_idx)
        c_start = time(start_idx(k));
        c_end = time(end_idx(k));

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

    fprintf('%s: Final segment count: %d, vehicle interval indices: %s\n', label, length(segment_times)-1, mat2str(car_segment_indices));

    info = struct(...
        'segment_times', segment_times, ...
        'car_segment_indices', car_segment_indices, ...
        'car_start_idx', start_idx, ...
        'car_end_idx', end_idx, ...
        'energy_curve', energy_curve, ...
        'var_curve', var_curve ...
    );
end

%% ====== Helper: DTW re-identification accuracy ======
function [reid_acc, out] = evaluate_reid_dtw(sig1, sig2, info1, info2)
    n1 = length(info1.car_start_idx);
    n2 = length(info2.car_start_idx);
    N_pairs = min(n1, n2);
    n_parts = 40;

    out = struct('N_pairs', N_pairs, 'N_matched', 0, 'dtw_distances', [], 'threshold', NaN, 'n_parts', n_parts, 'dtw_paths', {{}});
    reid_acc = 0;
    if N_pairs < 2
        reid_acc = NaN;
        return;
    end

    dists = nan(N_pairs, 1);
    paths = cell(N_pairs, 1);
    for k = 1:N_pairs
        seg1 = sig1(info1.car_start_idx(k):info1.car_end_idx(k));
        seg2 = sig2(info2.car_start_idx(k):info2.car_end_idx(k));
        seg1 = trim_car_interval_edge(seg1);
        seg2 = trim_car_interval_edge(seg2);

        seg1 = segment_signature(seg1, n_parts);
        seg2 = segment_signature(seg2, n_parts);

        [dists(k), p1, p2] = dtw_with_path(seg1, seg2);
        paths{k} = [p1(:), p2(:)];
    end

    % Adaptive threshold: median + 0.5*IQR
    med_d = median(dists, 'omitnan');
    q1 = prctile(dists, 25);
    q3 = prctile(dists, 75);
    thr = med_d + 0.5 * (q3 - q1);
    matched_mask = dists <= thr;

    N_matched = sum(matched_mask);
    reid_acc = (N_matched / N_pairs) * 100;

    out.N_pairs = N_pairs;
    out.N_matched = N_matched;
    out.dtw_distances = dists;
    out.threshold = thr;
    out.n_parts = n_parts;
    out.dtw_paths = paths;
end

function x = zscore_safe(x)
    mu = mean(x);
    sd = std(x);
    if sd < 1e-12
        x = x - mu;
    else
        x = (x - mu) / sd;
    end
end

function [dist, path_i, path_j] = dtw_with_path(a, b)
    n = length(a);
    m = length(b);
    D = inf(n+1, m+1);
    D(1,1) = 0;

    for i = 1:n
        for j = 1:m
            cost = abs(a(i) - b(j));
            D(i+1,j+1) = cost + min([D(i,j), D(i,j+1), D(i+1,j)]);
        end
    end

    % Normalize distances to reduce sequence-length dependence
    dist = D(n+1, m+1) / (n + m);

    % Backtrack the optimal path using one-based sequence indices
    i = n;
    j = m;
    path_i = i;
    path_j = j;
    while ~(i == 1 && j == 1)
        c_diag = inf; c_up = inf; c_left = inf;
        if i > 1 && j > 1, c_diag = D(i, j); end
        if i > 1, c_up = D(i, j+1); end
        if j > 1, c_left = D(i+1, j); end

        [~, step_id] = min([c_diag, c_up, c_left]);
        if step_id == 1
            i = i - 1;
            j = j - 1;
        elseif step_id == 2
            i = i - 1;
        else
            j = j - 1;
        end

        path_i = [i; path_i]; %#ok<AGROW>
        path_j = [j; path_j]; %#ok<AGROW>
    end
end

%% ====== Helper: plot DTW re-identification results ======
function plot_reid_dtw_pairs(sig1, sig2, info1, info2, dtw_info, sample_filename)
    N = dtw_info.N_pairs;
    if N <= 0
        return;
    end

    [~, k_best] = min(dtw_info.dtw_distances);
    n_parts = dtw_info.n_parts;

    seg1 = sig1(info1.car_start_idx(k_best):info1.car_end_idx(k_best));
    seg2 = sig2(info2.car_start_idx(k_best):info2.car_end_idx(k_best));
    seg1 = trim_car_interval_edge(seg1);
    seg2 = trim_car_interval_edge(seg2);
    s1 = segment_signature(seg1, n_parts);
    s2 = segment_signature(seg2, n_parts);

    path_ij = [];
    if isfield(dtw_info, 'dtw_paths') && numel(dtw_info.dtw_paths) >= k_best
        path_ij = dtw_info.dtw_paths{k_best};
    end
    if isempty(path_ij)
        [~, p1, p2] = dtw_with_path(s1, s2);
        path_ij = [p1(:), p2(:)];
    end

    figure('Name', 'DTW Alignment View', 'Position', [140, 120, 1100, 420]);
    tl = tiledlayout(1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    if nargin >= 6 && ~isempty(sample_filename)
        title(tl, sprintf('DTW Alignment (%s) | Pair=%d', sample_filename, k_best), 'Interpreter', 'none');
    else
        title(tl, sprintf('DTW Alignment | Pair=%d', k_best));
    end

    nexttile;
    hold on;
    pstep = max(1, floor(size(path_ij, 1) / 80));
    for p = 1:pstep:size(path_ij, 1)
        i = path_ij(p, 1);
        j = path_ij(p, 2);
        line([i, j], [s1(i), s2(j)], 'Color', [0.7 0.7 0.7], 'LineStyle', '-', 'LineWidth', 0.7);
    end
    scatter(1:n_parts, s1, 26, 'b', 'filled', 'DisplayName', 'Upstream');
    scatter(1:n_parts, s2, 26, 'r', 'o', 'DisplayName', 'Downstream');
    xlabel('Index');
    ylabel('Signal Value');
    legend('Location', 'best');
    grid off;
    apply_axis_style(gca);
end

%% ====== Subfunction: DTW alignment plot in reference-paper style ======
function plot_reid_dtw_reference(sig1, sig2, info1, info2, dtw_info, sample_filename)
    N = dtw_info.N_pairs;
    if N <= 0
        return;
    end

    n_parts = 30;
    % if isfield(dtw_info, 'n_parts') && ~isempty(dtw_info.n_parts)
    %     n_parts = dtw_info.n_parts;
    % end

    if isfield(dtw_info, 'dtw_distances') && numel(dtw_info.dtw_distances) >= N && any(isfinite(dtw_info.dtw_distances))
        valid_dists = dtw_info.dtw_distances;
        valid_dists(~isfinite(valid_dists)) = inf;
        [~, k_best] = min(valid_dists);
    else
        k_best = 1;
    end
    seg1 = sig1(info1.car_start_idx(k_best):info1.car_end_idx(k_best));
    seg2 = sig2(info2.car_start_idx(k_best):info2.car_end_idx(k_best));
    seg1 = trim_car_interval_edge(seg1);
    seg2 = trim_car_interval_edge(seg2);

    up_value = extraction_point_values(seg1, n_parts);
    down_value = extraction_point_values(seg2, n_parts);

    up_score = segment_signature(seg1, n_parts);
    down_score = segment_signature(seg2, n_parts);
    [~, p1, p2] = dtw_with_path(up_score, down_score);
    path_ij = [p1(:), p2(:)];

    path_ij = unique(path_ij, 'rows', 'stable');
    max_lines = min(n_parts, size(path_ij, 1));
    show_idx = unique(round(linspace(1, size(path_ij, 1), max_lines)), 'stable');
    path_show = path_ij(show_idx, :);
    n_show = size(path_show, 1);
    x = 1:n_show;
    up_plot = up_value(path_show(:, 1));
    down_plot = down_value(path_show(:, 2));
    up_plot = up_plot - up_plot(1);
    down_plot = down_plot - down_plot(1);

    figure('Name', 'DTW Reference Alignment', 'Position', [180, 120, 720, 520], 'Color', 'w');
    hold on;
    grid off;

    for k = 1:n_show
        plot([x(k), x(k)], [up_plot(k), down_plot(k)], 'r--', ...
            'LineWidth', 0.9, 'HandleVisibility', 'off');
    end

    plot(x, up_plot, 'ro', ...
        'MarkerSize', 7, ...
        'LineStyle', 'none', ...
        'LineWidth', 1.0, ...
        'DisplayName', 'upstream node A');
    plot(x, down_plot, 'b*', ...
        'MarkerSize', 8, ...
        'LineStyle', 'none', ...
        'LineWidth', 0.9, ...
        'DisplayName', 'downstream node B');

    xlabel('Sequence number of the extraction points', 'FontName', 'Times New Roman', 'FontWeight', 'bold');
    ylabel('Relative fused value of magnetic field', 'FontName', 'Times New Roman', 'FontWeight', 'bold');

    legend('Location', 'northwest', 'Box', 'off', 'FontName', 'Times New Roman', 'FontWeight', 'bold');
    x_max = ceil((n_show + 3) / 5) * 5;
    xlim([0, x_max]);
    xticks(0:5:x_max);

    y_all = [up_plot(:); down_plot(:)];
    if all(isfinite(y_all))
        y_min = floor((min(y_all) - 20) / 50) * 50;
        y_max = ceil((max(y_all) + 20) / 50) * 50;
        if y_min == y_max
            y_min = y_min - 50;
            y_max = y_max + 50;
        end
        ylim([y_min, y_max]);
        yticks(y_min:50:y_max);
    end

    ax = gca;
    set(ax, 'FontName', 'Times New Roman', ...
        'FontWeight', 'bold', ...
        'LineWidth', 1.0, ...
        'Box', 'off', ...
        'Color', 'w');

    if nargin >= 6 && ~isempty(sample_filename)
        set(gcf, 'Name', sprintf('DTW Reference Alignment - %s', sample_filename));
    end
end

%% ====== Helper: plot upstream/downstream scatter comparisons ======
function plot_reid_pair_scatter(sig1, sig2, info1, info2, dtw_info, sample_filename)
    N = dtw_info.N_pairs;
    if N <= 0
        return;
    end

    n_parts = 40;
    if isfield(dtw_info, 'n_parts') && ~isempty(dtw_info.n_parts)
        n_parts = dtw_info.n_parts;
    end

    [~, k_best] = min(dtw_info.dtw_distances);
    seg1 = sig1(info1.car_start_idx(k_best):info1.car_end_idx(k_best));
    seg2 = sig2(info2.car_start_idx(k_best):info2.car_end_idx(k_best));
    seg1 = trim_car_interval_edge(seg1);
    seg2 = trim_car_interval_edge(seg2);
    up_score = segment_signature(seg1, n_parts);
    down_score = segment_signature(seg2, n_parts);

    figure('Name', 'ReID Pair Scatter', 'Position', [220, 120, 980, 520]);
    hold on;
    grid off;

    for k = 1:n_parts
        plot([k, k], [up_score(k), down_score(k)], '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 0.8);
    end

    plot(1:n_parts, up_score, 'b*', 'MarkerSize', 6, 'DisplayName', 'Upstream');
    plot(1:n_parts, down_score, 'ro', 'MarkerSize', 6, 'DisplayName', 'Downstream');

    xlabel('Sample Index');
    ylabel('Vehicle Feature Score');
    xlim([1, n_parts]);
    if nargin >= 6 && ~isempty(sample_filename)
        title(sprintf('ReID Pair Scatter (%s) | Pair=%d | %d Segments | ACC=%.2f%%', sample_filename, k_best, n_parts, ...
            (dtw_info.N_matched / max(1, dtw_info.N_pairs)) * 100), 'Interpreter', 'none');
    else
        title(sprintf('ReID Pair Scatter | Pair=%d | %d Segments | ACC=%.2f%%', k_best, n_parts, ...
            (dtw_info.N_matched / max(1, dtw_info.N_pairs)) * 100));
    end
    legend('Location', 'best');
    apply_axis_style(gca);
end

function seg = trim_car_interval_edge(seg)
    seg = seg(:);
    n = length(seg);
    edge_len = floor(n * 0.30);
    if edge_len > 0 && 2 * edge_len < n
        seg = seg(edge_len + 1 : n - edge_len);
    end
end

function feat = extraction_point_values(seg, L)
    seg = seg(:);
    n = length(seg);
    feat = zeros(1, L);
    if n <= 0
        return;
    end

    edges = round(linspace(1, n + 1, L + 1));
    for i = 1:L
        s = edges(i);
        e = edges(i + 1) - 1;
        if s > n
            feat(i) = feat(max(i - 1, 1));
            continue;
        end
        e = min(e, n);
        if e < s
            e = s;
        end
        feat(i) = mean(seg(s:e));
    end
end

function feat = segment_signature(seg, L)
    seg = zscore_safe(seg(:));
    n = length(seg);
    feat = zeros(1, L);
    if n <= 0
        return;
    end

    edges = round(linspace(1, n+1, L+1));
    for i = 1:L
        s = edges(i);
        e = edges(i+1)-1;
        if s > n
            feat(i) = feat(max(i-1,1));
            continue;
        end
        e = min(e, n);
        if e < s
            e = s;
        end
        w = seg(s:e);
        feat(i) = mean(abs(w));
    end
end

%% ====== Helper: plot vehicle intervals ======
function plot_segments(signal, segment_times, car_segment_indices)
    y_min = min(signal);
    y_max = max(signal);

    for i = 1:length(segment_times)
        xline(segment_times(i), 'b--', 'Alpha', 0.5, 'LineWidth', 1.0);
    end

    for i = 1:length(car_segment_indices)
        seg_idx = car_segment_indices(i);
        t_start = segment_times(seg_idx);
        t_end   = segment_times(seg_idx+1);
        patch([t_start t_end t_end t_start], [y_min y_min y_max y_max], 'r', ...
            'FaceAlpha', 0.2, 'EdgeColor', 'none');
    end
end

%% ====== Helper: apply consistent axis styling ======
function apply_axis_style(ax)
    set(ax, 'FontName', 'Times New Roman', ...
        'FontWeight', 'bold', ...
        'LineWidth', 1.2, ...
        'Box', 'on');
end
