%% ====== Batch processing ======
clc;
warning('off', 'all');

data_folder = 'E:\Postdata\yan2\datasets\speed_detection\cable_noisy\new\';
files_list = dir(fullfile(data_folder, '*.txt'));
num_files = numel(files_list);

if num_files == 0
    error('No TXT files found in folder: %s', data_folder);
end

% Result containers
FileNames = cell(num_files, 1);
Init_Sim  = nan(num_files, 1);
Init_SNR  = nan(num_files, 1);
Final_Sim = nan(num_files, 1);
Final_SNR = nan(num_files, 1);
Final_NRR = nan(num_files, 1);
Status    = repmat({'Skipped'}, num_files, 1);
History   = cell(num_files, 1);

% Decomposition parameters
optimal_K = 15; % Number of modes
alpha = 2000; % Bandwidth penalty
tau = 0; % Noise tolerance
DC = 0; % Fix the first mode at DC
init = 1; % Frequency initialization
tol = 1e-7; % Convergence tolerance
enable_adaptive_redecomp = 1; % 1=adaptive redecomposition, 0=use the current reconstruction

for file_idx = 1:num_files
    filename = files_list(file_idx).name;
    filepath = fullfile(data_folder, filename);
    FileNames{file_idx} = filename;

    fprintf('\n------------------------------------------------------\n');
    fprintf('Processing file (%d/%d): %s\n', file_idx, num_files, filename);

    data = readmatrix(filepath);
    if isempty(data) || size(data, 2) < 4
        data = readmatrix(filepath, 'Delimiter', ',');
    end
    if isempty(data) || size(data, 2) < 4
        warning('Skipping file (fewer than four columns): %s', filename);
        continue;
    end

    time = data(:, 1);
    signal = data(:, 3); % Process channel 1 only

    valid_mask = isfinite(time) & isfinite(signal);
    time = time(valid_mask);
    signal = signal(valid_mask);

    if numel(time) < 20
        warning('Skipping file (insufficient valid samples): %s', filename);
        continue;
    end

    fprintf('=== Starting signal processing ===\n');

    %% ====== Part 1: sliding entropy ======
    [time, signal, segment_times, ~, ~, car_segment_indices] = ...
        part1_sliding_entropy_analysis(time, signal); %#ok<ASGLU>

    %% ====== Part 2: decomposition ======
    [IMFS_EMD, ~] = part2_vmd_decomposition(signal, alpha, tau, optimal_K, DC, init, tol);

    %% ====== Part 3: correlation analysis and IMF selection ======
    [signal_imfs, noise_imfs, mixed_imfs, ~] = part3_correlation_analysis(...
        time, signal, IMFS_EMD, segment_times); %#ok<ASGLU>


    %% ====== Part 4: signal reconstruction ======
    [reconstructed_signal, ~, used_best_imfs, blind_snr] = part4_signal_reconstruction(...
        time, signal, IMFS_EMD, signal_imfs, mixed_imfs, filepath, 1, segment_times, car_segment_indices); %#ok<ASGLU>

    mixed_imfs = intersect(mixed_imfs, used_best_imfs);

    %% ====== Part 5: similarity evaluation ======
    similarity = part5_similarity_evaluation(filepath, reconstructed_signal, IMFS_EMD, noise_imfs, 1);

    %% ====== Part 6: adaptive IMF redecomposition ======
    if enable_adaptive_redecomp == 1
        [IMFS_EMD, signal_imfs, noise_imfs, mixed_imfs, reconstructed_signal, similarity, current_sim, current_snr, history_str, opt_status] = ...
            part6_adaptive_redecomposition(time, signal, segment_times, IMFS_EMD, signal_imfs, noise_imfs, mixed_imfs, ...
            reconstructed_signal, similarity, filepath, 1, alpha, tau, DC, init, tol, false, car_segment_indices); %#ok<ASGLU>
    else
        current_sim = similarity.Best.Result.Corr;
        current_snr = blind_snr;
        history_str = 'No-Redep';
        opt_status = 0;
    end

    Init_Sim(file_idx) = similarity.Best.Result.Corr;
    Init_SNR(file_idx) = similarity.Best.Result.SNR;
    Final_Sim(file_idx) = current_sim;
    Final_SNR(file_idx) = current_snr;

    original_var = var(signal(isfinite(signal)), 1);
    denoised_var = var(reconstructed_signal(isfinite(reconstructed_signal)), 1);
    if original_var > 0 && denoised_var > 0
        Final_NRR(file_idx) = 10 * (log10(original_var) - log10(denoised_var));
    else
        Final_NRR(file_idx) = NaN;
    end

    History{file_idx} = history_str;

    if opt_status == 1
        status_str = 'Green';
    elseif opt_status == -1
        status_str = 'Red';
    else
        status_str = 'No-Trigger';
    end
    Status{file_idx} = status_str;
    fprintf('Completed: Sim=%.4f, SNR=%.4f, NRR=%.4f dB, Status=%s\n', ...
        current_sim, current_snr, Final_NRR(file_idx), status_str);
end

Init_Sim(~isfinite(Init_Sim)) = NaN;
Init_SNR(~isfinite(Init_SNR)) = NaN;
Final_Sim(~isfinite(Final_Sim)) = NaN;
Final_SNR(~isfinite(Final_SNR)) = NaN;
Final_NRR(~isfinite(Final_NRR)) = NaN;

ResultsTable = table(FileNames, Init_Sim, Init_SNR, Final_Sim, Final_SNR, Final_NRR, Status, History, ...
    'VariableNames', {'FileName', 'InitialSimilarity', 'InitialSNR', 'FinalSimilarity', 'FinalSNR', 'FinalNRR', 'OptimizationStatus', 'OptimizationHistory'});

out_xlsx = fullfile(data_folder, 'VMD_Results.xlsx');
writetable(ResultsTable, out_xlsx);

fprintf('\n=== Batch processing completed; results saved to: %s ===\n', out_xlsx);


