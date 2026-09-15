function similarity = part5_similarity_evaluation(filepath, reconstructed_signal, IMFS_EMD, noise_imfs, sig_idx)
% Compare reconstruction with the clean reference using amplitude and endpoint alignment.
    %
    %

    %% ====== 1. Clean signal folder ======
    clean_folder = 'E:\Postdata\yan2\datasets\speed_detection\clean\new\split\';

    %% ====== 2. Resolve the clean signal path from the noisy filename ======
    [~, noisy_name, ext] = fileparts(filepath);
    tokens = regexp(noisy_name, '^(.*)\+.*(_[xyz])$', 'tokens');
    if isempty(tokens)
        clean_name = [regexprep(noisy_name, '\+.*$', ''), ext];
    else
        clean_name = [tokens{1}{1}, tokens{1}{2}, ext];
    end
    clean_file = fullfile(clean_folder, clean_name);

    if ~isfile(clean_file)
        error('Clean signal file not found: %s', clean_file);
    end
    fprintf('\nMatching clean signal file: %s\n', clean_file);

    %% ====== 3. Load the clean signal ======
    data_clean = load(clean_file);

    if nargin < 5
        sig_idx = 1;
    end

    if sig_idx == 1
        t_clean = data_clean(:, 1); % time1
        clean_col = 3; % value1
    else
        t_clean = data_clean(:, 2); % time2
        clean_col = 4; % value2
    end

    if size(data_clean, 2) >= clean_col
        clean_signal = data_clean(:, clean_col);
    else
        clean_signal = data_clean(:, end);
    end

    clean_signal = clean_signal(:);
    reconstructed_signal = reconstructed_signal(:);

    %% ====== 4. Align signal lengths ======
    N = min(length(reconstructed_signal), length(clean_signal));
    reconstructed_signal = reconstructed_signal(1:N);
    clean_signal = clean_signal(1:N);
    t_clean = t_clean(1:N);

    %% ====== 5. Baseline metrics ======
    corr_pre = corrcoef(reconstructed_signal, clean_signal); corr_pre = corr_pre(1,2);
    mse_pre = mean((reconstructed_signal - clean_signal).^2);
    snr_pre = 10*log10(mean(clean_signal.^2) / mean((reconstructed_signal - clean_signal).^2));
    % fprintf('Original reconstructed signal: Corr=%.4f,  SNR=%.2f dB\n', corr_pre,  snr_pre);

    %% ====== 6. Calibrate amplitude using linear regression ======
    recon = reconstructed_signal - mean(reconstructed_signal);
    clean_zero = clean_signal - mean(clean_signal);
    X = [recon, ones(N,1)];
    params = X \ clean_zero; % alpha, beta
    alpha = params(1); beta  = params(2);
    reconstructed_adj = alpha * recon + beta;

    corr_post = corrcoef(reconstructed_adj, clean_signal); corr_post = corr_post(1,2);
    mse_post = mean((reconstructed_adj - clean_signal).^2);
    snr_post = 10*log10(mean(clean_signal.^2) / mean((reconstructed_adj - clean_signal).^2));
    % fprintf('After linear calibration: alpha=%.4f, beta=%.4f, Corr=%.4f, MSE=%.6f, SNR=%.2f dB\n', ...
    %     alpha, beta, corr_post, mse_post, snr_post);

    %% ====== 7. Align the starting point ======
    offset = reconstructed_signal(1) - clean_signal(1);
    reconstructed_aligned = reconstructed_signal - offset;

    corr_aligned = corrcoef(reconstructed_aligned, clean_signal); corr_aligned = corr_aligned(1,2);
    mse_aligned = mean((reconstructed_aligned - clean_signal).^2);
    snr_aligned = 10*log10(mean(clean_signal.^2) / mean((reconstructed_aligned - clean_signal).^2));
    % fprintf('After start alignment: Corr=%.4f,  SNR=%.2f dB\n', ...
    %     corr_aligned,  snr_aligned);

    %% ====== 7. Align both endpoints ======
    start_offset = reconstructed_signal(1) - clean_signal(1);
    end_offset   = reconstructed_signal(end) - clean_signal(end);
    offset = (start_offset + end_offset)/2;
    reconstructed_aligned1 = reconstructed_signal - offset;

    corr_aligned = corrcoef(reconstructed_aligned1, clean_signal); corr_aligned = corr_aligned(1,2);
    mse_aligned2 = mean((reconstructed_aligned1 - clean_signal).^2);
    snr_aligned2 = 10*log10(mean(clean_signal.^2) / mean((reconstructed_aligned1 - clean_signal).^2));
    % fprintf('After endpoint alignment: Corr=%.4f,  SNR=%.2f dB\n', ...
    %     corr_aligned,  snr_aligned2);

    end_offset = reconstructed_signal(end) - clean_signal(end);

    reconstructed_aligned2 = reconstructed_signal - end_offset;

    corr_aligned = corrcoef(reconstructed_aligned2, clean_signal);
    corr_aligned = corr_aligned(1,2);
    mse_aligned3 = mean((reconstructed_aligned2 - clean_signal).^2);
    snr_aligned3 = 10*log10(mean(clean_signal.^2) / mean((reconstructed_aligned2 - clean_signal).^2));

    % fprintf('After end alignment: Corr=%.4f,  SNR=%.2f dB\n', ...
    %     corr_aligned,  snr_aligned3);

    % figure;
    % plot(t_clean, clean_signal, 'b-', 'LineWidth', 1.5); hold on;
    % xlabel('Time'); ylabel('Amplitude');
    % title('Signal comparison');
    % grid on;

    %% ====== 9. Return the results structure ======
    similarity = struct(...
        'Original', struct('Corr', corr_pre, 'MSE', mse_pre, 'SNR', snr_pre, 'Signal', reconstructed_signal), ...
        'LinearCalibrated', struct('Corr', corr_post, 'MSE', mse_post, 'SNR', snr_post, 'Signal', reconstructed_adj), ...
        'StartAligned', struct('Corr', corr_aligned, 'MSE', mse_aligned, 'SNR', snr_aligned, 'Signal', reconstructed_aligned), ...
        'StartEndAligned',struct('Corr', corr_aligned, 'MSE', mse_aligned2, 'SNR', snr_aligned2, 'Signal', reconstructed_aligned),...
        'EndAligned', struct('Corr', corr_aligned, 'MSE', mse_aligned3, 'SNR', snr_aligned3, 'Signal', reconstructed_aligned)...
    );
%% ====== 10. Select the highest SNR ======
    snr_list = [similarity.Original.SNR, ...
                similarity.LinearCalibrated.SNR, ...
                similarity.StartAligned.SNR, ...
                similarity.StartEndAligned.SNR, ...
                similarity.EndAligned.SNR];

    [best_snr, best_idx] = max(snr_list);

    methods = {'Original','LinearCalibrated','StartAligned','StartEndAligned','EndAligned'};
    best_method = methods{best_idx};

    similarity.Best.Method = best_method;
    similarity.Best.SNR    = best_snr;
    similarity.Best.Result = similarity.(best_method);

    fprintf('>>> Best method: %s, SNR = %.2f dB\n', best_method, best_snr);
end

