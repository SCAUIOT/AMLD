function [IMFS_EMD, u_opt] = part2_vmd_decomposition( ...
    signal, alpha, tau, optimal_K, DC, init, tol)

% Decomposition method id:
%   1 = VMD
%   2 = EMD
%   3 = EEMD
%   4 = CEEMD
%   5 = CEEMDAN
method_id = 1;

if method_id == 1
    fprintf('=== Part 2: VMD decomposition ===\n');

    [u_opt, ~, omega] = VMD(signal, alpha, tau, optimal_K, DC, init, tol);
    u_opt = normalize_imf_orientation(u_opt, numel(signal));
    [u_opt, mode_order, mode_freqs] = sort_imfs_by_dominant_frequency(u_opt, omega, 'ascend');
    fprintf('VMD modes reordered low-to-high frequency. Original order: %s\n', mat2str(mode_order));
    fprintf('Sorted normalized center/dominant frequencies: %s\n', mat2str(mode_freqs, 4));
    IMFS_EMD = u_opt;

    % num_imfs = size(IMFS_EMD, 1);
    % if num_imfs > 0
    %     n_cols = ceil(sqrt(num_imfs));
    %     n_rows = ceil(num_imfs / n_cols);
    %     figure('Color', 'w', 'Name', 'Sorted IMFs');
    %     for k = 1:num_imfs
    %         subplot(n_rows, n_cols, k);
    %         plot(IMFS_EMD(k, :), 'k');
    %         title(sprintf('IMF %d', k), 'FontWeight', 'bold');
    %         set(gca, 'FontName', 'Times New Roman', 'FontSize', 10, 'FontWeight', 'bold');
    %     end
    % end

elseif method_id == 2
    fprintf('=== Part 2: EMD decomposition ===\n');

    imfs = emd(signal(:));
    imfs = normalize_imf_orientation(imfs, numel(signal));

    IMFS_EMD = imfs;
    u_opt = imfs;

elseif method_id == 3
    fprintf('=== Part 2: EEMD decomposition ===\n');

    ens_num = 30;
    noise_ratio = 0.2;
    imfs = eemd_local(signal, ens_num, noise_ratio);
    imfs = normalize_imf_orientation(imfs, numel(signal));

    IMFS_EMD = imfs;
    u_opt = imfs;

elseif method_id == 4
    fprintf('=== Part 2: CEEMD decomposition ===\n');

    ens_num = 30;
    noise_ratio = 0.2;
    imfs = ceemd_local(signal, ens_num, noise_ratio);
    imfs = normalize_imf_orientation(imfs, numel(signal));

    IMFS_EMD = imfs;
    u_opt = imfs;

elseif method_id == 5
    fprintf('=== Part 2: CEEMDAN decomposition ===\n');

    ens_num = 30;
    noise_ratio = 0.2;
    imfs = ceemdan_local(signal, ens_num, noise_ratio);
    imfs = normalize_imf_orientation(imfs, numel(signal));

    IMFS_EMD = imfs;
    u_opt = imfs;

else
    error('Unknown decomposition method_id: %d. Use 1=VMD, 2=EMD, 3=EEMD, 4=CEEMD, 5=CEEMDAN.', method_id);
end

save('optimal_K.mat', 'optimal_K');
save('imfs_result.mat', 'IMFS_EMD');

end

function imfs_avg = eemd_local(signal, ens_num, noise_ratio)
    signal = signal(:);
    N = numel(signal);
    sig_std = std(signal);
    if sig_std < 1e-12
        sig_std = 1;
    end

    imfs_sum = [];

    for k = 1:ens_num
        noisy_signal = signal + noise_ratio * sig_std * randn(N, 1);
        imfs_k = emd(noisy_signal);
        imfs_k = normalize_imf_orientation(imfs_k, N);
        imfs_sum = add_imf_set(imfs_sum, imfs_k);
    end

    imfs_avg = imfs_sum / ens_num;
end

function imfs_avg = ceemd_local(signal, ens_num, noise_ratio)
% CEEMD uses complementary noise pairs. For each random noise sequence,
% decompose signal + noise and signal - noise, then average the paired IMFs.

    signal = signal(:);
    N = numel(signal);
    sig_std = std(signal);
    if sig_std < 1e-12
        sig_std = 1;
    end

    imfs_sum = [];

    for k = 1:ens_num
        noise = noise_ratio * sig_std * randn(N, 1);

        imfs_plus = emd(signal + noise);
        imfs_minus = emd(signal - noise);

        imfs_plus = normalize_imf_orientation(imfs_plus, N);
        imfs_minus = normalize_imf_orientation(imfs_minus, N);

        pair_imfs = average_imf_pair(imfs_plus, imfs_minus);
        imfs_sum = add_imf_set(imfs_sum, pair_imfs);
    end

    imfs_avg = imfs_sum / ens_num;
end

function imfs_ceemdan = ceemdan_local(signal, ens_num, noise_ratio)
% CEEMDAN extracts each IMF with adaptive noise modes. At each stage, only
% the first IMF/local mean of the noisy residual is used, which reduces mode
% mixing while keeping the decomposition complete.

    signal = signal(:);
    N = numel(signal);
    sig_std = std(signal);
    if sig_std < 1e-12
        sig_std = 1;
    end

    noise_modes = cell(ens_num, 1);
    for k = 1:ens_num
        noise = randn(N, 1);
        noise_std = std(noise);
        if noise_std < 1e-12
            noise_std = 1;
        end
        noise = noise / noise_std;
        noise_modes{k} = normalize_imf_orientation(emd(noise), N);
    end

    beta0 = noise_ratio * sig_std;
    first_imf_sum = zeros(N, 1);
    for k = 1:ens_num
        noisy_signal = signal + beta0 * get_noise_mode(noise_modes{k}, 1, N);
        first_imf_sum = first_imf_sum + extract_first_imf(noisy_signal, N);
    end

    first_imf = first_imf_sum / ens_num;
    imfs_ceemdan = first_imf.';
    residue = signal - first_imf;

    mode_idx = 2;
    while count_extrema(residue) >= 2
        residue_std = std(residue);
        if residue_std < 1e-12
            break;
        end

        beta = noise_ratio * residue_std;
        local_mean_sum = zeros(N, 1);

        for k = 1:ens_num
            noisy_residue = residue + beta * get_noise_mode(noise_modes{k}, mode_idx, N);
            imf1 = extract_first_imf(noisy_residue, N);
            local_mean_sum = local_mean_sum + (noisy_residue - imf1);
        end

        next_residue = local_mean_sum / ens_num;
        curr_imf = residue - next_residue;

        if safe_mean_square(curr_imf) < 1e-24
            break;
        end

        imfs_ceemdan(end+1, :) = curr_imf.'; %#ok<AGROW>
        residue = next_residue;
        mode_idx = mode_idx + 1;
    end

    % CEEMDAN decomposition must include the final residue so that the
    % selected modes can reconstruct low-frequency signal content.
    if safe_mean_square(residue) >= 1e-24
        imfs_ceemdan(end+1, :) = residue.'; %#ok<AGROW>
    end
end

function imfs = normalize_imf_orientation(imfs, N)
% Keep IMF matrices in K x N format.

    if isempty(imfs)
        imfs = zeros(0, N);
        return;
    end

    if size(imfs, 2) ~= N
        imfs = imfs.';
    end
end

function imfs_sum = add_imf_set(imfs_sum, imfs_k)
    if isempty(imfs_sum)
        imfs_sum = imfs_k;
        return;
    end

    Kmin = min(size(imfs_sum, 1), size(imfs_k, 1));
    Nmin = min(size(imfs_sum, 2), size(imfs_k, 2));
    imfs_sum = imfs_sum(1:Kmin, 1:Nmin) + imfs_k(1:Kmin, 1:Nmin);
end

function pair_imfs = average_imf_pair(imfs_plus, imfs_minus)
    Kmin = min(size(imfs_plus, 1), size(imfs_minus, 1));
    Nmin = min(size(imfs_plus, 2), size(imfs_minus, 2));
    pair_imfs = 0.5 * (imfs_plus(1:Kmin, 1:Nmin) + imfs_minus(1:Kmin, 1:Nmin));
end

function noise_mode = get_noise_mode(noise_imfs, mode_idx, N)
    if isempty(noise_imfs) || size(noise_imfs, 1) < mode_idx
        noise_mode = zeros(N, 1);
    else
        noise_mode = noise_imfs(mode_idx, 1:min(N, size(noise_imfs, 2))).';
        if numel(noise_mode) < N
            noise_mode(end+1:N, 1) = 0;
        end
    end
end

function imf1 = extract_first_imf(x, N)
    imfs = emd(x(:));
    imfs = normalize_imf_orientation(imfs, N);
    if isempty(imfs)
        imf1 = zeros(N, 1);
    else
        imf1 = imfs(1, 1:min(N, size(imfs, 2))).';
        if numel(imf1) < N
            imf1(end+1:N, 1) = 0;
        end
    end
end

function n_extrema = count_extrema(x)
    x = x(:);
    x = x(isfinite(x));
    if numel(x) < 3
        n_extrema = 0;
        return;
    end

    dx = diff(x);
    dx(abs(dx) < 1e-12) = [];
    if numel(dx) < 2
        n_extrema = 0;
    else
        n_extrema = sum(diff(sign(dx)) ~= 0);
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
