# Vehicle Speed Estimation Based on Adaptive Multi-Layer Decomposition Denoising of Magnetic Signals

A MATLAB project for signal analysis and denoising using sliding-window entropy, Variational Mode Decomposition (VMD), correlation analysis, signal reconstruction, similarity evaluation, and adaptive redecomposition.

## File Structure

IMFs (Intrinsic Mode Functions) are the signal components produced by decomposition.

| File | Description |
|---|---|
| `main.m` | Runs batch processing on TXT files and applies Parts 1–6 to sensors. |
| `part1_sliding_entropy_analysis.m` | Identify vehicle segments and ultimately output segment boundaries for use in subsequent analysis. |
| `part2_vmd_decomposition.m` | Decomposes the input signal using VMD by default, sorts modes from low to high frequency, and saves decomposition results. |
| `part3_correlation_analysis.m` | Uses global correlation and vehicle–background correlation differences to classify modes as signal, noise, or mixed components, returning the IMF indices used for reconstruction and refinement. |
| `part4_signal_reconstruction.m` | Reconstructs the signal by summing selected signal and mixed IMFs. |
| `part5_similarity_evaluation.m` | Compares the original reconstruction with linear calibration and several offset corrections, selecting the variant with the highest estimated SNR. |
| `part6_adaptive_redecomposition.m` | Refines mixed IMFs through repeated VMD decomposition, submode selection, and reconstruction for up to five iterations. |
| `part7_denoised_window_features.m` | Detects vehicle intervals in two denoised sensor signals using window energy and variance. Compares sequentially paired vehicle segments using Dynamic Time Warping (DTW), reports an adaptive-threshold matching percentage as ReID-ACC, and estimates speed from the first detected arrival-time difference using an 9 m sensor spacing. |

### Current Entry-Point Behavior

- The active batch workflow runs Parts 1–6. The two-sensor workflow that calls Part 7 is currently commented out.
- Input TXT files must contain at least four columns. The batch workflow reads time from column 1 and the first sensor signal from column 3.
- Data and clean-reference paths are currently hard-coded and should be updated before running.
- Part 5 uses clean references for evaluation and calibration. Part 6 uses blind SNR for refinement and does not recompute reference-based correlation after each update.

*The final filename is truncated in the screenshot. Module descriptions are based on filenames.*

## Environment

- **MATLAB** — use a version compatible with the project scripts.
- Add all project scripts and required helper functions to the MATLAB path.
- Ensure that the VMD implementation and any toolboxes called by the scripts are available.

*The required MATLAB version and toolbox list should be confirmed from the source code.*

## Dataset

The dataset is organized by noise type: `cable`, `gaussian`, `impulse`, and `subway`. Each category contains sample folders with the following structure:

```text
datasets/
├── cable/
│   ├── sample1/
│   │   ├── label.txt
│   │   ├── sensor1.txt
│   │   └── sensor2.txt
│   ├── sample2/
│   ├── sample3/
│   ├── sample4/
│   └── sample5/
├── gaussian/
│   └── ...
├── impulse/
│   └── ...
└── subway/
    └── ...
```

Each sample folder contains:

- `label.txt`: sample speed information.
- `sensor1.txt`: signal data from sensor 1.
- `sensor2.txt`: signal data from sensor 2.

All noise categories follow the same sample folder structure. Update the dataset path in the MATLAB scripts before running.

## Usage

1. Open the project directory in MATLAB.
2. Add the required scripts and helper functions to the MATLAB path.
3. Prepare the dataset and update the data paths and processing parameters in the code.
4. Run:

```matlab
main
```

Processing results and output locations are defined in the scripts.
