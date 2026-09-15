# AMLD Dataset

This repository provides the datasets used for evaluating the proposed AMLD-based vehicle speed estimation method.

AMLD is designed for robust vehicle speed estimation using magnetic sensor networks under different electromagnetic interference conditions. The dataset contains paired magnetic signals collected from upstream and downstream sensor nodes together with the corresponding ground-truth vehicle speed.

## Dataset Structure

```text
dataset/
├── cable/
│   ├── sample1/
│   │   ├── sensor1.txt
│   │   ├── sensor2.txt
│   │   └── label.txt
│   ├── sample2/
│   └── ...
├── gaussian/
│   ├── sample1/
│   │   ├── sensor1.txt
│   │   ├── sensor2.txt
│   │   └── label.txt
│   └── ...
├── impulse/
│   ├── sample1/
│   │   ├── sensor1.txt
│   │   ├── sensor2.txt
│   │   └── label.txt
│   └── ...
└── subway/
    ├── sample1/
    │   ├── sensor1.txt
    │   ├── sensor2.txt
    │   └── label.txt
    └── ...
````

The dataset is divided into four interference conditions:

* **cable/**: Magnetic vehicle signals affected by high-voltage cable interference.
* **gaussian/**: Magnetic vehicle signals under Gaussian noise.
* **impulse/**: Magnetic vehicle signals under impulse noise.
* **subway/**: Magnetic vehicle signals affected by subway-related electromagnetic interference.

Each sample folder contains magnetic signals recorded by two spatially separated sensor nodes and the corresponding ground-truth vehicle speed.

## Dataset Description

A total of **1,730 sample folders** are provided under four different interference conditions.

### 1. High-Voltage Cable Interference Dataset

The `cable/` dataset contains **519 samples**. The magnetic signals are affected by electromagnetic interference associated with high-voltage cables.

Each sample contains paired magnetic signals recorded by the upstream and downstream sensor nodes and the corresponding ground-truth vehicle speed.

### 2. Gaussian Noise Dataset

The `gaussian/` dataset contains **519 samples**. Gaussian noise is introduced into the magnetic signals to evaluate the robustness of the denoising and vehicle speed estimation methods under random noise conditions.

### 3. Impulse Noise Dataset

The `impulse/` dataset contains **519 samples**. Impulse noise is introduced into the magnetic signals to evaluate the robustness of the proposed method against short-duration and high-amplitude disturbances.

### 4. Subway Interference Dataset

The `subway/` dataset contains **173 samples**. These signals contain electromagnetic interference associated with subway operation and are used to evaluate the generalization capability of the proposed method under a different real-world interference source.

All datasets are provided in **TXT format**.

## Data Format

Each sample contains three files:

```text
sampleX/
├── sensor1.txt
├── sensor2.txt
└── label.txt
```

### Sensor Signal Format

Both `sensor1.txt` and `sensor2.txt` contain two columns:

| Timestamp     | Magnetic Field |
| ------------- | -------------- |
| 1610678477762 | -1.200160      |
| 1610678477856 | -0.335838      |
| 1610678477950 | 2.351580       |
| 1610678478041 | -1.013876      |
| 1610678478134 | -0.878567      |

The two columns are defined as follows:

| Column         | Description                               |
| -------------- | ----------------------------------------- |
| Timestamp      | Sampling timestamp of the magnetic signal |
| Magnetic Field | Measured magnetic field intensity         |

`sensor1.txt` records the magnetic signal collected by the upstream sensor node, whereas `sensor2.txt` records the corresponding magnetic signal collected by the downstream sensor node.

The paired signals describe the magnetic response generated when the same vehicle successively passes the two sensor nodes.

### Speed Label Format

The `label.txt` file contains the ground-truth vehicle speed corresponding to the paired magnetic signals.

For example:

```text
24.19
```

The value represents the ground-truth speed of the vehicle corresponding to `sensor1.txt` and `sensor2.txt`.

## Vehicle Speed Estimation Principle

For each sample, the upstream and downstream magnetic signals correspond to the same vehicle passing two spatially separated magnetic sensor nodes.

After signal denoising and cross-node vehicle re-identification, the vehicle speed is estimated based on the known distance between the two sensor nodes and the time difference between the matched vehicle passages.

The paired-signal structure of the dataset supports the complete AMLD processing pipeline:

**Signal Denoising → Vehicle Re-identification → Speed Estimation**

## Ground Truth Annotation

The ground-truth speed labels were obtained during the vehicle experiments and are stored individually in the `label.txt` file of each sample.

The upstream and downstream magnetic signals are organized in paired sample folders to preserve their correspondence with the same vehicle passage.

## Citation

If you use this dataset in your research, please cite our paper.

```bibtex
@article{AMLD,
  title={...},
  author={...},
  journal={...},
  year={...}
}
```

## License

This dataset is released for academic and research purposes only.


