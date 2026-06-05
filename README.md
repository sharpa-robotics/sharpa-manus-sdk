# Manus SDK Integration for SharpaWave

This repository is used for integration and demonstration of SharpaWave with Manus MetaGloves Pro, including a Manus data acquisition client and hand retargeting examples.

## 🚀 Overview

The project is organized into two main components:

- `client/`: Linux client based on Manus SDK (C++), responsible for glove data acquisition and publishing.
- `retargeting/`: Retargeting and visualization modules (Python), responsible for mapping keypoints to SharpaWave joint space.

## 💼 Repository Layout

```text
manus-sdk/
├── client/
│   ├── ManusSDK/
│   ├── Makefile
│   └── *.cpp / *.cc
├── retargeting/
│   ├── retargeting_manus_demo_multiprocess.py
│   ├── environment.yml
│   └── requirements.txt
├── SharpaWave with Manus User Manual.pdf
└── README.md
```

## 📖 Documentation

For complete usage instructions, installation steps, network configuration, calibration workflow, developer interfaces, and troubleshooting, refer to:

- `SharpaWave with Manus User Manual.pdf`

Recommended reading order:

1. Setup Guide
2. Calibration
3. Developer Guide

## ▶️ Quick Entry

For quick navigation, start with:

- Client build entry: `client/Makefile`
- Retargeting demo entry: `retargeting/retargeting_manus_demo_multiprocess.py`
- Python environment: `retargeting/environment.yml`

## 📌 License

Repository license terms are provided in the `License` file.  
Manus SDK components are subject to their official license terms and must be used accordingly.
