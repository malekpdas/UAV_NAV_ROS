# calibration

This package contains tools for calibrating UAV sensors.

## Nodes

### [mag_calibration](descriptor/mag_calibration.yaml)

A robust nonlinear magnetometer calibration tool. It collects data samples and calculates the hard-iron bias and soft-iron matrix to map readings onto a unit sphere.

## Configuration

- `config/mag_calibration.yaml`: Sampling parameters and target topic.

## Usage

Run the calibration node while rotating the vehicle in all orientations to achieve complete spherical coverage.
