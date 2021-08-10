# Changelog

All notable changes to this repository will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/) and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## Unreleased

Fixed the ROS-Unity Integration tutorial `robo_demo.launch` to be up-to-date with file paths.

### Upgrade Notes

### Known Issues

### Added

### Changed

Changed the Pick and Place Demo's topic from SourceDestination to SourceDestination_input

### Deprecated

### Removed

### Fixed

Fixed the ROS-Unity Integration tutorial to use the correct link to install ROS-TCP-Connector package

Fixed the Pick and Place Tutorial to use ArticulationBody jointPositions, rather than the xDrive.target, for updating the current joint angle positions

Fixed network.md in ROS-Unity Integration tutorial by removing the obsolete UNITY_IP
