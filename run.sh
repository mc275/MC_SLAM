#!/bin/zsh
./Examples/Monocular/mono_EuRoC_vins Vocabulary/ORBvoc.bin config/euroc.yaml 
python evaluate/evaluate_plot.py evaluate/MH_02_easy.txt tmp/CameraFrameTrajectory.txt MH_02_easy

