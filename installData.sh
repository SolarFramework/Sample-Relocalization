# Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
mkdir -p data/fbow_voc
unzip -o fbow_voc.zip -d ./data/fbow_voc
rm fbow_voc.zip

# Download TUM camera calibration
curl https://repository.solarframework.org/generic/captures/singleRGB/TUM/tum_camera_calibration.json -L -o data/tum_camera_calibration.json

# Download TUM video for testing relocalization
curl https://repository.solarframework.org/generic/captures/singleRGB/TUM/rgbd_dataset_freiburg3_long_office_household_relocalization.avi -L -o data/rgbd_dataset_freiburg3_long_office_household_relocalization.avi

# Download TUM map for testing relocalization
curl https://repository.solarframework.org/generic/maps/TUM/freiburg3_long_office_household/map_linux_0_10_0.zip -L -o map.zip
unzip -o map.zip -d ./data
rm map.zip

# Download AR device captures
curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
unzip -o loopDesktopA.zip -d ./data/data_hololens
rm loopDesktopA.zip

curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopB.zip -L -o loopDesktopB.zip
unzip -o loopDesktopB.zip -d ./data/data_hololens
rm loopDesktopB.zip

curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/hololens_vlclf_marker.zip -L -o hololens_vlclf_marker.zip
unzip -o hololens_vlclf_marker.zip -d ./data/data_hololens/hololens_vlclf_marker
rm hololens_vlclf_marker.zip

# Download calibration file
echo Download calibration file
curl https://repository.solarframework.org/generic/captures/hololens/hololens_calibration.json -L -o ./data/data_hololens/hololens_calibration.json
