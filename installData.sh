# Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
mkdir -p data/fbow_voc
unzip -o fbow_voc.zip -d ./data/fbow_voc
rm fbow_voc.zip

# Download TUM camera calibration
curl https://artifact.b-com.com/solar-generic-local/captures/singleRGB/TUM/tum_camera_calibration.json -L -o data/tum_camera_calibration.json

# Download TUM video for testing relocalization
curl https://artifact.b-com.com/solar-generic-local/captures/singleRGB/TUM/rgbd_dataset_freiburg3_long_office_household_relocalization.avi -L -o data/rgbd_dataset_freiburg3_long_office_household_relocalization.avi

# Download TUM map for testing relocalization
curl https://artifact.b-com.com/solar-generic-local/maps/TUM/freiburg3_long_office_household/map_linux_0_10_0.zip -L -o map.zip
unzip -o map.zip -d ./data
rm map.zip
