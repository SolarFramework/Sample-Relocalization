# Sample-Relocalization

[![License](https://img.shields.io/github/license/SolARFramework/Sample-Slam?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)


The SolAR **Relocalization samples** show a SolAR pipeline of camera relocalization from each RGB image independently based on a prebuilt map.

## How to run

### Install required data

Before running the samples, you need to download data such as videos and the vocabulary of the bag of word used for image retrieval.
To install the required data, just launch the following script:

> #### Windows
>
	installData.bat

> #### Linux
>
	./installData.sh

This script will install the following data into the `./data` folder:
- The bag of words downloaded from our [GitHub releases] (https://github.com/SolarFramework/binaries/releases/download/fbow%2F0.0.1%2Fwin/fbow_voc.zip) and unzipped in the `./data/fbow_voc` folder.

### Camera Calibration

We provide a defaut camera calibration file that contains intrinsic parameters of the camera logitech C920.
If you want to change the calibration parameters of the camera, edit the `camera_calibration.yml` file.

### Prebuilt map

These relocalization samples require a prebuilt map of a scene. 
To do this, you can use our [SLAM samples](https://github.com/SolarFramework/Sample-Slam) that allow to create a map from a RGB sequence.
When it finishes, you archive a map in a folder. 

Open the configuration file corresponding to each relocalization sample and set the **directory** value of the **SolARMapper** to the path to the prebuilt map above.

### Run samples

You can run the relocalization samples using a webcam, an image sequence or a video by setting the configuration file.
From the binary directory, run following command for testing SolAR Relocalization:

* For mono thread test:
> #### Windows
>
	SolARRelocalizationMono.exe

> #### Linux
>
	./run.sh ./SolARRelocalizationMono

* For multithreading test:
> #### Windows
>
	SolARRelocalizationMulti.exe

> #### Linux
>
	./run.sh ./SolARRelocalizationMulti

* For pipeline plugin test:
> #### Windows
>
	TestRelocalizationPipeline.exe

> #### Linux
>
	./run.sh ./TestRelocalizationPipeline


Press `escape` to quit the application.

## Contact 
Website https://solarframework.github.io/

Contact framework.solar@b-com.com
