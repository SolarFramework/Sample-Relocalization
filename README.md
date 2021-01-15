# Sample-Relocalization

[![License](https://img.shields.io/github/license/SolARFramework/Sample-Slam?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)


The SolAR **Relocalization samples** show a SolAR pipeline of camera relocalization from each RGB image independently based on a prebuilt map.

## How to run

### Bag Of Word Vocabulary

:warning: Don't forget to download the [fbow vocabularies](https://github.com/SolarFramework/binaries/releases/download/fbow%2F0.0.1%2Fwin/fbow_voc.zip) unzip this archive and put the `akaze.fbow` in your execution folder.

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
<pre><code>SolARRelocalizationMono.exe</code></pre>

* For multithreading test:
<pre><code>SolARRelocalizationMulti.exe</code></pre>

* For pipeline plugin test:
<pre><code>TestRelocalizationPipeline.exe</code></pre>

Press `escape` to quit the application.

## Contact 
Website https://solarframework.github.io/

Contact framework.solar@b-com.com