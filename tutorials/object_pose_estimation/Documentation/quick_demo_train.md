# Data Collection: Quick Demo

If you just want to run the completed project in order to collect your training and validation data this section can help do it. 

To learn how to build something like this from scratch, see [Part 1](1_set_up_the_scene.md) and [Part 2](2_set_up_the_data_collection_scene.md) of our tutorial.

**Table of Contents**
- [Prerequisites](#Prerequisites)
- [Setup](#setup)
- [Swichting to Data Collection Mode](#switch)
- [Data Collection](#data-collection)

## <a name="reqs">Prerequisites</a>

To follow this tutorial you need to **clone** this repository even if you want to create your Unity project from scratch. 

>Note For Windows Users:
You need to have a software enabling you to run bash files. One option is to download [GIT](https://git-scm.com/downloads). During installation of GIT, add GIT Bash to windows context menu by selecting its option. After installation right click in your folder select [GIT Bash Here](Images/0_GIT_installed.png).


1. Open a terminal and put yourself where you want to host the repository. 
```bash
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
```

2. [Install Unity `2020.2.*`.](install_unity.md)

3. Open the completed project. To do so, open Unity Hub, click the `Add` button, and select `PoseEstimationDemoProject` from the `Unity-Robotics-Hub/tutorials/object_pose_estimation/` folder. 

## <a name='setup'>Setup</a>

1. Open the scene. Go to `Assets > Scenes` and double click on `TutorialPoseEstimation`. 

2. The size of the images that will be used for pose estimation depends on a setting in the Game view. Select the `Game` view and select `Free Aspect`. Then select the **+**, with the message `Add new item` on it if you put your mouse over the + sign. For the Width select `650` and for the Height select `400`. A gif below shows you how to do it. 

<p align="center">
<img src="Gifs/2_aspect_ratio.gif"/>
</p>

## <a name="switch">Switching to Data Collection Mode</a>
The completed project is set up for inference mode by default, so we must switch it to data collection mode.

1. Uncheck the `ROSObjects` GameObject in the _**Hierarchy**_ tab to disable it.

2. On the `Simulation Scenario` GameObject, check the `Fixed Length Scenario` component to enable it.

3. On the `Main Camera` GameObject, check the `Perception Camera (Script)` component to enable it.

## <a name="data-collection">Data Collection</a>
To get strarted with the data collection, follow the instructions in [Part 3: Collect the Training and Validation Data](3_data_collection_model_training.md#step-1) of the tutorial. This section will explain how to set the random seed of the environment, choose how many training data examples you'd like to collect, and get it running. 

If you'd like to then move on to training a pose estimation model on the data you've collected, move on to [Part 3: Train the Deep Learning Model](3_data_collection_model_training.md#step-2). 

Have fun!
