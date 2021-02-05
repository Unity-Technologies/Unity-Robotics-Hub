# Pose Estimation Demo: Phase 3

In [Phase 1](1_set_up_the_scene.md) of the tutorial, we learned how to create our scene in the Unity editor.

In [Phase 2](2_set_up_the_data_collection_scene.md) of the tutorial, we learned:
* How to equip the camera for the data collection
* How to equip the cube for the data collection 
* How to create your own randomizer 
* How to add our custom randomizer

In this phase, we will be collecting a large dataset of RGB images of the scene, and the corresponding pose of the cube. We will then use this data to train a machine learning model to predict the cube's position and rotation from images taken by our camera. We will then be ready to use the trained model for our pick-and-place task in [Phase 4](4_pick_and_place.md).

Steps included in this phase of the tutorial:

**Table of Contents**
  - [Step 1: Collect the Training and Validation Data](#step-1-collect-the-training-and-validation-data)
  - [Step 2: Train the Deep Learning Model](#step-2-train-the-deep-learning-model)
  - [Exercises for the Reader](#exercises-for-the-reader)

---

### <a name="step-1">Step 1: Collect the Training and Validation Data</a>

Now it is time to collect the data: a set of images with the corresponding position and orientation of the cube relative to the camera.

We need to collect data for the training process and data for the validation one. 

We have chosen a training dataset of 30,000 images and a validation dataset of 3,000 images. 

* **Action**: Select the `Simulation Scenario` GameObject and in the _**Inspector**_ tab, in the `Fixed Length Scenario` and in `Constants` set the `Total Iterations` to 30000.

* **Action**: Press play and wait until the simulation is done. It should take a bit of time (~10 min). Once it is done, go to the _**Console**_ tab which is the tag on the right of the _**Project**_ tab. 

You should see something similar to the following: 

<p align="center">
<img src="Images/3_saved_data.png"/>
</p>

In my case the data is written to `/Users/jonathan.leban/Library/Application Support/DefaultCompany/SingleCubePoseEstimationProject` but for you the data path will be different. Go to that directory from your terminal.

You should then see something similar to the following: 
<p align="center">
<img src="Images/3_data_logs.png"/>
</p>

* **Action**: Change this folder's name to `UR3_single_cube_training`. 

* **Action**: Now we need to collect the validation dataset. Select the `Simulation Scenario` GameObject and in the _**Inspector**_ tab, in the `Fixed Length Scenario` and in `Constants` set the `Total Iterations` to 3000.

* **Action**: Press play and wait until the simulation is done. Once it is done go to the _**Console**_ tab and go to the directory where the data has been saved. 

* **Action**: Change the folder name where this latest data was saved to `UR3_single_cube_validation`. 

* **Action (Optional)**: Move the `UR3_single_cube_training` and `UR3_single_cube_validation` folders to a directory of your choice.  


### <a name="step-2">Step 2: Train the Deep Learning Model</a>
Now its time to train our deep learning model! We've provided the model training code for you, but if you'd like to learn more about it - or make your own changes - you can dig into the details [here](../Model).

This step can take a long time if your computer doesn't have GPU support (~5 days on CPU). Even with a GPU, it can take around ~10 hours. We have provided an already trained model as an alternative to waiting for training to complete. If you would like to use this provided model, you can proceed to [Phase 4](4_pick_and_place.md).

#### Requirements

We support two approaches for running the model: Docker (which can run anywhere) or locally with Conda. 

#### Option A: Using Docker
If you would like to run using Docker, you can follow the [Docker steps provided](../Model/documentation/running_on_docker.md) in the model documentation.


#### Option B: Using Conda 
To run this project locally, you will need to install [Anaconda](https://docs.anaconda.com/anaconda/install/) or [Miniconda](https://docs.conda.io/en/latest/miniconda.html). 

If running locally without Docker, we first need to create a conda virtual environment and install the dependencies for our machine learning model. If you only have access to CPUs, install the dependencies specified in the `environment.yml` file. If your development machine has GPU support, you can choose to use the `environment-gpu.yml` file instead.

* **Action**: In a terminal window, enter the following command to create the environment. Replace `<env-name>` with an environment name of your choice, e.g. `pose-estimation`:
```bash
conda env create -n <env-name> -f environment.yml
```

Then, you need to activate the conda environment.

* **Action**: Still in the same terminal window, enter the following command:
```bash
conda activate <env-name>
```

#### Updating the Model Config

At the top of the [cli.py](../Model/pose_estimation/cli.py) file in the model code, you can see the documentation for all supported commands. Since typing these in can be laborious, we use a [config.yaml](../Model/config.yaml) file to feed in all these arguments. You can still use the command line arguments if you want - they will override the config. 

There are a few settings specific to your setup that you'll need to change.

First, we need to specify the path to the folders where your training and validation data are saved:

* **Action**: In the [config.yaml](../Model/config.yaml), under `system`, you need to set the argument `data/root` to the path of the  directory containing your data folders. For example, since I put my data (`UR3_single_cube_training` and `UR3_single_cube_validation`) in a folder called `data` in Documents, I set the following:
```bash
  data_root: /Users/jonathan.leban/Documents/data
```

Second, we need to modify the location where the model is going to be saved: 

* **Action**: In the [config.yaml](../Model/config.yaml), under `system`, you need to set the argument `log_dir_system` to the full path to the output folder where your model's results will be saved. For example, I created a new directory called `models` in my Documents, and then set the following:
```bash
log_dir_system: /Users/jonathan.leban/Documents/models
```

#### Training the model
Now its time to train our deep learning model!

* **Action**: If you are not already in the `tutorials/pose_estimation/Model` directory, navigate there. 

* **Action**: Enter the following command to start training: 
```bash 
python -m pose_estimation.cli train 
```

**Note (Optional)**: If you want to override certain training hyperparameters, you can do so with additional arguments on the above command. See the documentation at the top of [cli.py](../Model/pose_estimation/cli.py) for a full list of supported arguments.

**Note**: If the training process ends unexpectedly, check the [Troubleshooting Guide](troubleshooting.md) for potential solutions.

#### Visualizing Training Results with Tensorboard
If you'd like to examine the results of your training run in more detail, see our guide on [viewing the Tensorboard logs](tensorboard.md).

#### Evaluating the Model
Once training has completed, we can also run our model on our validation dataset to measure its performance on data it has never seen before. 

However, first we need to specify a few settings in our config file.

* **Action**: In [config.yaml](../Model/config.yaml), under `checkpoint`, you need to set the argument `log_dir_checkpoint` to the path where you have saved your newly trained model.

**Action**: If you are not already in the `Model`, navigate there. 

**Action** To start the evaluation run, enter the following command: 
```bash 
python -m pose_estimation.cli evaluate 
```

**Note (Optional)**: To override additional settings on your evaluation run, you can tag on additional arguments to the command above. See the documentation in [cli.py]() for more details.


### Exercises for the Reader
**Optional**: If you would like to learn more about randomizers and apply domain randomization to this scene more thoroughly, check out our further exercises for the reader [here](5_more_randomizers.md).

### Proceed to [Phase 4](4_pick_and_place.md).

### 

### Go back to [Phase 2](2_set_up_the_data_collection_scene.md)
