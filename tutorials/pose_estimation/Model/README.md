Pose Estimation Model
=====================
This repository enables users to train and evaluate a deep neural network to predict the pose of a single object from RGB images. We provide support for running both locally and with Docker.  

This project uses synthetic training data collected in Unity. To learn more about that, see our data collection [tutorial](https://github.cds.internal.unity3d.com/unity/Pose-Estimation-Demo).

This model is a modified implementation of [Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World](https://arxiv.org/pdf/1703.06907.pdf), by Tobin et. al. It is based on the classic VGG-16 architecture, and initialized with weights pre-trained on the ImageNet dataset. The head of the network is replaced with a 3D position prediction head that outputs (x, y, z), and an orientation predicton head that outputs a quaternion (q<sub>x</sub>, q<sub>y</sub>, q<sub>z</sub>, q<sub>w</sub>). 

<p align='center'>
  <img src='documentation/docs/network.png'/>
</p>

### Table of contents
- [Requirements](#requirements)
- [Running on local](#running-on-local)
  - [Dataset](#dataset) 
  - [Save and Load](#save-and-load)
    - [Save](#save)
    - [Load](#load)
  - [CLI](#cli)
    - [Train](#train)
    - [Evaluate](#evaluate)
- [Running on Docker](#running-on-docker)
- [Running on the Cloud](#running-on-the-cloud)
- [Visualizing the Results](#visualizing-the-results)
- [Unit Testing](#unit-testing)

### Supporting Documentation
- [Codebase Structure](documentation/codebase_structure.md)
- [Running on Docker](documentation//running_on_docker.md)
- [Running on the Cloud](documentation/running_on_the_cloud.md)

---

## Requirements

To run this project on local, you will need to install [Anaconda](https://docs.anaconda.com/anaconda/install/) or [Miniconda](https://docs.conda.io/en/latest/miniconda.html).

By following the instructions below, you will create a conda environment, so you do not need to already have Python 3 installed on your machine.

## Running on local
* **Action**: First, you need to clone this git repository: 
```bash 
git clone git@github.com:Unity-Technologies/Pose-Estimation-Model.git
```

A new folder called `Pose-Estimation-Model` has been created in your current directory. 
* **Action**: Go inside that directory by entering the following command in your terminal: 
```bash 
cd Pose-Estimation-Model
```

**NOTE**: You have two options: use a conda environment or use Docker. If you want to use Docker then click on [this](#running-on-docker). 

Then, you need to create a conda environment with the dependencies of your [environment.yml](environment.yml) file and **if your development machine has GPU support**, you can choose to use [environment-gpu.yml](environment-gpu.yml) file instead. 
This will create and activate the project in a virtual environment with Python 3 and all the packages required to run the project properly so that you don't have to install something on your computer. 
* **Action**: Still in the same terminal window, enter the following: 
```bash
conda env create -n <env-name> -f environment.yml
```

* **Action**: Then you need to activate the environment:
```bash
conda activate <env-name>
```

Then, once the project is created, you can run python commands and thus train or evaluate your neural network model. But to run those commands, the project needs to have access to the datasets. 


### Dataset
For the datasets, you have the ability to use a cloud like google cloud platform or to use your local computer. 
This feature is controlled by the argument `download_data_gcp` under `dataset` in [config.yaml](config.yaml) 
and by `--download-dat-gcp` when you launch the command in the terminal (see the [CLI](#cli) section further down)

In this section, I will show you how to run it on local.

**Note**: If you want to know more information on how to run it on a cloud, then go to [running_on_the_cloud](documentation/running_on_the_cloud.md).

There are two datasets you need to have on your local computer: `UR3_single_cube_training` and 
`UR3_single_cube_validation` for the training and the validation process. To create those datasets, you need to follow the Phase 1, 2 and 3 of the [Pose Estimation Demo](https://github.cds.internal.unity3d.com/unity/Pose-Estimation-Demo) tutorial. 

There are few steps you need to follow in order to feed your neural network from the data properly:
* **Action**: In the [config.yaml](config.yaml), at the bottom you can find the argument `data_root` 
under `system`. Here you need to enter the root of the upper level directory of your data. 
For example, you can put your data in a folder called `data` that you have created inside your `Documents` folder. On mac you will enter: `/Users/user.name/Documents/data`

* **Action**: In the [config.yaml](config.yaml), you also need to set the argument `download_data_gcp` under `dataset` to `False`. 

### Save and Load
#### Save
Now you have two options to save your model and your metrics (logger): either you save it on local or you save it on google cloud (you can use another cloud but you will have to make the changes yourself). 

* **Action**: in the [config.yaml](config.yaml) file, under the argument `system` there is an argument `log_dir_system`. This argument defines the directory where you want to save the model and the metrics. You need to put the full local path.  

As I am a Mac User, the path will be different if you are working on Windows.

**Note**: For more information on how the save method works, you can go in the [codebase_structure.md](documentation/codebase_structure.md) file, in the section `Save and Load methods`.

#### Load
You can load a model, so that you can evaluate the performance or continue the training. 

* **Action**: In the [config.yaml](config.yaml) file, under the key `system` there is a key `log_dir_system`. This specifies the directory where the model and metrics will be saved. Be sure to include the full local path. 
 
**Note**: For more information on how the save method works, you can go in the [codebase_structure.md](documentation/codebase_structure.md) file, in the section `Save and Load methods`.

### CLI 
At the top of the [cli.py](pose_estimation/cli.py) file, you can see the documentation for all supported commands. 

#### Train
To run the training commmand, you need to adopt the following format: 

```bash
cli.py train [options] [config] [dataset] [training-options] [hyperparameter-options] [save-options] [loading-options]
```

Thus, if you want to keep the [config.yaml](config.yaml) as it is and change the number of epochs to 5, the training batch 
size to 10 and set to 20 the number of steps you need to accumulate to upgrade the gradient, then the command will be: 
* **Action**: 
```bash 
python -m pose_estimation.cli train --epochs=5 --batch-training-size=10 --accumulation-steps=20
```

#### Evaluate  
To run the evaluate commmand, you need to adopt the following format: 

```bash
cli.py evaluate [options] [config] [dataset] [evaluation-options] [save-options] [loading-options]
```

Thus, if you want to keep the [config.yaml](config.yaml) as it is and change the test batch size to 10 and the path where you have saved the already trained model you want to use which is `/Users/first_name.last_name/Documents/save/UR3_single_cube_model_ep120.tar`. 
* **Action**: 
```bash 
python -m pose_estimation.cli evaluate --batch-test-size=10 --load-dir-checkpoint=/Users/first_name.last_name/Documents/save/UR3_single_cube_model_ep120.tar
```

## Running on Docker 
If you want to run the project on Docker, then follow [this guide](documentation/running_on_docker.md). 

## Running on the Cloud
If you want to run the project on the Cloud, then follow [this guide](documentation/running_on_the_cloud.md). 

## Visualizing the Results
To view the training or evaluate logs you can you use tensorboard. The logs are saved in the same directory the model is saved. 
You need to run the following command:

* **Action**: 
```bash
tensorboard --logdir=[LOG DIRECTORY]
```

For example if you have saved all your models in a folder called `save` inside your Documents folder, open a new terminal, put yourself into your `Documents` directory: 
```bash
tensorboard --logdir=save
```

You should see something similar to that: 
<p align="center">
<img src="documentation/docs/tensorboard.png" height=60/>
</p>

Then, as you can see on the image, my tensorboard will be accessible on the port 6006 from local. 

* **Action**: Open your internet browser and in the search bar, enter:
```
localhost:[PORT_NUMBER]
```

For me as my port is `6006` I will enter: 
```
localhost:6006
```

If you train your model following our suggestions you should see something similar to the following: 
<p align="center">
<img src="documentation/docs/performance_model.png"/>
</p>

Below is a description of the model's performance. For the loss, I used the L2 norm for the position and orientation. However, to evaluate the performance of my model both in the training and the validation part, I used the [translation_average_mean_square_error.py](pose_estimation/evaluation_metrics/translation_average_mean_square_error.py) which is the average of the L2 norm over the dataset and the [orientation_average_quaternion_error.py](pose_estimation/evaluation_metrics/orientation_average_quaternion_error.py) which is the average of the angle between the orientation of the prediction and the orientation of the target over the dataset.

|                                   | Training | Validation|
|:---------------------------------:|:--------:|:---------:|
|Translation (% of the cube's size) |   12%    |   10%     |
|Orientation (radian)               |   0.06   |   0.05    |   



## Unit Testing

We use [pytest](https://docs.pytest.org/en/latest/) to run tests located under `tests/`. You can run the tests after having done the instructions in the [Running on Local](#running-on-local) commands.

You can run the entire test suite with

* **Action**:
```bash
python -m pytest
```

or run individual test files with:

```bash
python -m pytest tests/test_average_translation_mean_square_error.py
```
