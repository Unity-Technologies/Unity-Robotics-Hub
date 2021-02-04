Codebase Structure
==================

In this project, I create a network to predict the position of a cube. 

### Architecture
The pose estimation project is organized as following.

PoseEstimationModel:
*    [environment-gpu.yml](../environment-gpu.yml):
        If the computer your are runnning the project from **has a gpu support**, this file sets the dependencices of the project and the different packages to install. It is meant to be used when you create your conda environment. 

*    [environment.yml](../environment.yml):
        If you the computer your are runnning the project from **does not have a gpu support**, this file sets the dependencices of the project and the different packages to install. It is meant to be used when you create your conda environment. 

*    [setup.py](../setup.py):
        This file is to create a package as your project. 

*    [cli.py](../pose_estimation/cli.py):
        This file contains the cli commands which are the commands to launch the different processes (either train or evaluate).

*    [config.yaml](../config.yaml):
        This file contains the default configuration for the estimator (pose estimation model) on the single cube dataset.

*    [single_cube_dataset.py](../pose_estimation/single_cube_dataset.py):
        This file contains knowledge on how the SingleCubeDataset class dataset should be loaded into memory. 

*    [model.py](../pose_estimation/model.py):
        This file contains the neural network along with the custom linear activation function to perform 
        the pose estimation task: predict the object's translation (coordinates x, y, z of the cube's center) 
        and the cube's orientation (quaternion describing the orientation of the cube) if the object is asymmetric otherwise it will predict only the translation.

*    [pose_estimation_estimator.py](../pose_estimation/pose_estimation_estimator.py):
        This file contains the pose estimation estimator and the different methods you can apply on your model as train, evaluate, save and 
        load.

*    [train.py](../pose_estimation/train.py):
        This file contains the model training process.

*    [evaluate.py](../pose_estimation/evaluate.py):
        This file contains the model evaluation process.

*    [evaluation_metrics](../pose_estimation/evaluation_metrics/):
        This module contains metrics used by the pose estimation estimator.

*    [logger.py](../pose_estimation/logger.py):
        This module contains the logger class which is a class designed to save elements (metrics, losses) visible on tensorboard. 

*    [storage](../pose_estimation/storage):
        This module contains functionality that relates to
        writing/downloading/uploading to/from different sources.

*    [tests](../tests):
        This module contains all the tests which you can run using the [pytest command](../README.md#unit-testing).

*    [Dockerfile](../Dockerfile):
        This file  is the file reponsible for the creation of the docker image. 

*    [kubeflow](../kubeflow/):
        This module contains kubeflow pipelines ([.py.tar.gz](../kubeflow/train_pipeline.py) files). You can have more information on how to set up a kubeflow pipeline in the [ReadMe](../kubeflow/README.md).


### Details of the config.py file 
In the following, I will explain what each argument in the [config.yaml](../config.yaml) means. 
There are 8 sections in the config files: 

* **estimator**: This will be the core name of the saved model

* _**train**_: 
  - **dataset_zip_file_name_training**: name of the training dataset file.

  - **batch_training_size**: number of training samples to work through before the model’s internal parameters are updated.

  - **accumulation_steps**: number of backwards passes which are performed before updating the parameters. The goal is to have the same model parameters for multiple inputs (batches) and then update the model's parameters based on all these batches, instead of performing an update after every single batch.

  - **epochs**: number of passes of the entire training dataset the machine learning algorithm has completed.

  - **beta_loss**: beta coefficient when we add the translation and orientation losses. 

  - **sample_size_train**: size of a dataset training sample. It is used to test operations/commands on a few examples.

* _**val**_:

  - **dataset_zip_file_name_validation**: name of the validation dataset file.

  - **batch_validation_size**: number of validation samples to work through before the metrics are calculated.
 
  - **eval_freq**: frequency of epochs when the evaulation process is launched.

  - **sample_size_val**: size of a dataset validation sample. It is used to test operations/commands on a few examples.

* _**test**_:

  - **dataset_zip_file_name_test**: name of the test dataset file.

  - **batch_test_size**: number of training samples to work through before the metrics are calculated.

  - **sample_size_test**: size of a dataset test sample. It is used to test operations/commands on a few examples.

* _**dataset**_:

  - **image_scale**: size of the image we use to train the model (some processings need to be applied to give as input to the model though).

  - **download_data_gcp**: if True it will download the data from gcp otherwise it will use the data you have on local.

  - **gcs_bucket**: name of GCS Bucket where the datasets are located.

  - **pose_estimation_gcs_path**: path inside the gcp bucket where the datasets are located.

  - **symmetric**: Boolean. If the object is symmetric then the element is True otherwise it is False. Based on that we will only predict the translation 
  or translation and orientation. 

* _**adam_optimizer**_:

  - **lr**: learning rate which scales the magnitude of our weight updates in order to minimize the network's loss function.

  - **beta_1**: the exponential decay rate for the first moment estimates.

  - **beta_2**: the exponential decay rate for the second-moment estimates.


* _**checkpoint**_: 
        
  - **load_dir_checkpoint**: path towards the saved model.

  - **save_frequency**: frequency of epochs when the model is saved. If it is set to 1 then the model will be saved every epoch and if it is set to 2 then the model will be saved ever two epochs.

* _**system**_: 

  - **log_dir_system**: path where the model and the metrics (.tar file that will be visioned by tensorbard) will be saved.

  - **data_root**: path towards the upper directory of the data.


### Save and Load methods
In the [pose_estimation_estimator.py](../pose_estimation/pose_estimation_estimator.py) file, there is one method to save and one method to load a model. 

The save method is called in [train.py](../pose_estimation/train.py) file at the line 95 and the load method is called in the [pose_estimation_estimator.py](../pose_estimation/pose_estimation_estimator.py) line 82. The model is saved using the save method of the checkpointer object and the model is loaded using the load method of the checkpointer object. The checkpointer object is created in [pose_estimation_estimator.py](../pose_estimation/pose_estimation_estimator.py) file line 50. Then, to understand how the model is saved or loaded we need to look into the [checkpoint.py](../pose_estimation/storage/checkpoint.py) file. 

But first, let's have a general overview of the [checkpoint.py](../pose_estimation/storage/checkpoint.py) file. There are three classes:
- **EstimatorCheckpoint**: it assigns `estimator checkpoint writer` according to `log_dir` which is responsible for saving estimators. The writer can be a GCS or local writer. It also assigns `loader` which is responsible for loading estimator from a given path. Loader can be a local, GCS or HTTP loader.
- **LocalEstimatorWriter**: it is to write (saves) estimator checkpoints locally.
- **GCSEstimatorWriter**: it is to write (saves) estimator checkpoints on GCP (Google Cloud Platform).

When the EstimatorCheckpoint object is created, the static method `_create_writer` is called and based on the format of the `log_dir`, which is the directory where you want to save your model (attribute `log_dir` under `system` in [config.yaml](../config.yaml)), a `LocalEstimatorWriter` or a `GCSEstimatorWriter` object is created.

#### Save
Now you have two options to save your model, either you save it on local or you save it on google cloud (you can use another cloud but you will have to make the changes yourself).
Then, if we go back to the method called to save the model, it is the `save` method of the `EstimatorCheckpoint` object. This method calls the `save` method of the object created by the `_create_writer` method.

*    `local`: the class `LocalEstimatorWriter` takes as attributes a `dirname` which is the `log_dir` path, a `prefix` which is the name of the estimator (corresponds to the argument `estimator` in the [config.yaml](../config.yaml) file), and a `suffix` which is by default equal to `.tar` (type of the file) and create a directory which will host the model. Then, the method `save` calls the method `save` of the estimator which in the [pose_estimation_estimator.py](../pose_estimation/pose_estimation_estimator.py) file. 

*    `gcp`: The class `GCSEstimatorWriter` takes as attributes `cloud_path` which is the `log_dir` path towards the gcp bucket and a `prefix` which is the name of the estimator (corresponds to the argument `estimator` in the [config.yaml](../config.yaml) file). In the method `save`, the model is saved on a temporary directory on the local computer that the cloud uses. The process used is the process I just described a little bit above in `local`. The `save` method returns the full GCS cloud path to the saved checkpoint file. Then the method `upload` from the [GCSClient()](../pose_estimation/storage/gcs.py) class is called: it is a method to upload files on Google Cloud Platform.

#### Load
Now you have three options to load your model, either you load it from local, you can load it on google cloud (you can use another cloud but you will have to make the changes yourself) or you can load it from http.
Then, if we go back to the method called to load the model, it is the `load` method of the `EstimatorCheckpoint` object. This method calls the `_get_loader_from_path` and based on the path, a load method will be launched (load_local, load_from_gcs, load_from_http).