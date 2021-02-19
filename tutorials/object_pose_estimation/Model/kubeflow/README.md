# Kubeflow pipelines 
The Kubeflow pipelines are located inside the [kubeflow/](kubeflow/) folder, where you can find one pipeline for training the model, and one for the evaluation. 

Train and Evaluate Model using Kubeflow
=======================================

## Create New Pipeline
You will need a Python environment with [kfp==0.5.1](https://pypi.org/project/kfp/) installed. 

### Compile Kubeflow pipeline

```bash
cd kubeflow
python pose_estimation_train_pipeline.py
```

This will create a file `pose_estimation_train_pipeline.py.tar.gz` which can be uploaded to Kubeflow pipeline for executions. 

Next, go to the Kubeflow dashboard, and upload and create new pipeline using the above pipeline. You should be able to create a new parameterized experiment to run a Kubeflow pipeline following [this tutorial](https://www.kubeflow.org/docs/pipelines/pipelines-quickstart).

## Train a model on Kubeflow for Single Object Pose Estimation
Go to [this pipeline](https://www.kubeflow.org/docs/gke/deploy/) and follow the tutorial to create a Kubeflow project. Once you have set up your project, go to the dashboard and press the `Create Run` button at the top of the screen (it has a solid blue fill and white letters).

1. Set `docker_image` to be `gcr.io/unity-ai-thea-test/datasetinsights:<git-commit-sha>` Where `<git-commit-sha>` is the sha from the latest version of master in the thea repo. It should be the latest commit in the history: [link](https://gitlab.internal.unity3d.com/machine-learning/thea/commits/master).

2. To check the progress of your model run `docker run -p 6006:6006 -v $HOME/.config:/root/.config:ro tensorflow/tensorflow tensorboard --host=0.0.0.0 --logdir gs://your/log/directory`. Open `http://localhost:6006` to view the dashboard. 
    
    This command assumes you have run `gcloud auth login` command and the local credential is stored in `$HOME/.config`, which is mounted to the home directory inside Docker. It must have read permission to `gs://your/log/directory`

3. If the mAP and mAR for validation are leveling off then you can terminate the run early; it's unlikely the model's performance will improve.
   
4.  The model will save checkpoints after every epoch to the logdir with the format `gs://logdir/ep#.estimator`, e.g.
`gs://thea-dev/runs/20200328_221415/FasterRCNN.ep24.estimator`
