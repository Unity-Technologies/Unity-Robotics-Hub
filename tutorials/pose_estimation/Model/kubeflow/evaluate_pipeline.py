import kfp.dsl as dsl
import kfp.gcp as gcp


def evaluate_op(
    *,
    gcs_bucket,
    pose_estimation_gcs_path,
    log_dir,
    docker_image,
    memory_limit,
    num_gpu,
    gpu_type,
    checkpoint_file=None,
):
    """ Create a Kubeflow ContainerOp to train an estimator on the cube sphere dataset.
    Args:
        gcs_bucket: GCS Bucket where the datasets are located
        pose_estimation_gcs_path: Path inside the gcp bucket where the datasets are located
        log_dir: path to save the Tensorboard event files.
        docker_image (str): Docker image registry URI.
        memory_limit (str): Set memory limit for this operator. For simplicity,
            we set memory_request = memory_limit.
        num_gpu (int): Set the number of GPU for this operator
        gpu_type (str): Set the type of GPU
    Returns:
        kfp.dsl.ContainerOp: Represents an op implemented by a container image
            to train an estimator.
    """

    command = ["python", "-m", "pose_estimation.cli"]
    arguments = [
        "evaluate",
        "--config-file=config.yaml",
        "--download-data-gcp=True",
        f"--gcs-bucket={gcs_bucket}",
        f"--pose-estimation-gcs-path={pose_estimation_gcs_path}",
        f"--log-dir={log_dir}",
    ]

    evalulate = dsl.ContainerOp(
        name="train", image=docker_image, command=command, arguments=arguments,
    )
    # GPU
    evalulate.set_gpu_limit(num_gpu)
    evalulate.add_node_selector_constraint(
        "cloud.google.com/gke-accelerator", gpu_type
    )

    evalulate.set_memory_request(memory_limit)
    evalulate.set_memory_limit(memory_limit)

    evalulate.apply(gcp.use_gcp_secret("user-gcp-sa"))

    return evalulate


@dsl.pipeline(
    name="evalulate pipeline",
    description="evalulate the model using kubeflow pipeline",
)
def evalulate_pipeline_single_cube(
    docker_image: str = "",
    gcs_bucket: str = "",
    pose_estimation_gcs_path: str = "",
    logdir: str = "",
):

    memory_limit = "64Gi"
    num_gpu = 1
    gpu_type = "nvidia-tesla-v100"

    # Pipeline definition
    evaluate_op(
        gcs_bucket=gcs_bucket,
        pose_estimation_gcs_path=pose_estimation_gcs_path,
        log_dir=logdir,
        docker_image=docker_image,
        memory_limit=memory_limit,
        num_gpu=num_gpu,
        gpu_type=gpu_type,
    )


if __name__ == "__main__":
    import kfp.compiler as compiler

    compiler.Compiler().compile(
        evalulate_pipeline_single_cube, __file__ + ".tar.gz"
    )
