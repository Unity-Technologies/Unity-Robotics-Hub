import torch
import torchvision

from pose_estimation.single_cube_dataset import SingleCubeDataset
from pose_estimation.evaluation_metrics.translation_average_mean_square_error import (
    translation_average_mean_square_error,
)
from pose_estimation.evaluation_metrics.orientation_average_quaternion_error import (
    orientation_average_quaternion_error,
)


def evaluate_model(estimator):
    """
    Do the evaluation process for the estimator

    Args:
        estimator: pose estimation estimator
    """
    config = estimator.config

    dataset_test = SingleCubeDataset(
        config=config,
        split="test",
        zip_file_name=config.test.dataset_zip_file_name_test,
        data_root=config.system.data_root,
        sample_size=config.test.sample_size_test,
    )

    estimator.logger.info("Start evaluating estimator: %s", type(estimator).__name__)

    test_loader = torch.utils.data.DataLoader(
        dataset_test,
        batch_size=config.test.batch_test_size,
        num_workers=0,
        drop_last=False,
    )

    estimator.model.to(estimator.device)
    evaluate_one_epoch(
        estimator=estimator,
        config=config,
        data_loader=test_loader,
        epoch=0,
        test=True,
    )


def evaluate_one_epoch(*, estimator, config, data_loader, epoch, test):
    """Evaluation of the model on one epoch
    Args:
        estimator: pose estimation estimator
        config: configuration of the model
        data_loader (DataLoader): pytorch dataloader
        epoch (int): the current epoch number
        test (bool): specifies which type of evaluation we are doing
    """
    estimator.model.eval()
    estimator.logger.info(f" evaluation started")

    metric_translation = 0.0
    metric_orientation = 0.0

    if test:
        batch_size = config.test.batch_test_size
    elif test == False:
        batch_size = config.val.batch_validation_size
    else:
        raise ValueError(f"You need to specify a boolean value for the test argument")

    number_batches = len(data_loader) / batch_size
    with torch.no_grad():
        metric_translation, metric_orientation = evaluation_over_batch(
            estimator=estimator,
            config=config,
            data_loader=data_loader,
            batch_size=batch_size,
            epoch=epoch,
            is_training=False,
        )

        estimator.writer.log_evaluation(
            evaluation_metric_translation=metric_translation,
            evaluation_metric_orientation=metric_orientation,
            epoch=epoch,
            test=test,
        )


# HELPER
def evaluation_over_batch(
    *,
    estimator,
    config,
    data_loader,
    batch_size,
    epoch,
    is_training=True,
    optimizer=None,
    criterion_translation=None,
    criterion_orientation=None,
):
    """
    Do the training process for all the estimators (one for each class)

    Args:
        estimator: pose estimation estimator
        config: configuration of the model
        data_loader (DataLoader): pytorch dataloader
        batch_size (int): size of the batch
        epoch (int): the current epoch number
        is_training (bool): boolean to say if we are in a training process or not
        optimizer: optimizer of the model
        criterion_translation (torch.nn): criterion for the evaluation of the translation loss
        criterion_orientation torch.nn: criterion for the evaluation of the orientation loss
    """
    
    sample_size = config.train.sample_size_train if is_training else config.val.sample_size_val
    len_data_loader = sample_size if (sample_size > 0) else len(data_loader)

    metric_translation = 0
    metric_orientation = 0

    for index, (images, target_translation_list, target_orientation_list) in enumerate(
        data_loader
    ):
        images = list(image.to(estimator.device) for image in images)

        loss_translation = 0
        loss_orientation = 0

        if estimator.model.is_symetric == False:
            output_translation, output_orientation = estimator.model(
                torch.stack(images).reshape(
                    -1, 3, config.dataset.image_scale, config.dataset.image_scale
                )
            )

            target_translation = target_translation_list.to(estimator.device)
            target_orientation = target_orientation_list.to(estimator.device)

            metric_translation += translation_average_mean_square_error(
                output_translation, target_translation
            )

            metric_orientation += orientation_average_quaternion_error(
                output_orientation, target_orientation
            )

            intermediate_mean_loss_translation = metric_translation / (index + 1)
            intermediate_mean_loss_orientation = metric_orientation / (index + 1)
            estimator.logger.info(
                f"intermediate mean translation loss after mini batch {index + 1} in epoch {epoch} is: {intermediate_mean_loss_translation}"
            )
            estimator.logger.info(
                f"intermediate mean orientation loss after mini batch {index + 1} in epoch {epoch} is: {intermediate_mean_loss_orientation}"
            )

            if is_training:
                loss_translation += criterion_translation(
                    output_translation, target_translation
                )
                loss_orientation += criterion_orientation(
                    output_orientation, target_orientation
                )
                train_loss = (
                    loss_translation + config.train.beta_loss * loss_orientation
                )

        else:
            output_translation = estimator.model(
                torch.stack(images).reshape(
                    -1, 3, config.dataset.image_scale, config.dataset.image_scale
                )
            )

            target_translation = target_translation_list.to(estimator.device)

            metric_translation += translation_average_mean_square_error(
                output_translation, target_translation
            )

            intermediate_mean_loss_translation = metric_translation / (index + 1)
            estimator.logger.info(
                f"intermediate mean translation loss after mini batch {index + 1} in epoch {epoch} is: {intermediate_mean_loss_translation}"
            )

            if is_training:
                loss_translation += criterion_translation(
                    output_translation, target_translation
                )
                train_loss = loss_translation

        if is_training:
            train_loss.backward()

            if (index + 1) % config.train.accumulation_steps == 0:
                optimizer.step()
                optimizer.zero_grad()

    metric_translation = metric_translation / len_data_loader
    metric_orientation = metric_orientation / len_data_loader

    return metric_translation, metric_orientation
