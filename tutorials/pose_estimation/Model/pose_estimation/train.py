import copy
import os

import torch
import torchvision

from pose_estimation.single_cube_dataset import SingleCubeDataset
from pose_estimation.evaluation_metrics.translation_average_mean_square_error import (
    translation_average_mean_square_error,
)
from pose_estimation.evaluation_metrics.orientation_average_quaternion_error import (
    orientation_average_quaternion_error,
)
from pose_estimation.evaluate import evaluate_one_epoch, evaluation_over_batch


def train_model(estimator):
    """
    Do the training process for the estimator

    Args:
        estimator: pose estimation estimator
    """
    config = estimator.config
    dataset_train = SingleCubeDataset(
        config=config,
        data_root=estimator.data_root,
        split="train",
        zip_file_name=config.train.dataset_zip_file_name_training,
        sample_size=config.train.sample_size_train,
    )
    dataset_val = SingleCubeDataset(
        config=config,
        data_root=estimator.data_root,
        split="validation",
        zip_file_name=config.val.dataset_zip_file_name_validation,
        sample_size=config.val.sample_size_val,
    )

    train_loader = torch.utils.data.DataLoader(
        dataset_train,
        batch_size=config.train.batch_training_size,
        num_workers=0,
        drop_last=True,
    )
    val_loader = torch.utils.data.DataLoader(
        dataset_val,
        batch_size=config.val.batch_validation_size,
        num_workers=0,
        drop_last=False,
    )

    train_loop(
        estimator=estimator,
        config=config,
        train_dataloader=train_loader,
        val_dataloader=val_loader,
    )


def train_loop(*, estimator, config, train_dataloader, val_dataloader):
    """
    Do the training loop: for each epoch, it does the
    training and the evaluation

    Args:
        estimator: pose estimation estimator
        label_id (int): corresponds to the label id in the captures_*.json folder minus 1.
        config (CfgNode): estimator config
        train_dataloader (torch.utils.data.DataLoader): training dataloader for the model training
        val_dataloader (torch.utils.data.DataLoader): validation dataloader to feed the model evaluation
    """
    config = config
    model = estimator.model.to(estimator.device)
    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.Adam(
        params,
        betas=(config.adam_optimizer.beta_1, config.adam_optimizer.beta_2),
        lr=config.adam_optimizer.lr,
    )

    n_epochs = config.train.epochs
    for epoch in range(1, n_epochs + 1):
        estimator.logger.info(f"Training Epoch[{epoch}/{n_epochs}]")
        _train_one_epoch(
            estimator=estimator,
            config=config,
            optimizer=optimizer,
            data_loader=train_dataloader,
            epoch=epoch,
        )

        if epoch % config.checkpoint.save_frequency == 0:
            estimator.checkpointer.save(estimator, epoch=epoch)

        if epoch % config.val.eval_freq == 0:
            evaluate_one_epoch(
                estimator=estimator,
                config=config,
                data_loader=val_dataloader,
                epoch=epoch,
                test=False,
            )


def _train_one_epoch(
    *, estimator, config, optimizer, data_loader, epoch,
):
    """
    Train the model on one epoch

    Args:
        estimator: pose estimation estimator
        label_id (int): corresponds to the label id in the captures_*.json folder minus 1.
        config: model configuration
        optimizer: pytorch optimizer
        data_loader(DataLoader): pytorch dataloader
        epoch (int): the number of the epoch we are at
    """
    estimator.model.train()
    optimizer.zero_grad()

    batch_size = config.train.batch_training_size

    criterion_translation = torch.nn.MSELoss()
    criterion_orientation = torch.nn.MSELoss()

    metric_translation, metric_orientation = evaluation_over_batch(
        estimator=estimator,
        config=config,
        data_loader=data_loader,
        batch_size=batch_size,
        epoch=epoch,
        is_training=True,
        optimizer=optimizer,
        criterion_translation=criterion_translation,
        criterion_orientation=criterion_orientation,
    )

    estimator.writer.log_training(
        training_metric_translation=metric_translation,
        training_metric_orientation=metric_orientation,
        epoch=epoch,
    )
