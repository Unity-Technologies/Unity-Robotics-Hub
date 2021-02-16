from pose_estimation.pose_estimation_estimator import PoseEstimationEstimator
from pose_estimation.single_cube_dataset import SingleCubeDataset
from pose_estimation.evaluate import evaluate_models, evaluate_one_epoch
from unittest.mock import MagicMock, patch
import os
import tempfile

import pytest
import torch
from yacs.config import CfgNode as CN

from pose_estimation.storage.checkpoint import EstimatorCheckpoint

# XXX This should not be a global variable. A tempdir should be a fixture and
# automatically cleanup after EVERY unit test finished execution.
tmp_dir = tempfile.TemporaryDirectory()
tmp_name = tmp_dir.name

data_root = os.path.join(os.getcwd(), "tests")
zip_file_name = "test_single_cube_dataset"
root = os.path.join(data_root, zip_file_name)


@pytest.fixture
def config():
    """prepare config."""
    with open("tests/config/test_config.yaml") as f:
        cfg = CN.load_cfg(f)

    return cfg


@pytest.fixture
def dataset(config):
    """prepare dataset."""
    dataset_test = SingleCubeDataset(
        config=config,
        data_root=data_root,
        zip_file_name=zip_file_name,
        sample_size=0,
        download=False,
    )
    return dataset_test


class TestEvaluate:
    @patch("pose_estimation.evaluate.evaluate_one_epoch")
    def test_evaluate_model(self, mock_evaluate_one_epoch, config, dataset):
        """test train on all epochs."""
        log_dir = os.path.join(tmp_name, "eval")
        if not os.path.exists(log_dir):
            os.mkdir(log_dir)

        config.system.data_root = data_root
        config.system.log_dir_system = log_dir

        pose_estimation_estimator = PoseEstimationEstimator(config=config)

        evaluate_models(estimator=pose_estimation_estimator)

        mock_evaluate_one_epoch.assert_called_once()
