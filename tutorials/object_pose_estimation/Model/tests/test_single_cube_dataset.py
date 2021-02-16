from pose_estimation.single_cube_dataset import (
    SingleCubeDataset,
    RawDataIterator,
)
import os

import pytest
from yacs.config import CfgNode as CN

data_root = os.path.join(os.getcwd(), "tests")
zip_file_name = "test_single_cube_dataset"
root = os.path.join(data_root, zip_file_name)


@pytest.fixture
def config():
    """prepare config."""
    with open("tests/config/test_config.yaml") as f:
        cfg = CN.load_cfg(f)

    return cfg


class TestSingleCubeDataset:
    def test_RawDataIterator(self, config):
        raw_data_iterator = RawDataIterator(path=root)
        data_dicts = []
        image_paths = []

        for data_dict, image_path in raw_data_iterator:
            data_dicts.append(data_dict)
            image_paths.append(image_path)

        assert len(data_dicts) == 10
        assert len(image_paths) == 10
        assert (len(set(image_paths))) == 10

        for i in range(len(data_dicts)):
            for j in range(len(config.dataset.symmetric)):
                data_dict = data_dicts[i][j]

                print(data_dict)
                assert "translation" in data_dict.keys()
                assert "rotation" in data_dict.keys()
                assert len(data_dict['translation']) == 3
                assert len(data_dict['rotation']) == 4

    def test_SingleCubeIterator(self, config):
        dataset_iterator = SingleCubeDataset(
            config=config,
            data_root=data_root,
            zip_file_name=zip_file_name,
            sample_size=0,
            download=False,
        )
        images, targets_trans, targets_orient = [], [], []

        assert len(dataset_iterator) == 10

        for image, target_trans, target_orient in dataset_iterator:
            images.append(image)
            targets_trans.append(target_trans)
            targets_orient.append(target_orient)
            print(target_orient)
        assert len(images) == 10
        assert len(targets_trans) == 10
        assert len(targets_orient) == 10

        print(targets_trans[0])

        for i in range(len(images)):
            for j in range(len(config.dataset.symmetric)):
                target_trans = targets_trans[i][j]
                target_orient = targets_orient[i][j]

                assert len(target_trans) == 3
                assert len(target_orient) == 4

        dataset_sample_iterator = SingleCubeDataset(
            config=config,
            data_root=data_root,
            zip_file_name=zip_file_name,
            sample_size=2,
        )
        sample_images, sample_targets_trans, sample_targets_orient = [], [], []

        for image, target_trans, target_orient in dataset_sample_iterator:
            sample_images.append(image)
            sample_targets_trans.append(target_trans)
            sample_targets_orient.append(target_orient)

        assert len(sample_images) == 2
        assert len(sample_targets_trans) == 2
        assert len(sample_targets_orient) == 2

        for i in range(len(sample_images)):
            for j in range(len(config.dataset.symmetric)):
                target_trans = sample_targets_trans[i][j]
                target_orient = sample_targets_orient[i][j]

                assert len(target_trans) == 3
                assert len(target_orient) == 4
