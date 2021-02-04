import os
from pathlib import Path
import zipfile
import json
import glob

import numpy as np

import torch
import torchvision

from PIL import Image
import random

from pose_estimation.storage.gcs import GCSClient

import logging

logger = logging.getLogger(__name__)


class SingleCubeDataset(torch.utils.data.IterableDataset):
    """
    This class is made to create a SingleCubeDataset object which is an
    object corresponding to the dataset we want to use to feed our
    deep learning model. We need to create the object as an
    Iterable Dataset object for two main reasons:
        - using an iterator enables to save memory problem. We don't
        have to stock in memory the whole dataset at each iteration
        of the model. The only useful data for a given iteration is
        the batch size we are dealing with
        - in pytorch, to feed a machine learning model, you need to
        create a DataLoader object. The DataLoader object takes
        as input a Dataset object or an IterableDataset object.
        Nevertheless, you can't use an iterator with the Dataset
        object

    Attributes:
        config: configuration of the model
        data_root (str): root path towards the data file
        split (str): specify which type of dataset we
        have access to
        zip_file_name (str): name of the data zip file
        sample_size (int): size of the batch dataset we want to create.
            This attribute is used if you want to test your model on a very
            small amount of data. It is a subset of the whole dataset
    """

    def __init__(
        self,
        *,
        config,
        data_root="/tmp",
        split="train",
        zip_file_name="single_cube_training",
        sample_size=0,
        **kwargs,
    ):
        self.config = config

        self.data_root = data_root
        self.zip_file_name = zip_file_name
        self.root = os.path.join(data_root, self.zip_file_name)

        if self.config.dataset.download_data_gcp:
            self.pose_estimation_gcs_path = self.config.dataset.pose_estimation_gcs_path
            self.gcs_bucket = self.config.dataset.gcs_bucket
            self._download()

        self.size = self.__len__()
        self.sample_size = min(sample_size, self.size)
        if self.sample_size > 0:
            self.random_indices = self._generate_random_indices()

    def pre_processing(self, element_iterator):
        """
        Do some pre-processing of the data in order to feed to the
        neural network

        Args:
            element_iterator: iterator on the data files

        Returns:
            a tuple of three element which are:
                - an image tensor
                - a tensor vector for the 3 cube's center
                coordinates (translation)
                - a the tensor vector for the 4 quaternion elements
                (orientation)
        """
        position_list, image_name = element_iterator

        translation = list(position_list[0]["translation"].values())
        orientation = list(position_list[0]["rotation"].values())
        translation = torch.tensor(translation, dtype=torch.float)
        orientation = torch.tensor(orientation, dtype=torch.float)

        image_origin = Image.open(image_name).convert("RGB")
        transform = self.get_transform()
        image = transform(image_origin).unsqueeze(0)

        return image, translation, orientation

    def __iter__(self):
        """
        Mandatory method in order to create an IterableDataset object.

        Returns:
            an iterator over the data after the pre-processing.
            It can return a batch of the data if we specify a value
            for the "sample_size" attribute
        """
        iterator = RawDataIterator(path=self.root)

        # Map each element
        mapped_itr = map(self.pre_processing, iterator)

        if self.sample_size > 0:
            mapped_itr = self._sample(mapped_itr)

        return mapped_itr

    def __len__(self):
        """
        Method to have the number of rows of the dataset

        Returns:
            (int): length of the dataset
        """
        for folder in os.listdir(self.root):
            if folder.startswith("RGB"):
                return len(os.listdir(os.path.join(self.root, folder)))

    def _sample(self, iterator):
        """
        Method to create an iterator over a batch of a dataset

        Args:
            iterator: an iterator over a dataset

        Returns:
            an iterator over the batch dataset
        """
        result = []
        for index, element in enumerate(iterator):
            if index in self.random_indices:
                result.append(element)
        return iter(result)

    def _generate_random_indices(self):
        """
        Method to generate a list of int between 0 and self.size and
        of length self.sample_size

        Returns:
            list of int which corresponds to a list of indexes
        """
        indices = random.sample(range(0, self.size), self.sample_size)
        return indices

    def _get_local_data_zip(self):
        """
        Create a local path for download zip file

        Returns:
            a str path toward the local data zip file
        """
        return os.path.join(self.root, f"{self.zip_file_name}.zip")

    def _download(self):
        """
        Method to download dataset from GCS
        """
        path = Path(self.root)
        path.mkdir(parents=True, exist_ok=True)
        client = GCSClient()
        object_key = os.path.join(
            self.pose_estimation_gcs_path, f"{self.zip_file_name}.zip"
        )

        data_zip_local = self._get_local_data_zip()
        if not os.path.exists(data_zip_local):
            logger.info(f"no data zip file found, will download.")
            client.download(
                bucket_name=self.gcs_bucket,
                object_key=object_key,
                localfile=data_zip_local,
            )
            with zipfile.ZipFile(data_zip_local, "r") as zip_dir:
                zip_dir.extractall(f"{self.root}")

    def get_transform(self):
        """
        Apply a transform on the input image tensor

        Returns:
            https://pytorch.org/docs/stable/torchvision/transforms.html
        """
        transform = torchvision.transforms.Compose(
            [
                torchvision.transforms.Resize(
                    (self.config.dataset.image_scale, self.config.dataset.image_scale,)
                ),
                torchvision.transforms.ToTensor(),
            ]
        )
        return transform


class RawDataIterator:
    """
    This class is made to create a RawDataIterator object which is
    an object corresponding to an iterator over a folder to
    extract the data you want.

    Attributes:
        log_index (int): index of the log file
        line_index (int): index of the line in the json file
        base_path (str): path towards the data folder which
            contains the Logs and the ScreenCapture folders
        log_folder_path (str): path towards the Dataset folder in your saved data
    """

    def __init__(self, path):
        self.log_index = 0
        self.image_index = 0
        self.base_path = path
        for file in os.listdir(path):
            if file.startswith("Dataset"):
                self.log_folder_path = os.path.join(path, file)

    def __iter__(self):
        """
        Mandatory method in order to create an Iterator object.
        """
        return self

    def __next__(self):
        """
        Mandatory method in order to create an Iterator object.

        Returns:
            a tuple corresponding to the data as a dictionnary and the
            path towards the corresponding images
        """
        path = self._log_path()
        if os.path.exists(path):
            file = open(path)
            data = json.load(file)
            if self.image_index >= len(data["captures"]):
                # move to next log file
                self.log_index += 1
                self.image_index = 0
                return self.__next__()
            else:
                # get results
                if self._fetch_results(data):
                    data_list, image_path = self._fetch_results(data)
                    self.image_index += 1
                    return (data_list, image_path)
                self.image_index += 1
                return self.__next__()
        raise StopIteration

    # HELPERS

    def _fetch_results(self, data):
        """
        Extract the result for a given line of a file

        Attribute:
            data (json object): https://www.geeksforgeeks.org/json-load-in-python/
                it is the data file
            
            Example:
                "filename": "RGB33aa3f79-f2dc-47a3-b631-fb80dcacdb29/rgb_2.png",
                "format": "PNG",
                "annotations": [
                    {
                    "id": "c6294393-a40c-4466-8cab-9c79c799c436",
                    "annotation_definition": "0bfbe00d-00fa-4555-88d1-471b58449f5c",
                    "values": [
                        {
                "label_id": 1,
                "label_name": "cube_position",
                "instance_id": 1,
                "translation": {
                    "x": -0.34494638442993164,
                    "y": -0.10846114158630371,
                    "z": 1.7134368419647217
                },
                "size": {
                    "x": 0.099999994039535522,
                    "y": 0.099999994039535522,
                    "z": 0.099999994039535522
                },
                "rotation": {
                    "x": -0.11946356296539307,
                    "y": 0.714719831943512,
                    "z": -0.12602438032627106,
                    "w": 0.67751163244247437
                }

        Returns:
            a tuple corresponding to the data as a list of dictionary and the
            path towards the corresponding images
        """

        data_list = []
        data_capture = data["captures"][self.image_index]
        values = data_capture["annotations"][0]["values"]
        if len(values) > 0:
            for classes in data_capture["annotations"][0]["values"]:
                data_list.append(classes)

            image_local_path = data_capture["filename"]
            image_path = os.path.join(self.base_path, image_local_path)
            return (data_list, image_path)

    def _log_path(self):
        """
        Give the path towards the Logs file inside the data folder

        Return:
            path (str): path towards the Logs file
        """
        if self.log_index < 10:
            path = "{}/captures_00{}.json".format(self.log_folder_path, self.log_index)
        elif self.log_index < 100:
            path = "{}/captures_0{}.json".format(self.log_folder_path, self.log_index)
        else:
            path = "{}/captures_{}.json".format(self.log_folder_path, self.log_index)
        return path
