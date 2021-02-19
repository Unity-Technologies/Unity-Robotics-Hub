from pose_estimation.evaluation_metrics.translation_average_mean_square_error import (
    translation_average_mean_square_error,
)
import torch
import numpy as np


class TestAverageMeanSquaredError:
    def get_y_true_y_pred(self):
        y_true = torch.tensor([[2.0, 1.0, 1.0], [1, 2, 2]])
        y_pred = torch.tensor([[1.0, 2.0, 3.0], [2, 3, 4]])
        return y_true, y_pred

    def test_metric(self):
        y_true, y_pred = self.get_y_true_y_pred()

        # compute real metric
        true_res = 0
        shape = y_pred.shape

        for i in range(shape[0]):
            error_sample = 0
            for j in range(shape[1]):
                error_sample += torch.mul(
                    y_pred[i, j] - y_true[i, j], y_pred[i, j] - y_true[i, j]
                ).item()
            true_res += error_sample / shape[1]
        true_res = true_res / shape[0]

        res = translation_average_mean_square_error(y_true, y_true)
        assert 0 == res

        res = translation_average_mean_square_error(y_pred, y_true)
        assert np.round(true_res, 4) == np.round(res, 4)
