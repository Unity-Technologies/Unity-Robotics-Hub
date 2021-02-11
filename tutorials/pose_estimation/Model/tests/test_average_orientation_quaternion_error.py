from pose_estimation.evaluation_metrics.orientation_average_quaternion_error import (
    orientation_average_quaternion_error,
)
import torch
import numpy as np


class TestAverageQuaternionError:
    def get_y_true_y_pred(self):
        y_true = torch.tensor([[1.0, 2.0], [2.0, 3.0]])
        y_pred = torch.tensor([[2.0, 1.0], [1.0, 2.0]])
        return y_true, y_pred

    def test_metric(self):

        y_true, y_pred = self.get_y_true_y_pred()
        # we need to normalize the vectors
        norm_y_pred = torch.norm(y_pred, dim=1).unsqueeze(0)
        norm_y_pred = torch.transpose(norm_y_pred, 1, 0)
        y_pred = torch.div(y_pred, norm_y_pred)

        norm_y_true = torch.norm(y_true, dim=1).unsqueeze(0)
        norm_y_true = torch.transpose(norm_y_true, 1, 0)
        y_true = torch.div(y_true, norm_y_true)

        # compute real metric
        true_res = 0
        shape = y_pred.shape

        for i in range(shape[0]):
            error_sample = 0
            for j in range(shape[1]):
                error_sample += (y_pred[i, j] * y_true[i, j]).item()
            error_sample = 2 * (error_sample ** 2) - 1

            if error_sample >= 0.999:
                error_sample = 0
            if error_sample <= -0.999:
                error_sample = np.pi / 2
            else:
                error_sample = np.arccos(error_sample)
            true_res += error_sample

        true_res = true_res / shape[0]

        # Update metric
        res = orientation_average_quaternion_error(y_true, y_true)
        assert 0 == res

        res = orientation_average_quaternion_error(
            y_true, torch.tensor([[0.0, 0.0], [0.0, 0.0]])
        )
        assert np.round(np.pi / 2, 4) == np.round(res, 4)

        res = orientation_average_quaternion_error(y_pred, y_true)
        assert np.round(true_res, 4) == np.round(res, 4)
