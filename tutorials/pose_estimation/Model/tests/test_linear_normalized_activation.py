from pose_estimation.model import LinearNormalized
import torch
import numpy as np


class TestLinearNormalizedActivation:
    def test_activation_function(self):
        activation_function = LinearNormalized()

        # now we compute the real activation function
        x = torch.tensor([[1.0, 2.0], [2.0, 3.0]])
        true_res = x
        for index in range(x.shape[0]):
            element = true_res[index]
            norm = torch.norm(element, p=2).item()
            element = element / norm
            true_res[index] = element

        res = activation_function._linear_normalized(x)

        assert torch.equal(np.round(res, 4), np.round(true_res, 4))

        true_res = torch.tensor([[0.0, 0.0]])
        res = activation_function._linear_normalized(true_res)
        assert torch.equal(np.round(res, 4), np.round(true_res, 4))
