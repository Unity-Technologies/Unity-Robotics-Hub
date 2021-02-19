r"""Translation Average Mean Square Error metrics.

The translation average mean square error is the mean of the mse loss over a batch of examples.
It is used as a metric for the translation's prediction
"""

import torch


def translation_average_mean_square_error(output, target):
    """
    Attributes:
        output (tensor of shape (batch size, 3)): it is the model's prediction
        for all the images in a batch
        target (tensor of shape (batch_size, 3)): it is the target
    Return:
        metric translation is a scalar
    """

    if target.shape != output.shape:
        raise ValueError(
            f"The shapes of real data {target.shape} and predicted data {output.shape} \
                            should be the same."
        )

    square_diffs = (target - output) ** 2
    norms = torch.mean(square_diffs, 1)
    metric_translation = torch.mean(norms, axis=-1).item()

    return metric_translation
