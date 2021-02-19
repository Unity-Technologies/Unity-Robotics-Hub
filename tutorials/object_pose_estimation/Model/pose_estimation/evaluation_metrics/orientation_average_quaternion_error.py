r"""Orientation Average Quaternion error metric.

The orientation average quaternion error is the mean of the quaternion loss over a batch of examples.
It is used as a metric for the orientation's prediction. 
In this project, the cube is only rotating around the y-axis so we can define the quaternion loss
as the angle between the orientation of the prediction and the orientation of the target: 
angle = arccos(2 * (<quaternion_output, quaternion_target>**2) - 1)

Where <.> denotes the scalar product between two vectors and it is define as the distance between two
quaternions. 
They are three cases:
- cos(angle) is above 0.999 (arbitrary threeshold but needs to be very close to 1) then the loss is 0
- cos(angle) is below -0.999 (arbitrary threeshold but needs to be very close to -1), then the loss is pi/2
- cos(angle) is betwwen -0.999 and 0.999 then the loss is equal to the angle

In this project, I only use 2 out of the 4 coordinates of the quaternion because as there are only rotations
around the y-axis, the coordinates q_x and q_z are equal to 0 in the configuration (q_w, q_x, q_y, q_z)

"""
import numpy as np

import torch


def orientation_average_quaternion_error(output, target):
    """
    Attributes:
        output (tensor of shape (batch size, 2)): it is the model's prediction
        for all the images in a batch
        target (tensor of shape (batch_size, 2)): it is the target
    Return:
        metric translation is a scalar
    """
    if target.shape != output.shape:
        raise ValueError(
            f"The shapes of real data {target.shape} and predicted data {output.shape} \
                           should be the same."
        )

    batch_size = target.shape[0]

    metric_orientation = 0
    for index in range(batch_size):
        distance_quaternion = torch.dot(target[index], output[index])
        cos_angle = 2 * torch.mul(distance_quaternion, distance_quaternion).item() - 1

        if cos_angle <= -0.999:
            metric_orientation += np.pi / 2
        elif -0.999 < cos_angle < 0.999:
            metric_orientation += np.arccos(cos_angle)

    metric_orientation = metric_orientation / batch_size

    return metric_orientation
