import torch
import torchvision


def preload():
    '''pre-load VGG model weights, for transfer learning. Automatically cached for later use.'''
    torchvision.models.vgg16(pretrained=True)


class PoseEstimationNetwork(torch.nn.Module):
    """
    PoseEstimationNetwork: Neural network based on the VGG16 neural network
    architecture developed by Tobin at al. (https://arxiv.org/pdf/1703.06907.pdf). 
    The model is a little bit different from the original one
    but we still import the model as it has already been trained on a huge
    dataset (ImageNet) and even if we change a bit its architecture, the main
    body of it is unchanged and the weights of the final model will not be too
    far from the original one. We call this method "transfer learning".
    The network is composed by two branches: one for the translation
    (prediction of a 3 dimensional vector corresponding to x, y, z coordinates) and
    one for the orientation (prediction of a 4 dimensional vector corresponding to
    a quaternion)
    """

    def __init__(self, *, is_symetric):
        super(PoseEstimationNetwork, self).__init__()
        self.is_symetric = is_symetric
        self.model_backbone = torchvision.models.vgg16(pretrained=True) # uses cache
        # remove the original classifier
        self.model_backbone.classifier = torch.nn.Identity()

        self.translation_block = torch.nn.Sequential(
            torch.nn.Linear(25088, 256),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(256, 64),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(64, 3),
        )
        self.orientation_block = torch.nn.Sequential(
            torch.nn.Linear(25088, 256),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(256, 64),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(64, 4),
            LinearNormalized(),
        )

    def forward(self, x):
        x = self.model_backbone(x)
        output_translation = self.translation_block(x)

        if self.is_symetric == False:
            output_orientation = self.orientation_block(x)
            return output_translation, output_orientation

        return output_translation


class LinearNormalized(torch.nn.Module):
    """
    Custom activation function which normalizes the input.
    It will be used to normalized the output of the orientation
    branch in our model because a quaternion vector is a
    normalized vector
    """

    def __init__(self):
        super(LinearNormalized, self).__init__()

    def forward(self, x):
        return self._linear_normalized(x)

    def _linear_normalized(self, x):
        """
        Activation function which normalizes an input
        It will be used in the orientation network because
        a quaternion is a normalized vector.
        Args:
            x (pytorch tensor with shape (batch_size, 4)): the input of the model
        Returns:
            a pytorch tensor normalized vector with shape(batch_size, 4)
        """
        norm = torch.norm(x, p=2, dim=1).unsqueeze(0)
        for index in range(norm.shape[1]):
            if norm[0, index].item() == 0.0:
                norm[0, index] = 1.0
        x = torch.transpose(x, 0, 1)
        x = torch.div(x, norm)
        return torch.transpose(x, 0, 1)

