import os 
import io

from setuptools import setup

# Package meta-data.
NAME = "pose_estimation"
DESCRIPTION = "Perform pose estimation on a single cube environment"
EMAIL = "perception@unity3d.com"
AUTHOR = "Unity Perception"
REQUIRES_PYTHON = ">=3.8"
VERSION = "0.1.0"

here = os.path.abspath(os.path.dirname(__file__))

# Load the package's __version__.py module as a dictionary.
about = {}
if not VERSION:
    project_slug = NAME.lower().replace("-", "_").replace(" ", "_")
    with open(os.path.join(here, project_slug, "__version__.py")) as f:
        exec(f.read(), about)
else:
    about["__version__"] = VERSION

setup(
    name = NAME,
    description = DESCRIPTION,
    author = AUTHOR,
    author_email=EMAIL,
    version = VERSION,
    packages = ['pose_estimation'],
    include_package_data=True,
    entry_points = {'console_scripts': [f"{NAME}={NAME}.cli:main"]},
    classifiers=[
        # Trove classifiers
        # Full list: https://pypi.python.org/pypi?%3Aaction=list_classifiers
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
    ],
)
