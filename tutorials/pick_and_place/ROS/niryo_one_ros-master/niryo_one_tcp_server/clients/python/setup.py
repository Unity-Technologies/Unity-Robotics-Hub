from setuptools import setup

setup(name='niryo_one_tcp_client',
      version='0.1',
      description='A tcp client python that manages the tcp communication with a Niryo One robot',
      author='Corentin Ducatez',
      author_email='c.ducatez@niryo.com',
      license='GPLv3',
      packages=['niryo_one_tcp_client'],
      zip_safe=False, install_requires=['rospkg', 'PyYAML'])
