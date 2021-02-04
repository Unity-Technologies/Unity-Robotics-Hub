FROM nvidia/cuda:10.1-cudnn7-runtime-ubuntu18.04

# Add Miniconda
# https://github.com/ContinuumIO/docker-images/blob/master/miniconda3/debian/Dockerfile
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH /opt/conda/bin:$PATH

RUN apt-get update --fix-missing && \
    apt-get install -y wget bzip2 ca-certificates libglib2.0-0 libxext6 libsm6 libxrender1 git mercurial subversion && \
    apt-get clean

RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda && \
    rm ~/miniconda.sh && \
    /opt/conda/bin/conda clean -tipsy && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc && \
    echo "conda activate base" >> ~/.bashrc && \
    find /opt/conda/ -follow -type f -name '*.a' -delete && \
    find /opt/conda/ -follow -type f -name '*.js.map' -delete && \
    /opt/conda/bin/conda clean -afy

# Add Tini init systems to handle orphaned processes
# https://cloud.google.com/solutions/best-practices-for-building-containers#problem_2_how_classic_init_systems_handle_orphaned_processes
ENV TINI_VERSION v0.19.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /usr/local/bin/tini
RUN chmod +x /usr/local/bin/tini

# Add project stuff
WORKDIR /
VOLUME /notebooks

# This will install all dependencies
COPY environment-gpu.yml ./
RUN conda env update -n base -f environment-gpu.yml && \
    conda clean -afy

# COPY the rest of code here
COPY . ./

# pip install this package
RUN pip install -e .

# pre-load VGG weights
RUN python -c 'import pose_estimation.model as model; model.preload()'

# Use -g to ensure all child process received SIGKILL
ENTRYPOINT ["tini", "-g", "--"]

CMD sh -c "jupyter notebook --notebook-dir=/ --ip=0.0.0.0 --no-browser --allow-root --port=8888 --NotebookApp.token='' --NotebookApp.password='' --NotebookApp.allow_origin='*' --NotebookApp.base_url=${NB_PREFIX}"
