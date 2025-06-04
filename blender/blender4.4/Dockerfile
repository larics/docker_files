# Use Ubuntu 24.04 as the base image
FROM ubuntu:24.04

# Set environment variables to non-interactive to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

ENV BLENDER_VERSION=4.4.3
ENV PYTHON_VERSION=3.11
# Used if scripts need a Python version string without dots, e.g., for naming or paths
ENV PYTHON_VERSION_NODOT=311

# Set up a working directory early
WORKDIR /opt

# 1. Install essential prerequisites & common tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # Core build tools
    build-essential \
    cmake \
    pkg-config \
    # Source control and fetching
    git \
    git-lfs \
    subversion \
    wget \
    ca-certificates \
    # Python 3 pip (system python3 will be used for initial scripts)
    python3-pip \
    # X11 development libraries (critical for GUI)
    libx11-dev \
    libxext-dev \
    libxi-dev \
    libxrender-dev \
    libxfixes-dev \
    libxxf86vm-dev \
    libsm-dev \
    # Wayland development libraries (for Wayland GHOST backend)
    libwayland-dev \
    wayland-protocols \
    libxkbcommon-dev \
    libegl1-mesa-dev \
    # PipeWire development library (for audio)
    libpipewire-0.3-dev \
    # Keep zlib1g-dev as it's super fundamental and small
    zlib1g-dev \
    # Keep Vulkan runtime loader if any Vulkan features are on
    libvulkan1 \
    # Fontconfig for system font discovery
    fontconfig \
    libfontconfig1-dev && \
    rm -rf /var/lib/apt/lists/*

# Ensure we use bash for RUN commands from this point onwards
SHELL ["/bin/bash", "-c"]

# Step 2 (Deadsnakes PPA) is removed.

# 3. Download Blender source code and libraries
ENV BLENDER_REPO_URL=https://projects.blender.org/blender/blender.git
ENV BLENDER_TAG=v${BLENDER_VERSION}
ENV BLENDER_LIBS_PYTHON_PATH="/opt/blender_source/lib/linux_x64/python/bin/python${PYTHON_VERSION}"
RUN echo "Cloning Blender main repository (tag ${BLENDER_TAG})..." && \
    git clone --branch ${BLENDER_TAG} --depth 1 ${BLENDER_REPO_URL} blender_source && \
    cd blender_source && \
    echo "Initializing and synchronizing Git submodules..." && \
    git submodule init && \
    git submodule sync --recursive && \
    echo "Ensuring 'update = checkout' for lib/linux_x64 submodule..." && \
    git config --local submodule.lib/linux_x64.update checkout && \
    # If you need test data, uncomment the next line:
    # git config --local submodule.tests/data.update checkout && \
    echo "Attempting to update all Git submodules..." && \
    git submodule update --init --recursive --depth 1 --jobs $(nproc --all || echo 1) --force --progress && \
    echo "Running git lfs pull..." && \
    git lfs pull && \
    echo "Running 'python3 build_files/utils/make_update.py --use-linux-libraries'..." && \
    python3 build_files/utils/make_update.py --use-linux-libraries --git-command "$(which git)" && \
    echo "make_update.py and submodule setup completed." && \
    echo "Verifying lib/linux_x64 and its Python installation..." && \
    if [ -d "lib/linux_x64" ] && [ -f "lib/linux_x64/.git" ] && [ -n "$(ls -A lib/linux_x64)" ]; then \
        echo "SUCCESS: lib/linux_x64 is populated."; \
        if [ -x "${BLENDER_LIBS_PYTHON_PATH}" ]; then \
            echo "SUCCESS: Python ${PYTHON_VERSION} found in Blender libraries at ${BLENDER_LIBS_PYTHON_PATH}"; \
            ${BLENDER_LIBS_PYTHON_PATH} --version; \
        else \
            echo "ERROR: Python ${PYTHON_VERSION} NOT FOUND or not executable in Blender libraries at ${BLENDER_LIBS_PYTHON_PATH}" >&2; \
            ls -la "/opt/blender_source/lib/linux_x64/python/bin/" || echo "Python bin dir not found in libs"; \
            exit 1; \
        fi; \
    else \
        echo "ERROR: lib/linux_x64 is NOT correctly populated after all steps!" >&2; \
        exit 1; \
    fi && \
    cd ..

# 4. Install remaining system dependencies via Blender's script
RUN cd blender_source && \
    echo "Attempting to run install_linux_packages.py with system python3" && \
    python3 ./build_files/build_environment/install_linux_packages.py --all --skip-questions && \
    apt-get clean && rm -rf /var/lib/apt/lists/* || true

# 5. Create Python virtual environment using Blender's bundled Python
RUN echo "Creating Python virtual environment for Blender addons using Python from Blender's libraries..." && \
    ${BLENDER_LIBS_PYTHON_PATH} -m venv /opt/blender_venv
ENV PATH="/opt/blender_venv/bin:${PATH}"

# Upgrade pip in venv
RUN echo "Upgrading pip in Blender's Python venv..." && \
    pip install --upgrade pip
    # If you decide you DO need numpy/requests for addons, add them back here:
    # && pip install numpy requests

# Set locale
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 6. Build Blender
RUN echo "------ Starting Blender Build ------" && \
    export VENV_SITE_PACKAGES="/opt/blender_venv/lib/python${PYTHON_VERSION}/site-packages" && \
    export PYTHONPATH="${VENV_SITE_PACKAGES}:${PYTHONPATH}" && \
    echo "PYTHONPATH for build: $PYTHONPATH" && \
    \
    mkdir -p /opt/blender_build && cd /opt/blender_build && \
    cmake ../blender_source \
    -DCMAKE_INSTALL_PREFIX=/opt/blender-${BLENDER_VERSION} \
    -DWITH_INSTALL_PORTABLE=ON \
    -DCMAKE_BUILD_TYPE=Release \
    # Add your other -D WITH_FEATURE=OFF flags here if needed.
    \
    CORES=$(nproc --all || echo 1) && \
    # make -j${CORES} && \
    # make -j${CORES} install
    make -j8 && \
    make -j8 install

# 7. Set up environment for running Blender
# The PATH for the blender installation directory itself is set.
ENV PATH="/opt/blender-${BLENDER_VERSION}:${PATH}"
ENV __NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0 
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# 8. Clean up
RUN rm -rf /opt/blender_build /opt/blender_source

# Default command to run Blender
WORKDIR /root
CMD /opt/blender-$BLENDER_VERSION/blender

# docker build -t blender4.4:latest .
# docker run -it --network host --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --privileged -e DISPLAY=$DISPLAY -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix --volume $HOME/.Xauthority:/root/.Xauthority:ro --env XAUTHORITY=/root/.Xauthority blender4.4:latest