FROM ubuntu:22.04

RUN apt update
RUN apt install -y \
    build-essential \
    cmake \
    git \
    googletest \
    googletest-tools \
    lcov \
    libarmadillo-dev \
    libgmock-dev \
    libgtest-dev \
    python3-all

RUN mkdir /src
RUN git clone --branch main https://github.com/marek-cel/libmcutils.git /src
RUN cd /src; cmake . -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_TESTING=Off -B build
RUN cd /src; cmake --build build --config Release -j 4
RUN cd /src; cmake --build build --config Release --target install
RUN rm -r /src
RUN ldconfig

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME
RUN useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

USER $USERNAME
