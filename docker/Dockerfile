FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update \
    && apt upgrade -y \
    && apt install -y \
        g++ \
        clang-9 \
        cmake \
        make \
        wget \
        git \
        libeigen3-dev \
    && mkdir /build \
    && cd build \
    && git clone https://github.com/coin-or/qpOASES.git --depth=1 \
    && mkdir qpOASES/build \
    && cd qpOASES/build \
    && cmake -DCMAKE_CXX_FLAGS_DEBUG="-g -fPIC" -DQPOASES_BUILD_EXAMPLES=OFF .. \
    && make install \
    && cd ../../ \
    && git clone https://github.com/oxfordcontrol/osqp.git --depth=1 \
    && mkdir osqp/build \
    && cd osqp/build \
    && git submodule update --init \
    && cmake .. \
    && make install \
    && cd ../../ \
    && git clone https://github.com/robotology/osqp-eigen.git --depth=1 \
    && mkdir osqp-eigen/build \
    && cd osqp-eigen/build \
    && cmake  .. \
    && make install \
    && cd ../../ \
    && git clone https://github.com/rbdl/rbdl.git --depth=1 \
    && mkdir rbdl/build \
    && cd rbdl/build \
    && cmake -DRBDL_BUILD_ADDON_URDFREADER=ON .. \
    && make install \
    && cd ../../ \
    && cd / \
    && rm -rf /build \
    && git clone https://github.com/NaokiTakahashi12/ik-performance-test.git --depth=1 \
    && mkdir ik-performance-test/build \
    && cd ik-performance-test/build \
    && cmake .. \
    && make \
    && apt remove -y \
        g++ \
        clang-9 \
        cmake \
        make \
        wget \
        git \
        libeigen3-dev \
    && apt clean \
    && apt autoremove -y \
    && rm -rf /var/lib/apt/lists/*
