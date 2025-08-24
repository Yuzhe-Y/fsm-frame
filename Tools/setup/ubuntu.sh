#!/bin/bash

set -e

DIR=$(pwd)

# 1. 安装依赖
sudo apt-get update
sudo apt-get install -y cmake g++ python3-dev libblas-dev liblapack-dev wget

# 2. 下载 CasADi 源码
CASADI_VERSION=3.6.5
TAR_FILE=${DIR}/casadi-${CASADI_VERSION}.tar.gz
SRC_DIR=${DIR}/casadi-${CASADI_VERSION}

wget -O ${TAR_FILE} https://github.com/casadi/casadi/archive/refs/tags/${CASADI_VERSION}.tar.gz

# 3. 解压源码
tar -xzf ${TAR_FILE} -C ${DIR}
cd ${SRC_DIR}

# 4. 编译并安装
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install

# 5. 清理
cd "${DIR}"
rm -rf "${SRC_DIR}"
rm -f "${TAR_FILE}"

echo "CasADi ${CASADI_VERSION} 安装完成！"