#!/bin/bash

# cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CUDA_FLAGS="-gencode=arch=compute_87,code=sm_87" ../SurroundView/ll

# projcmake.sh - Build script for SurroundView on Jetson Orin Nano

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Building SurroundView for Jetson Orin Nano ===${NC}"

# Check if ASSIMP is installed
if ! dpkg -l | grep -q libassimp-dev; then
    echo -e "${RED}ASSIMP not installed!${NC}"
    echo -e "${YELLOW}Installing ASSIMP...${NC}"
    sudo apt update
    sudo apt install libassimp-dev -y
fi

# Auto-detect ASSIMP path
ASSIMP_CONFIG=$(find /usr -name "assimp-config.cmake" 2>/dev/null | head -n 1)
if [ -n "$ASSIMP_CONFIG" ]; then
    ASSIMP_DIR=$(dirname "$ASSIMP_CONFIG")
    echo -e "${GREEN}Found ASSIMP at: ${ASSIMP_DIR}${NC}"
    CMAKE_ASSIMP_FLAG="-D assimp_DIR=${ASSIMP_DIR}"
else
    echo -e "${YELLOW}ASSIMP path not found, using default search${NC}"
    CMAKE_ASSIMP_FLAG=""
fi

# Clean build directory
echo -e "${YELLOW}Cleaning build directory...${NC}"
rm -rf build
mkdir -p build
cd build

# CMake configuration
echo -e "${YELLOW}Running CMake configuration...${NC}"
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_CUDA_ARCHITECTURES=87 \
      -D CMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
      ${CMAKE_ASSIMP_FLAG} \
      ..

# Check if cmake succeeded
if [ $? -ne 0 ]; then
    echo -e "${RED}CMake configuration failed!${NC}"
    echo -e "${YELLOW}Trying to find ASSIMP path manually...${NC}"
    find /usr -name "*assimp*.cmake" 2>/dev/null
    exit 1
fi

# Build
echo -e "${YELLOW}Building project...${NC}"
make -j4

# Check if build succeeded
if [ $? -eq 0 ]; then
    echo -e "${GREEN}Build successful!${NC}"
    echo -e "${GREEN}Executable: $(pwd)/SurroundView${NC}"
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi
