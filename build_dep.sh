# place all dependencies here
export DEPENDENCIES_DIR=/mnt/DATA/SDK/
mkdir -p ${DEPENDENCIES_DIR}
cd ${DEPENDENCIES_DIR}

# Boost
sudo apt-get install libboost-dev

# YAML
sudo apt-get install libyaml-cpp-dev

# g2o
cd ${DEPENDENCIES_DIR}
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/g2o" 
make -j
sudo make install

# build opencv 3.4
cd ${DEPENDENCIES_DIR}
wget https://github.com/opencv/opencv/archive/3.4.1.tar.gz
tar xf 3.4.1.tar.gz
wget https://github.com/opencv/opencv_contrib/archive/3.4.1.tar.gz
tar xf 3.4.1.tar.gz.1
cd ${DEPENDENCIES_DIR}/opencv-3.4.1
mkdir build
cd build
# build opencv without cuda support
# cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/opencv3" -DBUILD_TBB:BOOL="1" -DWITH_TBB:BOOL="1" -DCMAKE_BUILD_TYPE:STRING="Release" -DWITH_OPENMP:BOOL="1"  -DBUILD_opencv_gpu:BOOL="0" -DOPENCV_EXTRA_MODULES_PATH:PATH="/mnt/DATA/SDK/opencv_contrib-3.4.1/modules" -DBUILD_opencv_cudaobjdetect:BOOL="0" -DWITH_CUFFT:BOOL="0" -DBUILD_opencv_cudaimgproc:BOOL="0" -DBUILD_opencv_cudastereo:BOOL="0" -DBUILD_opencv_cudaoptflow:BOOL="0" -DBUILD_opencv_cudabgsegm:BOOL="0" -DBUILD_opencv_cudaarithm:BOOL="0" -DWITH_CUDA:BOOL="0" -DOPENCV_ENABLE_NONFREE:BOOL="1" -DBUILD_opencv_cudacodec:BOOL="0" -DWITH_CUBLAS:BOOL="0" -DBUILD_opencv_cudawarping:BOOL="0" -DBUILD_opencv_cudafilters:BOOL="0" -DCUDA_64_BIT_DEVICE_CODE:BOOL="0" -DBUILD_opencv_cudafeatures2d:BOOL="0" -DBUILD_opencv_cudalegacy:BOOL="0" 
# build opencv with cuda support
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/opencv3" -DBUILD_TBB:BOOL="1" -DWITH_TBB:BOOL="1" -DCMAKE_BUILD_TYPE:STRING="Release" -DWITH_OPENMP:BOOL="1"  -DBUILD_opencv_gpu:BOOL="1" -DOPENCV_EXTRA_MODULES_PATH:PATH=${DEPENDENCIES_DIR}/opencv_contrib-3.4.1/modules -DBUILD_opencv_cudaobjdetect:BOOL="1" -DWITH_CUFFT:BOOL="1" -DBUILD_opencv_cudaimgproc:BOOL="1" -DBUILD_opencv_cudastereo:BOOL="1" -DBUILD_opencv_cudaoptflow:BOOL="1" -DBUILD_opencv_cudabgsegm:BOOL="1" -DBUILD_opencv_cudaarithm:BOOL="1" -DWITH_CUDA:BOOL="1" -DOPENCV_ENABLE_NONFREE:BOOL="1" -DBUILD_opencv_cudacodec:BOOL="1" -DWITH_CUBLAS:BOOL="1" -DBUILD_opencv_cudawarping:BOOL="1" -DBUILD_opencv_cudafilters:BOOL="1" -DCUDA_64_BIT_DEVICE_CODE:BOOL="1" -DBUILD_opencv_cudafeatures2d:BOOL="1" -DBUILD_opencv_cudalegacy:BOOL="1" 
make -j
sudo make install

# MRPT (viz only)
cd ${DEPENDENCIES_DIR}
git clone https://github.com/MRPT/mrpt.git
cd mrpt
git checkout 0c3d605c3cbf5f2ffb8137089e43ebdae5a55de3
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/mrpt" 
make -j
sudo make install