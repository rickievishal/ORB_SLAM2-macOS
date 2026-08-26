#!/usr/bin/env bash
set -e

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
if [ ! -f ORBvoc.txt ]; then
    tar -xf ORBvoc.txt.tar.gz
fi
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
