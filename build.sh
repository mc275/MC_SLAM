mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ..
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.bin ./Examples/Monocular/KITTI00-02.yaml /media/mc/Study/DataSets/KITTI/00/

