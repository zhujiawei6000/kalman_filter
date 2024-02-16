conan install . --build=missing --settings=build_type=Debug
# conan install . --build=missing --settings=build_type=Release
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE="generators/conan_toolchain.cmake"
cmake --build . --config Debug
