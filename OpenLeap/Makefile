#include $(shell rospack find mk)/cmake.mk
.PHONY: clean download

BUILD_TYPE?=Release
CMAKE=cmake
CMAKE_ARGS=-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
           -DCMAKE_INSTALL_PREFIX=`rospack find OpenLeap`/OpenLeap

all:
	mkdir -p build
	@git clone git://github.com/nuclearmistake/OpenLeap.git build/git || (echo "INFO: skipping clone"; true)
	cd build/git && git pull && make libopenleap.so
	mkdir -p lib
	mkdir -p include
	cp -R build/git/include/* include
	cp build/git/libopenleap.so lib

clean:
	rm -rf build/build lib_leap
