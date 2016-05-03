all: linux qurt rpi2

define df_cmake_generate
mkdir -p build_$(1) && cd build_$(1) && cmake .. -DOS=$(2) -DCMAKE_TOOLCHAIN_FILE=$(3) -DDF_ENABLE_TESTS=1
endef

rpi2 linux:
	$(call df_cmake_generate,$@,posix,cmake/toolchains/Toolchain-$@.cmake)
	cd build_$@ && make

qurt:
	$(call df_cmake_generate,$@,$@,cmake/cmake_hexagon/toolchain/Toolchain-$@.cmake)
	cd build_$@ && make

run: linux
	build_linux/test/df_testapp

helgrind: linux
	valgrind --tool=helgrind build_linux/test/df_testapp

clean:
	rm -rf build_*

fix-style:
	./dspal/tools/fix_code_style.sh -p ".git dspal build_qurt build_linux build_rpi2"
