all: linux qurt rpi edison

define df_cmake_generate
mkdir -p build_$(1) && cd build_$(1) && cmake .. -DOS=$(2) -DCMAKE_TOOLCHAIN_FILE=$(3) -DDF_ENABLE_TESTS=1
endef

rpi linux bebop edison:
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

qurt_load: qurt
	cd build_qurt && make df_imu_test-load

fix-style:
	@./dspal/tools/fix_code_style.sh -p ".git dspal build_qurt build_linux build_rpi build_edison"
check-style:
	@./dspal/tools/fix_code_style.sh -p ".git dspal build_qurt build_linux build_rpi build_edison" --check
