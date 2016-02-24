all: linux qurt rpi2

.PHONY update:
update:
	git submodule update --init --recursive

define df_cmake_generate
mkdir -p build_$(1) && cd build_$(1) && cmake .. -DDF_TARGET=$(1) -DCMAKE_TOOLCHAIN_FILE=$(2) -DDF_ENABLE_TESTS=1
endef

rpi2 linux nuttx: update
	$(call df_cmake_generate,$@,cmake/toolchains/Toolchain-$@.cmake)
	cd build_$@ && make

qurt: update
	$(call df_cmake_generate,qurt,cmake/cmake_hexagon/toolchain/Toolchain-qurt.cmake)
	cd build_qurt && make

run: linux
	build_linux/test/df_testapp

helgrind: linux
	valgrind --tool=helgrind build_linux/test/df_testapp

clean:
	rm -rf build_*

fix-style:
	./dspal/tools/fix_code_style.sh -p ".git dspal build_qurt build_linux build_nuttx"
