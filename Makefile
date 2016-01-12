all: linux qurt

.PHONY submodule:
submodule:
	git submodule init
	git submodule update

define df_cmake_generate
mkdir -p build_$(1) && cd build_$(1) && cmake .. -Wno-dev -DDF_TARGET=$(1) -DCMAKE_TOOLCHAIN_FILE=$(2) -DDF_ENABLE_TESTS=1
endef

linux nuttx:
	$(call df_cmake_generate,$@,cmake/toolchains/Toolchain-$@.cmake)
	cd build_$@ && make

qurt:
	$(call df_cmake_generate,qurt,cmake_hexagon/toolchain/Toolchain-qurt.cmake)
	cd build_qurt && make

run: linux
	build_linux/test/df_testapp

helgrind: linux
	valgrind --tool=helgrind build_linux/test/df_testapp

clean:
	rm -rf build_*
