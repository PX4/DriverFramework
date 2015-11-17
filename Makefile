all: linux qurt

define df_cmake_generate
mkdir -p build_$(1) && cd build_$(1) && cmake .. -DDF_TARGET=$(1)
endef

define df_build
	$(call df_cmake_generate,$(1))
	cd build_$(1) && make
endef

linux nuttx:
	$(call df_build,$@)

external/dspal:
	cd external && git clone https://github.com/mcharleb/dspal

dspal_sync: external/dspal
	cd external/dspal && git pull

qurt: dspal_sync
	$(call df_build,$@)
	
clean:
	rm -rf build_*
