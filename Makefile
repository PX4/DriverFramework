all: build

.PHONY: build
generate:
	mkdir -p build
	cd build && cmake ..

build: generate
	cd build && make

run: build
	cd build && test/df_testapp
	
clean:
	rm -rf build
