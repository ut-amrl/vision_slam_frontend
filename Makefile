include $(shell rospack find mk)/cmake.mk
# Clang is a good compiler to use during development due to its faster compile
# times and more readable output.
C_compiler=/usr/bin/clang
CXX_compiler=/usr/bin/clang++

# GCC is better for release mode due to the speed of its output, and its support
# for OpenMP.
#C_compiler=/usr/bin/gcc
#CXX_compiler=/usr/bin/g++

#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

ifeq ($(build_type),Debug)
	override build_dir=build_debug
else
	build_dir=build
endif
.SILENT:

all: $(build_dir) $(build_dir)/CMakeLists.txt.copy
	$(info $$build_type is [${build_type}])
	$(MAKE) --no-print-directory -C $(build_dir)

# Sets the build type to Debug.
set_debug:
	$(eval build_type=Debug)

# Ensures that the build type is debug before running all target.
debug_all: | set_debug all

clean:
	rm -rf bin lib

$(build_dir)/CMakeLists.txt.copy: CMakeLists.txt Makefile $(build_dir)
	cd $(build_dir) && cmake -DCMAKE_BUILD_TYPE=$(build_type) \
		-DCMAKE_CXX_COMPILER=$(CXX_compiler) \
		-DCMAKE_C_COMPILER=$(C_compiler) ..
	cp CMakeLists.txt $(build_dir)/CMakeLists.txt.copy

$(build_dir):
	mkdir -p $(build_dir)

cleanup_cache:
	rm -rf $(build_dir)
