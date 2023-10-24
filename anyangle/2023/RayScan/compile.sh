#!/bin/bash
idx_dir="index_data"
build_dir="auto_build"

build() (
	cmake "-B${build_dir}" -DCMAKE_BUILD_TYPE=Release "-DCMAKE_CXX_FLAGS=$*"
	cd "${build_dir}"
	cmake --build .
)

mkdir -p ${idx_dir} ${build_dir}
build "$@"
# build exec
cp "${build_dir}/run" .
