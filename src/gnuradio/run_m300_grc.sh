#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_dir=$(cd -- "${script_dir}/../.." && pwd)
build_dir=${IQTAXI_BUILD_DIR:-"${project_dir}/build"}

required_library="${build_dir}/src/gnuradio/libgnuradio-iqtaxi.so"
if [[ ! -f "${required_library}" ]]; then
    echo "M300 IQTAXI GNU Radio library is not built: ${required_library}" >&2
    echo "Run: cmake --build ${build_dir} -j\"\$(nproc)\"" >&2
    exit 1
fi

export LD_LIBRARY_PATH="${build_dir}/src/core:${build_dir}/src/driver:${build_dir}/src/gnuradio:${LD_LIBRARY_PATH:-}"
export GRC_BLOCKS_PATH="${project_dir}/src/gnuradio/grc:${GRC_BLOCKS_PATH:-}"

exec gnuradio-companion "$@"
