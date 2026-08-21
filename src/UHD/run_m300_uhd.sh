#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_dir=$(cd -- "${script_dir}/../.." && pwd)
build_dir=${IQTAXI_BUILD_DIR:-"${project_dir}/build"}
module_library="${build_dir}/src/UHD/IQTAXI/libIQTaxiUHD.so"

if [[ ! -f "${module_library}" ]]; then
    echo "M300 IQTAXI UHD module is not built: ${module_library}" >&2
    echo "Run: cmake --build ${build_dir} --target IQTaxiUHD -j\"\$(nproc)\"" >&2
    exit 1
fi

runtime_dir=$(mktemp -d "${TMPDIR:-/tmp}/iqtaxi-uhd-runtime.XXXXXX")
module_dir="${runtime_dir}/uhd/modules"
trap 'rm -rf -- "${runtime_dir}"' EXIT
mkdir -p "${module_dir}"
ln -s "${module_library}" "${module_dir}/libIQTaxiUHD.so"

uhd_soname=$(readelf -d "$(command -v uhd_find_devices)" |
    awk '/\[libuhd\.so/ && !found { line=$0; sub(/^.*\[/, "", line); sub(/\].*$/, "", line); found=line } END { print found }')
uhd_library=$(ldconfig -p |
    awk -v soname="${uhd_soname}" '$1 == soname && !found { found=$NF } END { print found }')
if [[ -z "${uhd_soname}" || -z "${uhd_library}" || ! -f "${uhd_library}" ]]; then
    echo "Unable to locate the UHD runtime library used by uhd_find_devices" >&2
    exit 1
fi
ln -s "${uhd_library}" "${runtime_dir}/${uhd_soname}"

export UHD_MODULE_PATH=""
export UHD_MODULE_D_PATH=""
export LD_LIBRARY_PATH="${runtime_dir}:${build_dir}/src/core:${build_dir}/src/driver:${build_dir}/src/UHD/IQTAXI:${LD_LIBRARY_PATH:-}"

if [[ $# -eq 0 ]]; then
    set -- uhd_usrp_probe --args "type=m300,addr=/dev/xdma0"
fi

"$@"
