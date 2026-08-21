#!/usr/bin/env bash
# Register iqtaxi_source in SDR++ user config (~/.config/sdrpp/config.json).
#
# Usage:
#   ./register_sdrpp_plugin.sh
#   ./register_sdrpp_plugin.sh /usr/local/lib/sdrpp/plugins/iqtaxi_source.so
#   ./register_sdrpp_plugin.sh --config ~/.config/sdrpp/config.json --plugin /path/to/iqtaxi_source.so
#   ./register_sdrpp_plugin.sh --remove

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IQTAXI_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

CONFIG="${SDRPP_CONFIG:-$HOME/.config/sdrpp/config.json}"
PLUGIN=""
REMOVE=0
INSTANCE_NAME="IQTAXI Source"
MODULE_NAME="iqtaxi_source"

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS] [PLUGIN_SO]

Register or remove the IQTAXI SDR++ source plugin in config.json.

Options:
  -c, --config PATH   SDR++ config.json (default: ~/.config/sdrpp/config.json)
  -p, --plugin PATH   Path to iqtaxi_source.so (auto-detect if omitted)
  --remove            Remove IQTAXI entries from config
  -h, --help          Show this help

Examples:
  $(basename "$0")
  $(basename "$0") /usr/local/lib/sdrpp/plugins/iqtaxi_source.so
  $(basename "$0") --remove

After registration, fully quit and restart SDR++.
EOF
}

resolve_plugin_path() {
    if [[ -n "${PLUGIN}" ]]; then
        echo "${PLUGIN}"
        return
    fi

    local candidates=(
        "/usr/local/lib/sdrpp/plugins/iqtaxi_source.so"
        "/usr/lib/sdrpp/plugins/iqtaxi_source.so"
        "${IQTAXI_ROOT}/build/src/sdr++/iqtaxi_source.so"
        "${IQTAXI_ROOT}/src/sdr++/build/iqtaxi_source.so"
    )

    local path
    for path in "${candidates[@]}"; do
        if [[ -f "${path}" ]]; then
            echo "$(readlink -f "${path}")"
            return
        fi
    done

    echo ""
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--config)
            CONFIG="$2"
            shift 2
            ;;
        -p|--plugin)
            PLUGIN="$2"
            shift 2
            ;;
        --remove)
            REMOVE=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
        *)
            PLUGIN="$1"
            shift
            ;;
    esac
done

if [[ ! -f "${CONFIG}" ]]; then
    echo "error: config not found: ${CONFIG}" >&2
    echo "Start SDR++ once to create it, or pass -c /path/to/config.json" >&2
    exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "error: python3 is required" >&2
    exit 1
fi

BACKUP="${CONFIG}.bak.$(date +%Y%m%d%H%M%S)"
cp -a "${CONFIG}" "${BACKUP}"
echo "backup: ${BACKUP}"

if [[ "${REMOVE}" -eq 1 ]]; then
    python3 - "${CONFIG}" "${INSTANCE_NAME}" "${MODULE_NAME}" <<'PY'
import json
import sys

config_path, instance_name, module_name = sys.argv[1:4]
legacy_names = {instance_name, "IQTAXI E206 Source"}
with open(config_path, encoding="utf-8") as f:
    cfg = json.load(f)

modules = cfg.get("modules") or []
cfg["modules"] = [m for m in modules if "iqtaxi_source" not in m]

instances = cfg.get("moduleInstances") or {}
for name in list(instances):
    if name in legacy_names or instances[name].get("module") == module_name:
        del instances[name]
cfg["moduleInstances"] = instances

with open(config_path, "w", encoding="utf-8") as f:
    json.dump(cfg, f, indent=4)
    f.write("\n")

print("removed IQTAXI entries from", config_path)
PY
    echo "done. restart SDR++."
    exit 0
fi

PLUGIN="$(resolve_plugin_path)"
if [[ -z "${PLUGIN}" || ! -f "${PLUGIN}" ]]; then
    echo "error: iqtaxi_source.so not found." >&2
    echo "Build and install first, or pass the .so path:" >&2
    echo "  sudo cmake --install build" >&2
    echo "  $0 /usr/local/lib/sdrpp/plugins/iqtaxi_source.so" >&2
    exit 1
fi

python3 - "${CONFIG}" "${PLUGIN}" "${INSTANCE_NAME}" "${MODULE_NAME}" <<'PY'
import json
import sys

config_path, plugin_path, instance_name, module_name = sys.argv[1:5]
with open(config_path, encoding="utf-8") as f:
    cfg = json.load(f)

modules = cfg.get("modules")
if not isinstance(modules, list):
    modules = []
if plugin_path not in modules:
    modules.append(plugin_path)
cfg["modules"] = modules

instances = cfg.get("moduleInstances")
if not isinstance(instances, dict):
    instances = {}
if "IQTAXI E206 Source" in instances and instance_name not in instances:
    instances[instance_name] = instances.pop("IQTAXI E206 Source")
if instance_name not in instances:
    instances[instance_name] = {
        "enabled": True,
        "module": module_name,
    }
else:
    instances[instance_name]["enabled"] = True
    instances[instance_name]["module"] = module_name
cfg["moduleInstances"] = instances

with open(config_path, "w", encoding="utf-8") as f:
    json.dump(cfg, f, indent=4)
    f.write("\n")

print("config :", config_path)
print("plugin :", plugin_path)
print("instance:", instance_name)
PY

echo ""
echo "Next steps:"
echo "  1. Fully quit SDR++ and start it again"
echo "  2. Source -> select 'IQTAXI'"
echo "  3. In the source panel, pick Device: E100 / E200 / E206"
echo ""
echo "Plugin config is saved separately as:"
echo "  ${HOME}/.config/sdrpp/iqtaxi_source_config.json"
