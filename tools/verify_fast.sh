#!/usr/bin/env bash
# Hardware-free installed-package gate. Provision dependencies separately.
set -euo pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
interfaces_source="$(realpath "${AI4R_INTERFACES_SOURCE:?Set AI4R_INTERFACES_SOURCE to the pinned dream_interfaces checkout}")"
expected_revision="$(python3 -c 'import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))["repositories"]["dream_interfaces"]["version"])' "$task_root/ci/dependencies.repos")"
[[ "$(git -C "$interfaces_source" rev-parse HEAD)" == "$expected_revision" ]] || { echo 'Wrong dream_interfaces revision' >&2; exit 1; }
git -C "$interfaces_source" diff --quiet HEAD -- || { echo 'Modified dream_interfaces source' >&2; exit 1; }

# Clear inherited overlays/discovery choices so tests neither use a different
# message IDL nor discover an ordinary robot graph. No hardware nodes launch.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=218 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONDONTWRITEBYTECODE=1
exec 9>"${TMPDIR:-/tmp}/ai4r-policy-domain-218.lock"
flock -n 9 || { echo 'Another policy test gate is using domain 218' >&2; exit 1; }

workspace="$task_root/.verification/ws"
mkdir -p "$workspace/src"
export ROS_LOG_DIR="$workspace/log/ros"
touch "$task_root/.verification/COLCON_IGNORE"
for package in dream_interfaces ai4r_policy; do
  source_path="$task_root"
  [[ "$package" != dream_interfaces ]] || source_path="$interfaces_source"
  link="$workspace/src/$package"
  if [[ -e "$link" || -L "$link" ]]; then
    [[ -L "$link" && "$(realpath "$link")" == "$source_path" ]] || { echo "Unexpected source path: $link" >&2; exit 1; }
  else
    ln -s "$source_path" "$link"
  fi
done
[[ "$(find "$workspace/src" -mindepth 1 -maxdepth 1 | wc -l)" -eq 2 ]] || { echo 'Unexpected verification package' >&2; exit 1; }
cd "$workspace"
colcon build --packages-select dream_interfaces ai4r_policy --symlink-install \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
set +u
source install/setup.bash
set -u
colcon test --packages-select ai4r_policy --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
ros2 launch ai4r_policy ai4r_policy.launch.py --show-args
echo 'verify-fast: PASS (synthetic ROS only; no physical hardware)'
