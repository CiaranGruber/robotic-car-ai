#!/usr/bin/env bash

set -euo pipefail

# --------------------
# Constants
# --------------------

# Car number (1–20); used to form the host address.
CAR_NUM=20

# SSH connection constants for the car.
REMOTE_PORT=22
REMOTE_USER="ai4r"
REMOTE_PASSWORD="ai4r"
REMOTE_HOST="10.43.254.$((13 + CAR_NUM))"
SRC_PATH="~/ai4r_student_workspace/src/ai4r_policy/"
# Workspace (repository) root: parent of this script's directory.
ROOT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXCLUDE_FROM="$ROOT_PATH/car_upload_filter.txt"

# Terminal colours for script status logs (distinct from rsync/ssh output).
COLOUR_BLUE=$'\033[1;34m'
COLOUR_RED=$'\033[1;31m'
COLOUR_RESET=$'\033[0m'

# --------------------
# Helpers
# --------------------

log() {
    # Print a status message in blue to stderr so it stands out from command output.
    #
    # Args:
    #   message: Status text to show.
    echo -e "${COLOUR_BLUE}==> $*${COLOUR_RESET}" >&2
}

# Set when a step is interrupted (Ctrl+C); used with trap INT in run_step.
STEP_INTERRUPTED=0

ask_yes_no() {
    # Ask a yes/no question on stderr.
    #
    # Args:
    #   prompt: Question text shown before the reply.
    #
    # Returns:
    #   0 for y/yes/…; 1 otherwise (including empty / N).
    local prompt="$1"
    local reply
    read -r -p "$prompt" reply || true
    case "$(printf '%s' "$reply" | tr '[:upper:]' '[:lower:]')" in
        y|yes|yeah|yep)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

run_step() {
    # Run a named step. On Ctrl+C (or rsync SIGINT exit 20), ask whether to
    # cancel all remaining operations or skip this step and continue.
    #
    # Args:
    #   label: Short name for logs and the prompt.
    #   command...: Command and arguments to run.
    local label="$1"
    shift
    local status=0

    STEP_INTERRUPTED=0
    # Keep the shell alive on SIGINT so we can prompt; the child still stops.
    trap 'STEP_INTERRUPTED=1' INT
    set +e
    "$@"
    status=$?
    set -e
    trap - INT

    if [[ "$STEP_INTERRUPTED" -eq 1 || "$status" -eq 130 || "$status" -eq 20 ]]; then
        log "Interrupted during ${label}."
        echo >&2
        if ask_yes_no "Cancel all remaining operations? [y/N] "; then
            log "Cancelling remaining operations."
            exit 130
        fi
        log "Skipping ${label}; continuing with remaining operations."
        return 0
    fi
    if [[ "$status" -ne 0 ]]; then
        log "${label} failed (exit ${status})."
        exit "$status"
    fi
}

ssh_to_car() {
    # Run a command on the car over SSH.
    # Uses a login shell so PATH and tools such as dream match an interactive SSH.
    #
    # Args:
    #   remote_command: Command to run on the car.
    # -l: login (profile), -i: interactive so .bashrc does not return early.
    ssh -tt -p "$REMOTE_PORT" "$REMOTE_USER@$REMOTE_HOST" bash -lic "$(printf '%q' "$1")"
}

copy_files_to_car() {
    # Copy the repository to the car with rsync, using car_upload_filter.txt.
    #
    # Args:
    #   dry_run: If non-empty, list what would be transferred without writing.
    local dry_run="${1:-}"
    local dry_run_flag=()
    if [[ -n "$dry_run" ]]; then
        dry_run_flag=(--dry-run)
    fi

    rsync -av \
        ${dry_run_flag[@]+"${dry_run_flag[@]}"} \
        -e "ssh -p $REMOTE_PORT" \
        --filter="merge ${EXCLUDE_FROM}" \
        "$ROOT_PATH/" "$REMOTE_USER@$REMOTE_HOST:$SRC_PATH"
}

usage() {
    cat <<EOF
Usage: $(basename "$0") <command>

Upload this package to car ${CAR_NUM} (${REMOTE_USER}@${REMOTE_HOST}).

Commands:
  dry_copy          List files that would be uploaded (rsync --dry-run)
  upload            Upload files to ${SRC_PATH}
  upload_and_build [--debug-images <type>]
                    Upload files, run: dream build ros student, then
                    start_runtime (options are passed through).
                    Ctrl+C during a step asks whether to cancel the rest
                    or skip that step and continue.
  start_runtime [--debug-images <type>]
                    Restart the usual dream runtime services on the car,
                    including the IMU, lidar and GPU cone detector. Stop an active
                    CPU cone detector first; do not fall back to CPU.
                    With --debug-images, oakd_gpu_cone_detector is restarted with
                    that flag (dream runtime restart oakd_gpu_cone_detector
                    --debug-images <type>).
  help              Show this help

Paths in car_upload_filter.txt are rsync filter rules for the upload. Each
SSH/rsync step may prompt for the password separately.
EOF
}

# --------------------
# Commands
# --------------------

dry_copy() {
    # List what would be uploaded to the car, without writing.
    log "Dry-run: listing files that would be uploaded to ${REMOTE_USER}@${REMOTE_HOST}:${SRC_PATH}"
    copy_files_to_car dry-run
    log "Dry-run complete."
}

upload() {
    # Upload the repository to the car.
    log "You are uploading to ${COLOUR_RED}car ${CAR_NUM}"
    if ! ask_yes_no "Continue with upload? [y/N] "; then
        log "Upload cancelled."
        exit 1
    fi
    log "Uploading files to ${REMOTE_USER}@${REMOTE_HOST}:${SRC_PATH}"
    copy_files_to_car
    log "Upload complete."
}

build_on_car() {
    # Build the student workspace on the car over SSH.
    log "Starting build process (running 'dream build ros student' while ssh'd to the car)..."
    ssh_to_car "dream build ros student"
    log "Build complete."
}

upload_and_build() {
    # Upload the repository to the car, build the student workspace, then restart
    # runtime services. Ctrl+C during a step asks whether to cancel the rest or
    # skip that step and continue.
    #
    # Args:
    #   optional flags for start_runtime, e.g. --debug-images <type>.
    run_step "upload" upload
    run_step "build" build_on_car
    run_step "start_runtime" start_runtime "$@"
}

start_runtime() {
    # Start or restart the usual dream runtime services on the car.
    #
    # Args:
    #   optional --debug-images <type>: pass through to oakd_gpu_cone_detector only.
    # Intersection navigation requires fresh IMU yaw rate and Cartesian lidar;
    # include both services after a reboot that interrupted them.
    local services=(
        foxglove_bridge
        traxxas_vehicle_interface
        rplidar_c1
        bno08x_imu_interface
        oakd_gpu_cone_detector
        aruco_detector
        ai4r_policy
    )
    local debug_images_type=""
    local service
    local remote_cmd="set -e; set -o pipefail; "
    local restart_cmd

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --debug-images)
                if [[ $# -lt 2 || -z "${2:-}" || "$2" == -* ]]; then
                    echo "start_runtime: --debug-images requires a <type> value" >&2
                    usage >&2
                    exit 1
                fi
                debug_images_type="$2"
                shift 2
                ;;
            *)
                echo "start_runtime: unknown option: $1" >&2
                usage >&2
                exit 1
                ;;
        esac
    done

    log "Restarting dream runtime services on ${REMOTE_USER}@${REMOTE_HOST}..."
    # UNIT names whose STATE is running (column 1 / 2 of `dream runtime status`).
    remote_cmd+='running_services=$(dream runtime status | awk '\''$2 == "running" { print $1 }'\''); '
    # Both detectors use the same camera. This car requires the GPU component.
    remote_cmd+='if printf "%s\n" "$running_services" | grep -qx oakd_cone_detector; then dream runtime stop oakd_cone_detector; fi; '
    for service in "${services[@]}"; do
        if [[ "$service" == "foxglove_bridge" ]]; then
            remote_cmd+='if ! printf "%s\n" "$running_services" | grep -qx foxglove_bridge; then dream runtime restart foxglove_bridge; fi; '
            continue
        fi
        restart_cmd="dream runtime restart ${service}"
        if [[ "$service" == "oakd_gpu_cone_detector" && -n "$debug_images_type" ]]; then
            restart_cmd+=$(printf ' --debug-images %q' "$debug_images_type")
        fi
        if [[ "$service" == "rplidar_c1" ]]; then
            # An interrupted unit (left over from an earlier boot) may need start rather than restart
            restart_cmd+=" || dream runtime start ${service}"
        fi
        remote_cmd+="${restart_cmd}; "
    done
    # One SSH session: status check, then restart each service in order.
    ssh_to_car "$remote_cmd"
    log "Runtime restart complete."
}

# --------------------
# Main
# --------------------

main() {
    local command="${1:-}"
    case "$command" in
        dry_copy|upload)
            "$command"
            ;;
        upload_and_build|start_runtime)
            shift
            "$command" "$@"
            ;;
        help|"")
            usage
            ;;
        *)
            echo "Unknown command: $command" >&2
            echo >&2
            usage >&2
            exit 1
            ;;
    esac
}

main "$@"
