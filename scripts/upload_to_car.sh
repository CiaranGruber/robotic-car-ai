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
EXCLUDE_FROM="$ROOT_PATH/files_not_to_upload.txt"

# Terminal colours for script status logs (distinct from rsync/ssh output).
COLOUR_BLUE=$'\033[1;34m'
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

ssh_to_car() {
    # Run a command on the car over SSH.
    # Uses a login shell so PATH and tools such as dream match an interactive SSH.
    #
    # Args:
    #   remote_command: Command to run on the car.
    # -l: login (profile), -i: interactive so .bashrc does not return early.
    ssh -p "$REMOTE_PORT" "$REMOTE_USER@$REMOTE_HOST" bash -lic "$(printf '%q' "$1")"
}

copy_files_to_car() {
    # Copy the repository to the car with rsync, excluding paths in files_not_to_upload.txt.
    #
    # Args:
    #   dry_run: If non-empty, list what would be transferred without writing.
    local dry_run="${1:-}"
    local dry_run_flag=()
    if [[ -n "$dry_run" ]]; then
        dry_run_flag=(--dry-run)
    fi

    # The ${array[@]+...} form expands an empty array to nothing; macOS's bash 3.2 otherwise
    # rejects "${array[@]}" as unbound under set -u.
    rsync -av \
        ${dry_run_flag[@]+"${dry_run_flag[@]}"} \
        -e "ssh -p $REMOTE_PORT" \
        --exclude-from="$EXCLUDE_FROM" \
        "$ROOT_PATH/" "$REMOTE_USER@$REMOTE_HOST:$SRC_PATH"
}

usage() {
    cat <<EOF
Usage: $(basename "$0") <command>

Upload this package to car ${CAR_NUM} (${REMOTE_USER}@${REMOTE_HOST}).

Commands:
  dry_copy          List files that would be uploaded (rsync --dry-run)
  upload            Upload files to ${SRC_PATH}
  upload_and_build  Upload files, then run: dream build ros student
  help              Show this help

Paths in files_not_to_upload.txt are excluded from the upload. Each SSH/rsync
step may prompt for the password separately.
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
    log "Uploading files to ${REMOTE_USER}@${REMOTE_HOST}:${SRC_PATH}"
    copy_files_to_car
    log "Upload complete."
}

upload_and_build() {
    # Upload the repository to the car, then build the student workspace.
    log "Uploading files to ${REMOTE_USER}@${REMOTE_HOST}:${SRC_PATH}"
    copy_files_to_car
    log "Upload complete."
    log "Starting build process (running 'dream build ros student' while ssh'd to the car)..."
    ssh_to_car "dream build ros student"
    log "Build complete."
}

# --------------------
# Main
# --------------------

main() {
    local command="${1:-}"
    case "$command" in
        dry_copy|upload|upload_and_build)
            "$command"
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
