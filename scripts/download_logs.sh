#!/usr/bin/env bash

# Download the policy node's drive logs (policy/data_logging) from the car, for analysis or to make real
# scenarios (docs/ai_rules/3.02_creating_real_scenario_data.md). It only reads from the car.

set -euo pipefail

# --------------------
# Constants
# --------------------

# Workspace (repository) root: parent of this script's directory.
ROOT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Car number (1–20); used to form the host address. By default the car car_management.sh uploads to.
CAR_NUM="$(sed -n 's/^CAR_NUM=\([0-9][0-9]*\).*/\1/p' "$ROOT_PATH/scripts/car_management.sh")"

# SSH connection constants for the car.
REMOTE_PORT=22
REMOTE_USER="ai4r"
# logging.logging_folder in config/ai4r_policy.yaml.
LOG_PATH="/home/ai4r/policy_logs"
# One SSH connection is shared by every step, so the password is asked for once. Kept short: macOS limits socket
# paths to about 100 characters.
CONTROL_PATH="/tmp/ai4r-logs-%C"

# Local folder for the logs. It is gitignored, and scripts/data_converter.py reads the pairs from it.
DEST_PATH="$ROOT_PATH/scenarios/data/original_data"

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

fail() {
    # Print an error in red to stderr and stop.
    #
    # Args:
    #   message: Error text to show.
    echo -e "${COLOUR_RED}$*${COLOUR_RESET}" >&2
    exit 1
}

ssh_options() {
    # Print the SSH options shared by ssh and rsync: the port and the shared connection.
    printf '%s ' -p "$REMOTE_PORT" -o ControlMaster=auto -o "ControlPath=$CONTROL_PATH" -o ControlPersist=120
}

on_car() {
    # Run a command on the car over the shared SSH connection, without a terminal.
    #
    # Args:
    #   remote_command: Command to run on the car, with its arguments.
    # shellcheck disable=SC2046
    ssh $(ssh_options) "$REMOTE_USER@$REMOTE_HOST" "$@"
}

close_connection() {
    # Close the shared SSH connection, if one was opened.
    # shellcheck disable=SC2046
    ssh $(ssh_options) -O exit "$REMOTE_USER@$REMOTE_HOST" 2>/dev/null || true
}

check_name() {
    # Stop unless a run name or prefix only has the characters the logger uses (it is used on the car's shell).
    #
    # Args:
    #   name: Run name (stem, e.g. 2026-10-19_13-09-36) or its start.
    if [[ ! "$1" =~ ^[A-Za-z0-9_.-]+$ ]]; then
        fail "Not a log run name: '$1' (letters, digits, '_', '-' and '.' only, e.g. 2026-10-19_13-09-36)"
    fi
}

check_count() {
    # Stop unless a count is a whole number of at least 1.
    #
    # Args:
    #   count: The count given.
    if [[ ! "$1" =~ ^[1-9][0-9]*$ ]]; then
        fail "Not a count: '$1' (a whole number of at least 1)"
    fi
}

runs_on_car() {
    # Print the runs on the car whose names start with a prefix, newest first, one name per line.
    # A run is the files <name>_observations.json, _actions.json, _config.json and _memory.json.
    #
    # Args:
    #   prefix: Start of the run names; empty for all runs.
    #   count: Most runs to print; empty for all.
    local prefix="$1"
    local count="${2:-}"
    local newest=""
    # Cut the list on the car, so ssh is not stopped part way through its output
    if [[ -n "$count" ]]; then
        newest=" | head -n $count"
    fi
    on_car "cd $LOG_PATH && ls -1t -- ${prefix}*_observations.json 2>/dev/null${newest} \
        | sed 's/_observations\\.json\$//'"
}

download_runs() {
    # Download every file of each run into DEST_PATH. rsync skips files already downloaded.
    #
    # Args:
    #   names: The runs' names.
    local name
    local file
    local files
    mkdir -p "$DEST_PATH"
    for name in "$@"; do
        check_name "$name"
        files="$(on_car "ls -1 -- $LOG_PATH/${name}_*.json 2>/dev/null" || true)"
        if [[ -z "$files" ]]; then
            fail "No run named ${name} in ${LOG_PATH} on car ${CAR_NUM}"
        fi
        log "Downloading ${name}"
        while IFS= read -r file; do
            rsync -av -e "ssh $(ssh_options)" "$REMOTE_USER@$REMOTE_HOST:$file" "$DEST_PATH/"
        done <<< "$files"
    done
    log "Downloaded $# run(s) into ${DEST_PATH}:"
    for name in "$@"; do
        ls -lh "$DEST_PATH/${name}"_*.json >&2
    done
    log "To make scenarios from them: python -m scripts.data_converter sync (docs/ai_rules/3.02)"
}

usage() {
    cat <<EOF
Usage: $(basename "$0") [--car <num>] [--dest <folder>] <command>

Download drive logs from car ${CAR_NUM} (${REMOTE_USER}@10.43.254.$((13 + CAR_NUM)):${LOG_PATH})
into ${DEST_PATH}.

Commands:
  list [count]      List the newest runs on the car (default 10), with their size
  latest [count]    Download the newest runs (default 1)
  run <name>...     Download the named runs, e.g. run 2026-10-19_13-09-36
  match <prefix>    Download every run whose name starts with <prefix>,
                    e.g. match 2026-10-19_13 (all runs from 13:00 to 13:59)
  help              Show this help

Options:
  --car <num>       Car number (default: CAR_NUM in car_management.sh)
  --dest <folder>   Local folder for the logs (default: scenarios/data/original_data,
                    which is gitignored)

Each run is <name>_observations.json, _actions.json, _config.json and _memory.json.
SSH asks for the car's password once; the connection is reused for the rest.
Files already downloaded are skipped. Nothing on the car is changed.
EOF
}

# --------------------
# Commands
# --------------------

list() {
    # List the newest runs on the car with their total size.
    #
    # Args:
    #   count: Most runs to list (default 10).
    local count="${1:-10}"
    check_count "$count"
    log "Newest ${count} run(s) on car ${CAR_NUM} (${LOG_PATH}):"
    on_car "cd $LOG_PATH && ls -1t -- *_observations.json 2>/dev/null | head -n $count | while read -r f; do \
        name=\${f%_observations.json}; \
        printf '%s  %6s  %s\n' \"\$name\" \"\$(du -ch -- \"\$name\"_*.json | tail -n 1 | cut -f1)\" \
            \"\$(date -r \"\$f\" '+%H:%M:%S')\"; \
        done"
}

latest() {
    # Download the newest runs.
    #
    # Args:
    #   count: How many (default 1).
    local count="${1:-1}"
    local names=()
    local name
    check_count "$count"
    while IFS= read -r name; do
        [[ -n "$name" ]] && names+=("$name")
    done <<< "$(runs_on_car "" "$count")"
    if [[ ${#names[@]} -eq 0 ]]; then
        fail "No runs in ${LOG_PATH} on car ${CAR_NUM}"
    fi
    download_runs "${names[@]}"
}

match() {
    # Download every run whose name starts with a prefix.
    #
    # Args:
    #   prefix: Start of the run names.
    local prefix="${1:-}"
    local names=()
    local name
    if [[ -z "$prefix" ]]; then
        fail "match needs a prefix, e.g. match 2026-10-19_13"
    fi
    check_name "$prefix"
    while IFS= read -r name; do
        [[ -n "$name" ]] && names+=("$name")
    done <<< "$(runs_on_car "$prefix")"
    if [[ ${#names[@]} -eq 0 ]]; then
        fail "No runs starting with ${prefix} in ${LOG_PATH} on car ${CAR_NUM}"
    fi
    download_runs "${names[@]}"
}

# --------------------
# Main
# --------------------

main() {
    local positional=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --car)
                [[ $# -ge 2 ]] || fail "--car needs a car number"
                [[ "$2" =~ ^([1-9]|1[0-9]|20)$ ]] || fail "Not a car number (1–20): '$2'"
                CAR_NUM="$2"
                shift 2
                ;;
            --dest)
                [[ $# -ge 2 && -n "$2" ]] || fail "--dest needs a folder"
                DEST_PATH="$2"
                shift 2
                ;;
            *)
                positional+=("$1")
                shift
                ;;
        esac
    done
    set -- ${positional[@]+"${positional[@]}"}
    [[ -n "$CAR_NUM" ]] || fail "No CAR_NUM in scripts/car_management.sh; pass --car <num>"
    REMOTE_HOST="10.43.254.$((13 + CAR_NUM))"

    local command="${1:-}"
    case "$command" in
        list|latest|match)
            shift
            trap close_connection EXIT
            "$command" "$@"
            ;;
        run)
            shift
            [[ $# -ge 1 ]] || fail "run needs at least one run name, e.g. run 2026-10-19_13-09-36"
            trap close_connection EXIT
            download_runs "$@"
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
