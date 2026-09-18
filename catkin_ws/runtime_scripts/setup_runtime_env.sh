#!/usr/bin/env bash

# Build one combined catkin_isolated runtime environment. This file is meant
# to be sourced by the other scripts in this directory.

_tron_runtime_is_sourced() {
  [[ "${BASH_SOURCE[0]}" != "$0" ]]
}

_tron_runtime_source_file() {
  local nounset_was_on=0
  [[ "$-" == *u* ]] && nounset_was_on=1
  set +u
  source "$1"
  ((nounset_was_on)) && set -u
}

_tron_runtime_prepend_unique() {
  local variable_name="$1"
  local value="$2"
  local current_value="${!variable_name:-}"
  [[ -n "$value" ]] || return 0
  case ":$current_value:" in
    *":$value:"*) return 0 ;;
  esac
  if [[ -n "$current_value" ]]; then
    export "$variable_name=$value:$current_value"
  else
    export "$variable_name=$value"
  fi
}

_tron_runtime_main() {
  local ros_setup="/opt/ros/noetic/setup.bash"
  local script_dir inferred_ws selected_ws candidate pkg local_setup marker source_entry
  local missing=0
  local -a workspace_candidates package_order required_packages extra_workspaces package_sources
  local -A sourced_setups

  if [[ ! -f "$ros_setup" ]]; then
    echo "[ERROR] ROS setup not found: $ros_setup" >&2
    return 2
  fi

  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  inferred_ws="$(cd "$script_dir/.." && pwd)"

  if [[ -n "${TRON_WS:-}" ]]; then
    if [[ ! -d "$TRON_WS/devel_isolated" ]]; then
      echo "[ERROR] TRON_WS has no devel_isolated directory: $TRON_WS" >&2
      return 2
    fi
    selected_ws="$(cd "$TRON_WS" && pwd)"
  else
    workspace_candidates=(
      "$inferred_ws"
      /root/catkin_ws
      /home/guest/catkin_ws
    )
    selected_ws=""
    for candidate in "${workspace_candidates[@]}"; do
      if [[ -d "$candidate/devel_isolated" ]]; then
        selected_ws="$(cd "$candidate" && pwd)"
        break
      fi
    done
    if [[ -z "$selected_ws" ]]; then
      echo "[ERROR] cannot locate GPS-TRON catkin workspace" >&2
      echo "        Set TRON_WS=/absolute/path/to/catkin_ws" >&2
      return 2
    fi
  fi
  export TRON_WS="$selected_ws"

  _tron_runtime_source_file "$ros_setup"

  extra_workspaces=("$TRON_WS")
  [[ -n "${LIVOX_WS:-}" ]] && extra_workspaces+=("$LIVOX_WS")
  [[ -n "${FASTLIO_WS:-}" ]] && extra_workspaces+=("$FASTLIO_WS")
  extra_workspaces+=(
    /home/guest/catkin_ws
    /root/catkin_ws
    /home/tron/livox_ws
    /home/tron/catkin_ws
  )

  package_order=(
    livox_ros_driver2
    fast_lio
    fast_lio_localization_sc_qn
    phone_gps_bridge
    tron_open_space_nav
    tron_sensor_bridge
    tron_local_collision_safety
  )

  for pkg in "${package_order[@]}"; do
    local_setup=""
    for candidate in "${extra_workspaces[@]}"; do
      [[ -d "$candidate" ]] || continue
      if [[ -f "$candidate/devel_isolated/$pkg/local_setup.bash" ]]; then
        local_setup="$candidate/devel_isolated/$pkg/local_setup.bash"
        break
      fi
      if [[ -f "$candidate/install_isolated/$pkg/local_setup.bash" ]]; then
        local_setup="$candidate/install_isolated/$pkg/local_setup.bash"
        break
      fi
      if [[ -f "$candidate/devel/local_setup.bash" ]] \
          && find "$candidate/src" -maxdepth 4 -type d -name "$pkg" -print -quit 2>/dev/null \
             | grep -q .; then
        local_setup="$candidate/devel/local_setup.bash"
        break
      fi
    done

    if [[ -z "$local_setup" ]]; then
      echo "[WARN] local setup not found for optional package: $pkg" >&2
      continue
    fi
    if [[ -n "${sourced_setups[$local_setup]:-}" ]]; then
      continue
    fi

    _tron_runtime_source_file "$local_setup"
    sourced_setups[$local_setup]=1

    # catkin_isolated local_setup updates binary/runtime prefixes but these
    # workspaces do not install a ROS_PACKAGE_PATH hook. Merge the source
    # spaces recorded by catkin's marker so rospack can see every package.
    marker="$(dirname "$local_setup")/.catkin"
    if [[ -f "$marker" ]]; then
      while IFS= read -r source_entry || [[ -n "$source_entry" ]]; do
        [[ -d "$source_entry" ]] || continue
        package_sources+=("$source_entry")
      done < <(tr ';' '\n' < "$marker")
    fi
    echo "[OK] loaded local environment: $pkg ($local_setup)"
  done

  # Each generated local_setup can normalize ROS_PACKAGE_PATH, so apply all
  # collected source spaces once, after the final local environment is loaded.
  for source_entry in "${package_sources[@]}"; do
    _tron_runtime_prepend_unique ROS_PACKAGE_PATH "$source_entry"
  done

  rospack profile >/dev/null 2>&1 || true

  required_packages=(
    phone_gps_bridge
    tron_open_space_nav
    tron_sensor_bridge
    tron_local_collision_safety
  )
  for pkg in "${required_packages[@]}"; do
    if rospack find "$pkg" >/dev/null 2>&1; then
      echo "[OK] package visible: $pkg"
    else
      echo "[ERROR] required package is not visible: $pkg" >&2
      missing=1
    fi
  done

  if ((missing)); then
    echo "[ERROR] runtime environment is incomplete" >&2
    return 3
  fi

  export TRON_RUNTIME_ENV_READY=1
  echo "[OK] combined runtime environment ready: $TRON_WS"
  return 0
}

if _tron_runtime_main "$@"; then
  _tron_runtime_status=0
else
  _tron_runtime_status=$?
fi
if _tron_runtime_is_sourced; then
  return "$_tron_runtime_status"
fi
exit "$_tron_runtime_status"
