#!/usr/bin/env bash

# Resolve one supported BEITONG A2P3A joystick and export the matching
# Linux-js Mode-2 axis layout. This file is sourced by the public launchers.

beitong_profile_for_device() {
  local device="$1"
  local properties
  if ! properties="$(udevadm info --query=property --name="$device" 2>/dev/null)"; then
    return 1
  fi

  if grep -qx 'ID_VENDOR_ID=20bc' <<<"$properties" &&
    grep -qx 'ID_MODEL_ID=511c' <<<"$properties"; then
    printf '%s\n' bfm
    return 0
  fi

  if grep -qx 'ID_VENDOR_ID=045e' <<<"$properties" &&
    grep -qx 'ID_MODEL_ID=028e' <<<"$properties" &&
    grep -qx 'ID_VENDOR=BEITONG' <<<"$properties" &&
    grep -qx 'ID_MODEL=BEITONG_A2P3A_XINPUT_DONGLE' <<<"$properties"; then
    printf '%s\n' xinput
    return 0
  fi

  return 1
}

resolve_beitong_joystick() {
  local requested_device="${JOYSTICK_DEVICE:-}"
  local requested_profile="${JOYSTICK_PROFILE:-}"
  local resolved_device detected_profile candidate candidate_profile
  local -a matches=()
  local -a profiles=()

  requested_profile="${requested_profile#"${requested_profile%%[![:space:]]*}"}"
  requested_profile="${requested_profile%"${requested_profile##*[![:space:]]}"}"
  requested_profile="${requested_profile,,}"

  if ! command -v udevadm >/dev/null 2>&1; then
    echo "udevadm is required to identify the BEITONG joystick." >&2
    return 1
  fi

  if [[ -n "$requested_device" ]]; then
    if [[ ! -e "$requested_device" ]]; then
      echo "JOYSTICK_DEVICE does not exist: $requested_device" >&2
      return 1
    fi
    resolved_device="$(readlink -f -- "$requested_device")"
    if ! detected_profile="$(beitong_profile_for_device "$resolved_device")"; then
      echo "Unsupported joystick identity: $requested_device" >&2
      echo "Expected BEITONG A2P3A BFM 20bc:511c or BEITONG XInput 045e:028e." >&2
      return 1
    fi
  else
    shopt -s nullglob
    for candidate in /dev/input/js*; do
      if candidate_profile="$(beitong_profile_for_device "$candidate")"; then
        matches+=("$candidate")
        profiles+=("$candidate_profile")
      fi
    done
    shopt -u nullglob

    if (( ${#matches[@]} == 0 )); then
      echo "No supported BEITONG A2P3A joystick was found under /dev/input/js*." >&2
      echo "Expected BFM 20bc:511c or BEITONG XInput 045e:028e." >&2
      return 1
    fi
    if (( ${#matches[@]} > 1 )); then
      echo "Multiple supported BEITONG joysticks were found; set JOYSTICK_DEVICE explicitly:" >&2
      printf '  %s\n' "${matches[@]}" >&2
      return 1
    fi
    resolved_device="${matches[0]}"
    detected_profile="${profiles[0]}"
  fi

  if [[ ! "$resolved_device" =~ ^/dev/input/js[0-9]+$ ]]; then
    echo "Unexpected joystick node: $resolved_device" >&2
    return 1
  fi
  if [[ ! -c "$resolved_device" || ! -r "$resolved_device" ]]; then
    echo "Joystick is not a readable character device: $resolved_device" >&2
    ls -l "$resolved_device" >&2 2>/dev/null || true
    return 1
  fi
  if [[ -n "$requested_profile" && "$requested_profile" != "$detected_profile" ]]; then
    echo "JOYSTICK_PROFILE=$requested_profile conflicts with detected profile=$detected_profile." >&2
    return 1
  fi

  JOYSTICK_DEVICE="$resolved_device"
  JOYSTICK_PROFILE="$detected_profile"
  JOYSTICK_AXIS_YAW="${JOYSTICK_AXIS_YAW:-0}"
  JOYSTICK_AXIS_THROTTLE="${JOYSTICK_AXIS_THROTTLE:-1}"
  JOYSTICK_SIGN_YAW="${JOYSTICK_SIGN_YAW:--1}"
  JOYSTICK_SIGN_THROTTLE="${JOYSTICK_SIGN_THROTTLE:--1}"
  JOYSTICK_SIGN_ROLL="${JOYSTICK_SIGN_ROLL:-1}"
  JOYSTICK_SIGN_PITCH="${JOYSTICK_SIGN_PITCH:--1}"

  case "$JOYSTICK_PROFILE" in
    bfm)
      JOYSTICK_AXIS_ROLL="${JOYSTICK_AXIS_ROLL:-2}"
      JOYSTICK_AXIS_PITCH="${JOYSTICK_AXIS_PITCH:-3}"
      ;;
    xinput)
      JOYSTICK_AXIS_ROLL="${JOYSTICK_AXIS_ROLL:-3}"
      JOYSTICK_AXIS_PITCH="${JOYSTICK_AXIS_PITCH:-4}"
      ;;
    *)
      echo "Internal error: unsupported BEITONG profile $JOYSTICK_PROFILE" >&2
      return 1
      ;;
  esac

  export JOYSTICK_DEVICE JOYSTICK_PROFILE
  export JOYSTICK_AXIS_YAW JOYSTICK_AXIS_THROTTLE
  export JOYSTICK_AXIS_ROLL JOYSTICK_AXIS_PITCH
  export JOYSTICK_SIGN_YAW JOYSTICK_SIGN_THROTTLE
  export JOYSTICK_SIGN_ROLL JOYSTICK_SIGN_PITCH

  printf '%s\n' \
    "[JOYSTICK] selected device=$JOYSTICK_DEVICE profile=$JOYSTICK_PROFILE axes(yaw/throttle/roll/pitch)=$JOYSTICK_AXIS_YAW/$JOYSTICK_AXIS_THROTTLE/$JOYSTICK_AXIS_ROLL/$JOYSTICK_AXIS_PITCH"
}
