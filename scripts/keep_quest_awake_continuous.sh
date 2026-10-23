#!/usr/bin/env bash
set -uo pipefail

interval_s="${QUEST_KEEP_AWAKE_INTERVAL_S:-2}"
prox_close_action="com.oculus.vrpowermanager.prox_close"
automation_disable_action="com.oculus.vrpowermanager.automation_disable"
was_connected=false

cleanup() {
  trap - EXIT INT TERM
  adb shell am broadcast -a "${automation_disable_action}" >/dev/null 2>&1 || true
  echo "Quest continuous keep-awake stopped."
}
trap cleanup EXIT
trap 'exit 0' INT TERM

echo "Quest continuous keep-awake started; refresh=${interval_s}s."
while true; do
  if [[ "$(adb get-state 2>/dev/null || true)" == "device" ]]; then
    if [[ "${was_connected}" != true ]]; then
      adb shell input keyevent KEYCODE_WAKEUP >/dev/null 2>&1 || true
      echo "Quest connected; wake signal sent."
      was_connected=true
    fi
    adb shell am broadcast -a "${prox_close_action}" >/dev/null 2>&1 || true
  else
    if [[ "${was_connected}" == true ]]; then
      echo "Quest disconnected; waiting for ADB reconnection."
      was_connected=false
    fi
  fi
  sleep "${interval_s}"
done
