#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROBE_TOPIC="/__eye_turtle_topic_origin_probe"
PROBE_PID=""

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
set -u

cleanup() {
  if [[ -n "${PROBE_PID}" ]]; then
    kill "${PROBE_PID}" 2>/dev/null || true
    wait "${PROBE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

gid_prefix() {
  awk -F. '{print $1"."$2"."$3"."$4}' <<<"$1"
}

gid_pid() {
  local b4 b5
  IFS=. read -r _ _ _ _ b4 b5 _ <<<"$1"
  [[ -n "${b4:-}" && -n "${b5:-}" ]] || return 1
  printf '%d' "$((16#${b4} + (16#${b5} << 8)))"
}

endpoint_rows() {
  awk -F': ' '
    /^Node name:/ {name=$2}
    /^Node namespace:/ {namespace=$2}
    /^Topic type:/ {type=$2}
    /^Endpoint type:/ {endpoint=$2}
    /^GID:/ {
      node=(namespace=="/" ? "/" name : namespace "/" name)
      print endpoint "\t" node "\t" type "\t" $2
    }
  ' <<<"$1"
}

echo "正在建立本机 DDS 探针..."
ros2 topic pub -r 2 "${PROBE_TOPIC}" std_msgs/msg/Empty '{}' >/dev/null 2>&1 &
PROBE_PID=$!

LOCAL_GID=""
for _ in {1..20}; do
  PROBE_INFO="$(ros2 topic info "${PROBE_TOPIC}" -v 2>/dev/null || true)"
  LOCAL_GID="$(awk -F': ' '/^GID:/ {print $2; exit}' <<<"${PROBE_INFO}")"
  [[ -n "${LOCAL_GID}" ]] && break
  sleep 0.1
done

if [[ -z "${LOCAL_GID}" ]]; then
  echo "错误：无法取得本机 DDS 探针信息，请确认 ROS_DOMAIN_ID 和 RMW 配置。" >&2
  exit 1
fi

LOCAL_PREFIX="$(gid_prefix "${LOCAL_GID}")"
LOCAL_HOST="$(hostname)"
echo "本机: ${LOCAL_HOST}  DDS前缀: ${LOCAL_PREFIX}"

if (($#)); then
  TOPICS=("$@")
else
  mapfile -t TOPICS < <(ros2 topic list | sort)
fi

for topic in "${TOPICS[@]}"; do
  [[ "${topic}" == "${PROBE_TOPIC}" ]] && continue
  echo
  echo "================================================================"
  echo "话题: ${topic}"
  INFO="$(ros2 topic info "${topic}" -v 2>&1 || true)"
  if ! grep -q '^Type:' <<<"${INFO}"; then
    echo "状态: 不存在或当前不可发现"
    continue
  fi

  TYPE="$(awk -F': ' '/^Type:/ {print $2; exit}' <<<"${INFO}")"
  PUB_COUNT="$(awk -F': ' '/^Publisher count:/ {print $2; exit}' <<<"${INFO}")"
  SUB_COUNT="$(awk -F': ' '/^Subscription count:/ {print $2; exit}' <<<"${INFO}")"
  echo "类型: ${TYPE}"
  echo "发布者: ${PUB_COUNT:-0}    订阅者: ${SUB_COUNT:-0}"
  if [[ "${PUB_COUNT:-0}" == "0" ]]; then
    echo "结论: 没有 Publisher，因此当前没有数据来源。"
  fi

  while IFS=$'\t' read -r role node endpoint_type gid; do
    [[ -n "${gid}" ]] || continue
    prefix="$(gid_prefix "${gid}")"
    pid="$(gid_pid "${gid}" 2>/dev/null || true)"
    location="远端"
    process=""
    if [[ "${prefix}" == "${LOCAL_PREFIX}" ]]; then
      if [[ -n "${pid}" ]] && process="$(ps -p "${pid}" -o args= 2>/dev/null)" && [[ -n "${process}" ]]; then
        location="本机"
      else
        location="本机残留/进程已退出"
      fi
    fi

    if [[ "${role}" == "PUBLISHER" ]]; then
      purpose="数据来源"
    else
      purpose="数据消费者"
    fi
    echo
    echo "- ${purpose}: ${node}"
    echo "  位置: ${location}"
    echo "  GID: ${gid}"
    if [[ -n "${pid}" ]]; then
      echo "  推定PID: ${pid}"
    fi
    if [[ -n "${process}" ]]; then
      echo "  本机命令: ${process}"
    fi
  done < <(endpoint_rows "${INFO}")
done

echo
echo "说明: ROS2 图接口不提供远端主机名/IP；标记为远端时，节点名和GID是可确认的来源标识。"
