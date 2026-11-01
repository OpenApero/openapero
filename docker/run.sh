#!/usr/bin/env bash
# Start an Open Apero container. Usage:
#   docker/run.sh [--gui] [--gpu] [--root] [--image NAME] [-- COMMAND...]
#     --gui   forward X11 so RViz opens on the host display (run `xhost +local:docker` once first)
#     --gpu   pass the NVIDIA GPUs through (needs the NVIDIA Container Toolkit on the host) and
#             mount $HSSD_DATA_ROOT, for the Habitat experiments; it does not select the image:
#             add --image open_apero:habitat (built from docker/Dockerfile.habitat)
#     --root  run as root, e.g. to rebuild the workspace; files it writes to the data dir are root's
# The container runs as the host user with HOME=/home/open_apero, a tmpfs; run results land in the
# host's data root, $OPEN_APERO_DATA_ROOT or $HOME/open_apero_data, mounted at ~/open_apero_data in
# the container.
set -euo pipefail

usage() {
  printf '%s\n' "$1" >&2
  echo "usage: docker/run.sh [--gui] [--gpu] [--root] [--image NAME] [-- COMMAND...]" >&2
  exit 2
}

image=open_apero:latest
as_root=false
args=(--rm --net=host)
[ -t 0 ] && args+=(-it)

while [ $# -gt 0 ]; do
  case "$1" in
    --gui)
      args+=(-e "DISPLAY=${DISPLAY:?DISPLAY is not set}" -e QT_X11_NO_MITSHM=1
             -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
      [ -d /dev/dri ] && args+=(--device /dev/dri)
      shift ;;
    --gpu)
      args+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all
             -e "HSSD_DATA_ROOT=/data/hssd" -v "${HSSD_DATA_ROOT:?HSSD_DATA_ROOT is not set}:/data/hssd:ro")
      shift ;;
    --root) as_root=true; shift ;;
    --image)
      case "${2:-}" in ""|-*) usage "--image needs an image name" ;; esac
      image="$2"; shift 2 ;;
    --) shift; break ;;
    *) usage "unknown option: $1" ;;
  esac
done

data_root=${OPEN_APERO_DATA_ROOT:-$HOME/open_apero_data}
mkdir -p "$data_root"
if [ "$as_root" = true ]; then
  args+=(-v "$data_root:/root/open_apero_data")
else
  # The user has no /etc/passwd entry in the image; USER and LOGNAME stand in for it.
  home=/home/open_apero
  args+=(--user "$(id -u):$(id -g)" -e "HOME=$home" -e "USER=$(id -un)" -e "LOGNAME=$(id -un)"
         --tmpfs "$home:exec,uid=$(id -u),gid=$(id -g),mode=0755"
         -v "$data_root:$home/open_apero_data")
fi
exec docker run "${args[@]}" "$image" "$@"
