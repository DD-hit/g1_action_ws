#!/usr/bin/env bash
# Run on the Jetson HOST, not inside the ROS container.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
container=${1:-ele_g1_humble}
[[ $(uname -m) == aarch64 ]] || { echo 'Jetson ARM64 host required'; exit 1; }
test -f /usr/include/aarch64-linux-gnu/NvInfer.h
docker exec "$container" bash -c 'apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ros-humble-realsense2-camera ros-humble-rqt-image-view python3-scipy python3-yaml'
docker exec "$container" python3 -m pip install --target /opt/ele_elevator/python --no-deps opencv-python-headless==4.11.0.86
g++ -std=c++17 -O2 -shared -fPIC "$ROOT/native/trt_bridge.cc" \
  -I/usr/include/aarch64-linux-gnu -I/usr/local/cuda/include \
  -L/usr/local/cuda/lib64 -lnvinfer -lnvinfer_plugin -lcudart \
  -Wl,-rpath,/opt/ele_elevator/lib -o /tmp/libele_trt.so
docker exec "$container" mkdir -p /opt/ele_elevator/lib
docker cp /tmp/libele_trt.so "$container":/opt/ele_elevator/lib/libele_trt.so
python3 - "$container" <<'PY'
import glob, pathlib, subprocess, sys
container=sys.argv[1]
patterns=['libnvinfer.so.8','libnvinfer_plugin.so.8','libcublas.so.11','libcublasLt.so.11','libcudnn*.so.8']
copied=set()
for pattern in patterns:
    for folder in ['/usr/lib/aarch64-linux-gnu','/usr/local/cuda/lib64']:
        for name in glob.glob(folder+'/'+pattern):
            p=pathlib.Path(name)
            if p.name in copied: continue
            subprocess.run(['docker','cp',str(p.resolve()),container+':/opt/ele_elevator/lib/'+p.name],check=True)
            copied.add(p.name)
print('Jetson runtime libraries installed:',sorted(copied))
PY
docker exec "$container" bash -c 'LD_LIBRARY_PATH=/opt/ele_elevator/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-} ldd /opt/ele_elevator/lib/libele_trt.so'
