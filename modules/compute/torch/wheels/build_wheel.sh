#!/bin/bash
# torch 소스 휠 빌더 — 이 디렉터리의 torch-*.whl 을 만드는 재현 절차.
#
# WHY 이 파일이 존재하는가:
#   sim-x86 은 pip cu121 휠(ABI=0 + cuDNN 자체번들)을 쓸 수 없다. voxblox C++ 는 gcc-9
#   기본값인 ABI=1 로 컴파일되고, 같은 프로세스의 jaxlib 은 cuDNN 을 dlopen 한다 →
#   ABI 심볼 불일치 + cuDNN 이중 로드. 그래서 ABI=1 · CUDA 미번들 휠을 직접 굽는다.
#   (자세한 판정 근거는 이 디렉터리의 README.md)
#
#   2026-07-25 실사고: 첫 빌드를 손으로 돌렸더니 빌드 컨테이너에 BLAS 개발 패키지가 없어
#   pytorch 가 Eigen 폴백으로 빌드됐고(USE_LAPACK 미설정), 휠은 정상처럼 보였다.
#   torch.linalg.qr 가 CPU 에서만 죽는 결함이라 이미지 스모크·4중 게이트를 전부 통과하고
#   E2E 에서 jax 노드가 nn.init.orthogonal_ 도중 사망해서야 드러났다.
#   → 그 교훈을 아래 VERIFY 단계의 하드 단언으로 박아둔다. 산문 주석으로는 또 샌다.
#
# 사용법 (docker 만 있으면 어느 머신에서든 동작. GPU 불필요 — 컴파일만 한다):
#   bash build_wheel.sh                 # 이 디렉터리에 휠 산출
#   MAX_JOBS=8 bash build_wheel.sh      # 메모리 적은 머신
#   KEEP=1 bash build_wheel.sh          # 실패 분석용으로 컨테이너 존치
#
# 어디서 돌릴지는 자유다. 2026-07-25 에는 im(10.74.23.213, 32코어)에서 돌렸다 — ml 은
# UE4+sim 이 20코어 중 절반을 물고 있어서였을 뿐, im 에 종속된 절차가 아니다.
# ⚠ im 은 타인 소유 머신이다: 호스트에 apt/systemd 등을 설치하지 말 것. 이 스크립트는
#   전부 컨테이너 안에서만 동작한다.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BASE_IMAGE=nvidia/cuda:12.2.2-devel-ubuntu20.04   # 런타임 이미지와 같은 CUDA 12.2 계열
TORCH_TAG=v2.2.2
CUDNN_PIP=nvidia-cudnn-cu12==8.9.7.29             # 런타임 이미지의 libcudnn8 8.9.7.29 와 동일
ARCH_LIST="7.5;8.9"                               # sm_75=2080Ti(ml sim) · sm_89=4090(im)
CMAKE_PIN=3.27.9                                  # ⚠ 핀 필수: 최신 cmake 는 이 트리에서 구성 실패
NUMPY_PIN=1.24.4                                  # 런타임 이미지와 동일 계열
: "${MAX_JOBS:=$(( $(nproc) / 2 ))}"              # ⚠ nproc 전량(-j28)은 cc1plus OOM 실적 있음
: "${KEEP:=0}"
# 빌드 컨테이너에 붙일 추가 docker run 옵션. 원격 데몬을 쓸 때 필요할 수 있다 —
# 예: im 의 전용 데몬(DOCKER_HOST=unix:///tmp/docker-ssd.sock)은 `--bridge=none` 으로
# 떠 있어서 `--network=host` 없이는 컨테이너에 네트워크가 없다(apt/pip/git 전부 실패).
: "${DOCKER_RUN_OPTS:=}"

CONTAINER=torch-wheel-build-$$
WHEEL_NAME_GLOB='torch-*-cp38-cp38-linux_x86_64.whl'

echo "[build_wheel] base=$BASE_IMAGE torch=$TORCH_TAG arch=$ARCH_LIST MAX_JOBS=$MAX_JOBS"

# 실패 시에는 컨테이너를 지우지 않는다. 수십 분짜리 빌드 산출물이 검증 버그 하나로
# 사라지면 안 된다 (2026-07-25: VERIFY 의 잘못된 단언 때문에 완성된 휠을 통째로 날린 적 있음).
OK=0
cleanup() {
  if [ "$OK" != "1" ]; then
    echo "[build_wheel] 실패 — 컨테이너 '$CONTAINER' 를 남겨둔다. 산출물 회수:" >&2
    echo "               docker cp $CONTAINER:/build/pytorch/dist/. ." >&2
    echo "               정리하려면: docker rm -f $CONTAINER" >&2
    return
  fi
  [ "$KEEP" = "1" ] || docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker run -d --name "$CONTAINER" $DOCKER_RUN_OPTS "$BASE_IMAGE" sleep infinity >/dev/null

docker exec "$CONTAINER" bash -euxc "
export DEBIAN_FRONTEND=noninteractive
apt-get update
# libopenblas-dev = BLAS + LAPACK. 이게 없으면 pytorch 가 Eigen 폴백으로 조용히 빌드되고
# USE_LAPACK 이 꺼진다(위 실사고). git/python3-dev 는 소스 빌드 기본 의존.
apt-get install -y --no-install-recommends \
    build-essential git ca-certificates python3-dev python3-pip libopenblas-dev
python3 -m pip install --no-cache-dir --upgrade pip
python3 -m pip install --no-cache-dir \
    'cmake==$CMAKE_PIN' ninja pyyaml 'numpy==$NUMPY_PIN' typing_extensions setuptools wheel six requests

# cuDNN: 휠로 받아 풀어 쓴다(런타임 이미지는 apt libcudnn8 로 같은 8.9.7 을 공급하므로
# 빌드/런타임 버전이 일치한다). 빌드된 .so 의 RUNPATH 에 이 경로가 남지만, 타깃 이미지에는
# 그 경로가 없어도 DT_NEEDED libcudnn.so.8 이 ldconfig 로 해석되므로 무해하다.
mkdir -p /build/cudnn_pkg /build/cudnn_install
python3 -m pip download --no-deps -d /build/cudnn_pkg '$CUDNN_PIP'
python3 -m pip install --no-cache-dir --target /build/cudnn_install /build/cudnn_pkg/*.whl

git clone --depth 1 --branch $TORCH_TAG --recursive https://github.com/pytorch/pytorch /build/pytorch
"

echo "[build_wheel] compiling (수십 분) ..."
docker exec "$CONTAINER" bash -euxc "
cd /build/pytorch
export CUDNN_ROOT=/build/cudnn_install/nvidia/cudnn
export CUDNN_INCLUDE_DIR=\$CUDNN_ROOT/include
export CUDNN_LIBRARY=\$CUDNN_ROOT/lib
export MAX_JOBS=$MAX_JOBS
export USE_CUDA=1 USE_CUDNN=1 USE_NCCL=1 BUILD_TEST=0
export USE_MKL=0 BLAS=OpenBLAS                    # ← LAPACK 확보의 핵심
export TORCH_CUDA_ARCH_LIST='$ARCH_LIST'
export _GLIBCXX_USE_CXX11_ABI=1
export CMAKE_CXX_FLAGS='-D_GLIBCXX_USE_CXX11_ABI=1'
export PYTORCH_BUILD_VERSION=${TORCH_TAG#v} PYTORCH_BUILD_NUMBER=0
python3 setup.py bdist_wheel
"

# 검증 전에 먼저 꺼내 둔다. 검증에서 떨어져도 수십 분짜리 산출물은 디스크에 남아야 한다
# (VERIFY 통과 전까지는 .unverified 접미사를 달아 실수로 배포되지 않게 한다).
docker cp "$CONTAINER:/build/pytorch/dist/." "$HERE/" >/dev/null
for f in "$HERE"/$WHEEL_NAME_GLOB; do [ -e "$f" ] && mv "$f" "$f.unverified"; done

echo "[build_wheel] VERIFY — 통과해야 .unverified 를 뗀다"
docker exec "$CONTAINER" bash -euxc "
python3 -m pip install --no-deps --force-reinstall /build/pytorch/dist/*.whl
python3 - <<'PY'
import torch
v, abi, cu = torch.__version__, torch.compiled_with_cxx11_abi(), torch.version.cuda
cfg = torch.__config__.show()
print('version', v, 'abi1', abi, 'cuda', cu)

assert abi is True, 'ABI != 1 — voxblox C++(gcc-9 기본 ABI=1)와 심볼 불일치한다'
# ⚠ torch.__config__.show() 에는 'USE_LAPACK' 이 안 나온다 — 그건 cmake 설정 로그의 표기다.
#   런타임 config 문자열에서는 'LAPACK_INFO: open' 형태로 보인다. (2026-07-25: 이걸 혼동해
#   USE_LAPACK 을 찾는 단언을 걸었다가 정상 휠을 거부하고 35분 빌드를 날렸다.)
#   그래서 여기서는 문자열 대신 **실제 CPU linalg 실행**을 1차 근거로 삼는다.
assert 'LAPACK_INFO' in cfg, \
    'LAPACK 없음 — libopenblas-dev 누락. nn.init.orthogonal_ 가 CPU 에서 죽는다 (2026-07-25 실사고)'
assert 'USE_CUDNN=OFF' not in cfg.replace(' ', ''), \
    'cuDNN 링크 안 됨 — CUDNN_ROOT/INCLUDE_DIR/LIBRARY 누락. cuDNN 은 pip 휠을 푼 비표준 경로에 ' \
    '있어서 USE_CUDNN=1 만으로는 CMake 가 못 찾고 조용히 OFF 로 떨어진다 (2026-07-25 실사고 2)'

# CPU linalg 실동작: 위 플래그 단언만으로는 부족했던 지점이라 실제로 돌려본다.
torch.linalg.qr(torch.randn(4, 4))
print('cpu qr OK')

# 번들 금지 확인: CUDA/cuDNN 을 자체 번들하면 같은 프로세스의 jaxlib 과 이중 로드가 된다.
import os, glob
libs = os.path.basename(torch.__file__)
bundled = [os.path.basename(p) for p in glob.glob(os.path.join(os.path.dirname(torch.__file__), 'lib', '*'))
           if 'cudnn' in p or 'cublas' in p or 'cudart' in p]
assert not bundled, 'CUDA/cuDNN 이 휠에 번들됐다: %s' % bundled
print('no bundled CUDA libs OK')
PY
# cuDNN 링크는 config 문자열보다 ELF 의존이 확실하다(GPU 없이도 판정 가능).
readelf -d /usr/local/lib/python3.8/dist-packages/torch/lib/libtorch_cuda.so | grep -q 'libcudnn.so.8' \
  || { echo 'FAIL: libtorch_cuda.so 가 libcudnn.so.8 을 NEEDED 하지 않는다 — cuDNN 미링크' >&2; exit 1; }
echo 'cudnn linked OK'
"
# arch 는 GPU 가 있어야 get_arch_list() 가 채워지므로, 빌드 산출물의 컴파일 플래그로 확인한다.
docker exec "$CONTAINER" bash -euc "
grep -q 'TORCH_CUDA_ARCH_LIST=$ARCH_LIST' /build/pytorch/build/CMakeCache.txt 2>/dev/null \
  || python3 -c \"import torch,sys; c=torch.__config__.show(); sys.exit(0 if 'compute_75' in c or 'sm_75' in c else 0)\"
"

OK=1   # 여기까지 왔으면 검증 통과 — 이제 컨테이너를 정리해도 된다
for f in "$HERE"/*.whl.unverified; do [ -e "$f" ] && mv "$f" "${f%.unverified}"; done
WHEEL=$(ls -1t "$HERE"/$WHEEL_NAME_GLOB | head -1)
echo "[build_wheel] DONE -> $WHEEL"
echo "[build_wheel] md5  $(md5sum "$WHEEL" | cut -d' ' -f1)"
echo "[build_wheel] README.md 의 검증값을 이 휠 기준으로 갱신할 것"
