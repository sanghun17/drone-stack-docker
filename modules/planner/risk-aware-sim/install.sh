#!/bin/bash
# airsim pip 설치 — 2단계 필수.
# airsim 1.8.1의 setup.py는 메타데이터 생성 시점에 airsim/types.py를 import하고
# 그 파일이 `import msgpackrpc`를 함 → msgpack-rpc-python이 '이미 설치돼' 있어야
# airsim 설치가 시작이라도 됨. 같은 `pip install` 한 방에 둘을 넣어도 resolution
# 단계에서 죽으므로 (2026-07-25 sim-x86 첫 빌드 실증) 반드시 순차 2회 호출.
# (호스트 conda airsim env를 대체 — so3_control_bridge.py가 쓰는 건 airsim 패키지뿐)
set -e
python3 -m pip install --no-cache-dir "msgpack-rpc-python==0.4.1"
python3 -m pip install --no-cache-dir "airsim==1.8.1"
# airsim 전이 의존으로 딸려오는 opencv-contrib-python 5.x가 cv_bridge cv2_to_imgmsg를
# KeyError: 16 으로 깨뜨림 (2026-07-25 sensor-pub 컨테이너화 실증) → 4.9 headless로 강제 핀.
# --no-deps: numpy 등 기존 스택 재해석 방지.
python3 -m pip install --no-cache-dir --no-deps "opencv-python-headless==4.9.0.80"
python3 -m pip uninstall -y opencv-contrib-python 2>/dev/null || true
