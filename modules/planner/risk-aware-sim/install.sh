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
