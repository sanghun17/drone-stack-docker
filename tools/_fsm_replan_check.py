#!/usr/bin/env python3
"""FSM replan 이유 집계 + TRACKING_DRIFT 발생 여부 (bag의 /rosout에서)."""
import sys, collections, rosbag
b = rosbag.Bag(sys.argv[1])
reasons = collections.Counter(); drift_lines = []; t0 = None
for topic, m, t in b.read_messages(topics=['/rosout']):
    if t0 is None: t0 = t.to_sec()
    msg = getattr(m, 'msg', '')
    if '[FSM] replan reason=' in msg:
        r = msg.split('reason=')[1].split(',')[0].strip()
        reasons[r] += 1
    if 'DRIFT' in msg.upper() or 'tracking' in msg.lower():
        drift_lines.append((t.to_sec()-t0, msg.strip()))
b.close()
print("BAG:", sys.argv[1].split('/')[-1])
print("[FSM] replan reason 집계 (loginfo_throttle 1Hz라 실제 replan 수 < 이 값 아님, 이유 종류 확인용):")
for r, c in reasons.most_common(): print(f"   {r:16s} {c}")
print(f"\nTRACKING_DRIFT 발생: {'YES' if reasons.get('TRACKING_DRIFT') else 'NO'}")
print(f"drift/tracking 관련 로그 라인 {len(drift_lines)}개:")
for ts, msg in drift_lines[:15]: print(f"   +{ts:6.1f}s  {msg}")
