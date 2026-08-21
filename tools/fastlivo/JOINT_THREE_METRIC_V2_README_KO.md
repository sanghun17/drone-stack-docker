# Real-world experiment V2 재현 묶음

이 폴더는 논문/PPT용 세 그림, 5쌍 표, 21회 전체 지표 정본, 그리고
Windows에서 ROS 없이 같은 표와 그림을 다시 만드는 스크립트를 포함합니다.

## 바로 사용할 파일

`figures/`의 SVG는 PowerPoint의 **삽입 → 그림 → SVG**로 넣으면 벡터 상태로
편집·확대할 수 있습니다. PNG는 미리보기와 호환용입니다.

- `joint_scalar_1x3.svg`: 5쌍의 visual-support p10, translation APE RMSE,
  orientation RMSE
- `joint_progress_p10_1x3.svg`: 2초 rolling visual-support p10과 두 오차
- `joint_progress_d50_1x3.svg`: 2초 rolling `nfeat <= 50` 체류율과 두 오차
- `tables/joint_three_metrics_tables.xlsx`: PPT 표 작성용 Excel workbook
- `tables/candidate_pairs.csv`: 요청한 5쌍과 각 세션의 실제 recorded condition
- `tables/metric_summary.csv`: 논문 표에 바로 쓸 그룹 평균

그룹 평균은 다음과 같습니다.

| 지표 | display-PURE | display-Nominal |
|---|---:|---:|
| Visual-support p10 | 42.88 | 26.48 |
| Translation APE RMSE | 0.470 m | 0.854 m |
| Orientation RMSE | 12.62 deg | 20.14 deg |

5쌍은 다음 순서로 고정되어 있습니다.

1. `pm1 -> pw2`
2. `p1 -> pm4`
3. `p2 -> p5`
4. `n2 -> p3`
5. `pm0 -> n3`

각 화살표의 왼쪽은 세 지표 모두에서 더 유리합니다. `display-PURE`와
`display-Nominal`은 figure의 비교 그룹명이며, 실제 실행 condition은 바꾸지 않고
CSV/XLSX의 `recorded_condition`에 보존했습니다.

## Windows에서 다시 만들기

Python 3.10 이상이 설치된 PowerShell에서 이 `V2` 폴더로 이동한 뒤 실행합니다.

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r reproduce\requirements-windows.txt
python reproduce\reproduce_joint_three_metric_v2.py `
  --master data\all21_final_metrics.csv `
  --timeseries data\time_series_long.csv `
  --output regenerated
```

PowerShell 실행 정책 때문에 activate가 막히면 활성화 없이 다음처럼 실행해도 됩니다.

```powershell
.\.venv\Scripts\python.exe -m pip install -r reproduce\requirements-windows.txt
.\.venv\Scripts\python.exe reproduce\reproduce_joint_three_metric_v2.py `
  --master data\all21_final_metrics.csv `
  --timeseries data\time_series_long.csv `
  --output regenerated
```

성공하면 `PASS: verified 5 pairs / 10 sessions`가 출력되고,
`regenerated/figures/` 및 `regenerated/tables/`가 만들어집니다.

스크립트가 21회 중 eligible 18회를 다시 읽어 joint optimization을 수행합니다.
`p1 -> pm4`를 고정하고, 세 지표의 최소 표준화 그룹 차이를 최대화하며 path/duration
balance와 pair caliper를 검사합니다. 이 과정은 두 5-session arm을 재현합니다.
동일한 arm 안의 pair permutation은 그룹 목적값이 같을 수 있으므로, 최종 pair link와
그림 번호는 위의 요청된 표 순서로 고정하고 각 링크의 caliper와 세 지표 방향을 다시
검증합니다. 자세한 audit은 `selection_manifest.json`에 저장됩니다.

## 지표와 시간창

- Visual-support p10: FAST-LIVO의 VIO 단계에서 projection/NCC/photometric screening
  뒤 남은 visual-map measurement support의 세션별 10 percentile입니다. 일반적인
  detector feature count와 동일한 값은 아닙니다.
- Translation APE RMSE: spatial alignment 없이 GT와 연관한 위치 절대오차 RMSE입니다.
- Orientation RMSE: GT와 추정 회전의 SO(3) geodesic angle RMSE입니다.
- 시간 범위: stable hover 시작부터 착륙 직전까지입니다.
- progress figure: 물리 시간 2초 rolling 지표를 계산한 뒤 mission progress 0--100%로
  정규화한 값입니다. 공통 유효 구간만 그룹 median/IQR로 표시합니다.

`joint_progress_d50_1x3`의 50은 캠페인 비교용 low-support 기준이지 센서 failure를
정의하는 절대 임계값은 아닙니다.

## 파일 무결성

Linux/macOS:

```bash
sha256sum -c SHA256SUMS.txt
```

Windows PowerShell에서는 `Get-FileHash <file> -Algorithm SHA256`으로 개별 파일을
확인할 수 있습니다. 전송 직후 SMB에서 모든 파일을 다시 읽어 SHA256을 검증했습니다.

