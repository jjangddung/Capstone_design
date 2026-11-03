# 초음파 센서 기반 2D 위치추정 — capstone_design

1 × 1 m 정사각형 공간에서 벽에 설치한 초음파 센서 4개의 거리로 원기둥 타깃의 중심 위치를 추정한다. MATLAB/Simulink 가상 실험과 Arduino 실측에 같은 위치추정 함수를 사용하며, LS·LMedS·MotionLMedS 결과를 비교한다. 현재 실험은 타깃을 한 위치에 고정하고 30회 측정하는 방식이다.

현재 구현은 **각 센서의 왕복 비행시간으로 계산한 거리 기반 추정**이다. 초기 구상했던 송수신기 분리형 TDOA, 40 kHz 원파형 수집, 머신러닝 보정은 구현되어 있지 않다.

## 1. 현재 센서 배치와 설정

원점은 공간의 왼쪽 아래 벽 모서리이며, x는 오른쪽, y는 위쪽이다. 센서의 **거리 측정 기준면이 벽에서 2.5 cm 안쪽**에 있는 조건을 반영한다.

| 센서 | Arduino 핀 | 측정 기준면 좌표 [m] | 방향 |
|---|---|---|---|
| S1 | D8 | (0.025, 0.500) | 오른쪽 (+x) |
| S2 | D9 | (0.975, 0.500) | 왼쪽 (−x) |
| S3 | D10 | (0.500, 0.025) | 위쪽 (+y) |
| S4 | D7 | (0.500, 0.975) | 아래쪽 (−y) |

각 센서는 벽 중앙에서 벽에 수직으로 공간 안쪽을 바라본다. Arduino의 출력 순서도 S1, S2, S3, S4에 맞춰야 한다.

| 항목 | 현재 설정 | 설명 |
|---|---|---|
| 공간 | 1 × 1 m | 벽 기준 좌표계 |
| `sensorInset` | 0.025 m | 센서 기준면의 돌출 거리 |
| `radius` | **0.009 m = 9 mm** | 원기둥 타깃 반지름 |
| `roi` | `[0.4 0.6 0.4 0.6]` | 추정 가능한 중심 위치: x, y 각각 40~60 cm |
| `beamFullAngleDeg` | **38° (±19°)** | 기본 가상 감지각 |
| `residualLimit` | 0.025 m | 강건 추정의 거리 잔차 허용값 |
| `roiRangeExtra` | 0.05 m | ROI 기반 거리 필터 양쪽에 추가한 5 cm 여유 |
| `alpha`, `beta` | 1, 0 | 센서별 거리 보정 계수, 현재 항등 보정 |
| `baudRate` | 115200 | Arduino와 MATLAB 통신 속도 |
| `bodySize` | `[0.4 0.4]` m | 시각화용 예시 몸체 크기; 실측 치수 아님 |

설정 원본은 [ultraConfig.m](ultraConfig.m)이다. 아래 연결 문서에 과거 예시 좌표나 반지름 5 mm가 남아 있다면 위 표와 현재 코드의 값을 기준으로 한다.

### 거리 모델과 감지각

타깃 중심을 `p = [x, y]`, 센서 좌표를 `s_i`, 반지름을 `r`이라고 하면 예측 거리는 다음과 같다.

```text
d_i(p) = norm(p - s_i) - r
```

센서와 반사 타깃이 같은 높이에 있고 원형 단면의 가까운 표면에서 반사된다는 근사다. 중앙 (0.5, 0.5) m에서 이상적인 표면 거리는 네 센서 모두 **0.475 − 0.009 = 0.466 m = 46.6 cm**이다. RAW 거리값에 2.5 cm를 일괄 더하지 않고 센서 좌표에 돌출량을 반영한다.

FIXED 모드의 가상 감지 조건은 센서 정면 방향 거리 `a > 0`, 횡방향 거리 `abs(b) <= a*tan(19°)`이다. 타깃 중심 기준이며, 모든 거리에서 검증된 실물 감지 범위라는 뜻은 아니다. 실측 모드에서는 GT와 가상 감지각으로 수신 거리를 생성하거나 보정하지 않는다.

## 2. Arduino 연결 및 30회 측정

MATLAB R2024b에서 프로젝트 폴더를 Current Folder로 설정한다. Arduino에는 현재 사용 중인 **4센서·24항목 CSV 출력 스케치**를 업로드한다. Serial Monitor와 Serial Plotter는 닫아 COM 포트를 MATLAB이 사용할 수 있게 한다.

```matlab
launchUltra
```

1. 화면의 **Connect Arduino (USB)**를 누르고 실제 포트(현재 실험 환경에서는 COM9)를 선택한다.
2. 연결 상태가 **Ready**로 바뀔 때까지 기다린다.
3. 타깃을 정지시키고 **X / GT [m]**, **Y / GT [m]**에 벽 기준 실제 중심 좌표를 입력한다. 예: 40 cm, 50 cm → `0.4`, `0.5`.
4. **Measure 30**을 누른다. 측정 중 GT 입력은 잠기며, 완료 후 다시 변경할 수 있다.
5. Arduino 종료 신호를 받으면 `results/`에 실험별 CSV 두 개가 자동 저장된다. 연결을 끊지 않고 다음 위치를 측정할 수 있다.
6. 작업을 마치면 **Disconnect** 또는 창 닫기로 종료한다.

포트를 직접 지정해서 연결할 수도 있다. `ultraConfig`의 기본 포트는 COM3이므로 COM9 사용 시 명시한다.

```matlab
cfg = ultraConfig();
cfg.inputMode = 'serial';
cfg.serialPort = 'COM9';
cfg.baudRate = 115200;
launchUltra(cfg)
```

실측 모드는 PC 타이머로 USB 데이터를 읽으므로 **Simulink Run을 별도로 누르지 않는다**. MATLAB이 연결 확인과 측정 명령을 보내므로 Serial Monitor에서 따로 입력할 필요도 없다. GT 입력은 로봇을 이동시키지 않으며, 위치추정 함수에 정답으로 전달되지 않는다. 비교·오차 평가·파일명에 사용한다.

센서 기준면 사이 간격은 95 cm이다. MATLAB은 Arduino에 `G 95 95`를 보내고, GT에서 (2.5, 2.5) cm를 뺀 좌표를 P1 명령으로 전달한다. 예를 들어 화면 GT (50, 50) cm는 `P1 47.5 47.5`로 전송된다. 수신 GT와 Arduino 계산 좌표는 다시 벽 기준으로 변환하므로 사용자가 직접 보정할 필요는 없다.

현재 스케치는 센서를 순차 측정하고 각 측정 뒤 60 ms를 기다린다. Echo 대기와 통신 시간도 있으므로 실제 순회 주기는 240 ms로 고정되지 않는다. CSV의 `ScanPeriod_ms`로 확인한다. 순차 발사와 대기시간은 간섭을 줄이는 방법이며 간섭이 없음을 보장하지는 않는다.

## 3. 결과 파일 확인

저장 위치는 프로젝트의 **`results/`** 폴더다. 현재 PC에서는 다음 경로다.

```text
C:\Users\장동민\workspace\capstone_design\results
```

실험별 파일명 예:

```text
GT_X50.000cm_Y50.000cm_Run006_N30_20261103_201344_748_samples.csv
GT_X50.000cm_Y50.000cm_Run006_N30_20261103_201344_748_stats.csv
```

- `GT_X..._Y...`: 해당 실험에 입력한 벽 기준 GT, 단위 cm.
- `Run006`: 연결 세션 내 실험 순번. Arduino의 실험 번호는 `Arduino_Run_ID`에 따로 저장한다.
- `N30`: 수신·해석한 30개 순회 데이터. 위치추정 성공 횟수가 아니다.
- 날짜·시각: 연결 세션의 시작 시각.
- `INCOMPLETE`: 종료 신호, 예정 횟수 또는 측정 번호 연속성이 충족되지 않은 실험.

같은 이름의 파일은 덮어쓰지 않고 `_v2` 등의 접미사로 구분한다. 측정 완료 시 화면과 MATLAB 명령창에 저장 경로를 표시한다. 측정 중 연결을 끊은 경우에도 수집된 부분을 저장한다.

### `samples.csv` — 한 행이 센서 4개의 한 순회

| 열 | 의미 |
|---|---|
| `GT_X_cm`, `GT_Y_cm` | 입력한 실제 중심 좌표 |
| `S1_RAW_cm` ~ `S4_RAW_cm` | MATLAB 보정·제외 전 수신 거리 |
| `S*_Echo_us` | Echo 펄스 시간 [µs] |
| `S*_Time_us` | Arduino의 센서 측정 시작 시각 |
| `S*_Status` | `OK`, `TIMEOUT` 등 센서 상태 |
| `S*_Used` | 위치추정 전 유효성 필터 통과 여부; 최종 inlier 여부와 다름 |
| `S*_Corrected_cm` | 센서 보정 계수 적용 후 거리 |
| `UsedSensor_N` | 추정 전 필터를 통과한 센서 수 |
| `LS_*`, `LMedS_*`, `MotionLMedS_*` | 알고리즘별 좌표·오차·성공 여부·상태 |
| `ScanPeriod_ms` | 같은 실험의 연속 순회 시작 시각 차이; 첫 행은 없을 수 있음 |
| `Arduino_X_cm`, `Arduino_Y_cm` | Arduino의 단순 계산 결과를 벽 좌표로 변환한 비교값 |

여기서 **RAW는 Arduino가 Echo 시간으로 계산한 거리**다. 40 kHz 아날로그 수신 파형을 뜻하지 않는다. 위치추정에 사용하지 않은 센서 거리도 CSV에 보존한다. Echo가 없으면 수신된 NaN과 TIMEOUT을 그대로 기록하며, 실패한 추정 좌표와 오차도 NaN으로 남긴다.

### `stats.csv` — 알고리즘별 통계

LS, LMedS, MotionLMedS별로 한 행씩 저장한다.

- `Expected_N`, `Received_N`, `Complete`: 예정 횟수, 실제 수신 횟수, 수집 완료 여부.
- `Success_N`, `Failure_Rate`: 위치추정 성공 횟수와 수신 행 기준 실패율.
- `Mean_X/Y_cm`, `Std_X/Y_cm`: 성공한 좌표의 평균과 표본 표준편차.
- `Bias_X/Y_cm`, `Mean_Error_cm`, `Std_Error_cm`, `RMSE_cm`, `Max_Error_cm`: GT 대비 오차 통계.
- `GT_Samples`: 추정에 성공했고 GT도 유효해서 오차 평가에 사용한 표본 수.
- `S*_RAW_Valid_N`, `S*_RAW_Mean_cm`, `S*_RAW_Std_cm`: 센서별 OK·유한 거리의 개수, 평균, 표본 표준편차. 추정에서 제외된 거리도 포함한다.

위치 통계는 성공한 표본만, 오차 통계는 성공하고 GT가 있는 표본만 사용한다. 표준편차는 N−1 기준이며 표본이 2개 미만이면 NaN이다. 따라서 성공이 1회뿐인 RMSE를 30회 모두 성공한 결과처럼 해석하면 안 된다. 센서 RAW 통계는 알고리즘 행마다 같은 값으로 반복된다.

### 추가 기록

| 파일 | 내용 |
|---|---|
| `serial_experiment_*.csv` | Arduino 수신 원문 및 통신 기록; 24항목 로그 재생에 사용 |
| `live_*.mat` | 연결 종료 시 저장하는 설정·측정 이력·외부 입력 기록 |
| `live_*_summary.csv`, `live_*_run*_summary.csv` | 세션 전체 및 실험별 기존 요약 |
| `simulation_*.mat`, `simulation_*_summary.csv` | 가상 실험 결과; 실제 Arduino 측정과 구분 |

원문 로그의 `# COORDINATE_ORIGIN_M` 헤더는 좌표 변환에 사용하므로 유지한다. 상세 형식은 [CSV_RESULTS.md](CSV_RESULTS.md)를 참고한다.

## 4. 로봇이 없는 상태에서 측정하기

빈 공간에서도 Arduino 거리값을 받고 저장할 수 있다. 현재 UI에서는 ROI 안의 임시 GT를 입력하고 **Measure 30**으로 수집한다. 이때 파일명의 GT는 실험 구분용 임시값이며 실제 로봇 위치를 뜻하지 않는다. 별도 기록에 빈 공간 실험임을 표시하고 위치 정확도 평가에서는 제외한다.

확인할 값은 `samples.csv`의 **S1~S4 RAW 거리·Echo·Status**, `stats.csv`의 **S1~S4 RAW 평균·표준편차**다. 로봇이 없어 벽이 측정되면 거리값은 있어도 위치추정 좌표는 NaN일 수 있다.

예를 들어 `GT_X50.000cm_Y50.000cm_Run006_N30_20261103_201344_748_samples.csv`에는 빈 공간에서 얻은 약 97~98 cm의 RAW 거리 30회가 저장되어 있다. 이 데이터가 위치추정 필터에서 제외되는 것과 센서 데이터 수집 실패는 다른 상황이다. 임시 GT를 넣어도 벽 반사가 로봇 반사로 바뀌지는 않는다.

## 5. 거리 필터와 NaN 해석

보정 거리는 `(RAW - beta) / alpha`로 계산한다. 센서 상태, 유한값, 기본 거리 범위, 활성화된 배경 필터, ROI에서 가능한 거리 범위를 검사한다. 현재 ROI 기반 범위는 다음 식이다.

```text
여유 = residualLimit + roiRangeExtra = 2.5 cm + 5 cm
최소 허용 거리 = 센서에서 ROI까지의 최단 거리 − 반지름 − 여유
최대 허용 거리 = 센서에서 ROI의 가장 먼 모서리까지 거리 − 반지름 + 여유
```

현재 대칭 배치와 반지름 9 mm에서는 네 센서 모두 약 **29.10~64.96 cm**이다. 이는 센서 자체의 측정 가능 범위가 아니라 **중앙 타깃 추정을 위한 필터**다. 범위 밖 RAW 거리도 저장한다. 추가 5 cm는 거리 필터에만 반영했으며 추정 ROI와 LMedS 잔차 한계 2.5 cm를 넓힌 것은 아니다.

| 상태·현상 | 의미 |
|---|---|
| `TIMEOUT` | 제한 시간 안에 Echo가 수신되지 않음 |
| `INSUFFICIENT` | 필터 통과 센서가 3개 미만 |
| `NO_CONSENSUS` | LMedS에서 잔차 조건을 만족하는 센서 3개 이상을 확보하지 못함 |
| `POOR_GEOMETRY` | 해당 거리·배치의 기하조건이 위치 계산에 부적합 |
| `TIME_SPAN_TOO_LARGE` | 센서별 측정 시각 차이가 설정된 허용 조건을 초과 |
| `MOTION_REJECTED` | MotionLMedS 후보 선정 실패; 이동 제약뿐 아니라 잔차 불일치도 원인이 될 수 있음 |
| 좌표 NaN, RAW는 숫자 | 수집은 되었지만 해당 알고리즘의 위치추정은 실패 |

센서 `OK`는 Echo를 받았다는 뜻이며 원기둥 반사를 받았음을 보장하지 않는다. 실측에서는 미보정 가상 벽 거리로 실제 값을 제거하지 않도록 배경 필터를 기본 비활성화한다. 빈 공간 실측을 반영할 때 `background`, `backgroundTolerance`, `liveWallCalibrated`를 함께 검토한다.

설정을 수정한 뒤에는 기존 연결·실행을 종료하고 **새 `cfg = ultraConfig()`로 다시 시작**한다. 실행 중인 세션과 이미 만들어 둔 cfg에는 수정한 파일 값이 자동 반영되지 않는다. 과거 CSV도 자동 재계산되지 않는다.

## 6. 가상 실험 및 로그 재생

### Simulink 가상 실험

```matlab
launchUltra
```

`ultrasonic_localization.slx`와 조작 창이 열린다. **X/Y 입력 → Apply X/Y + settings → Run / Restart** 순서로 실행한다. 기본 가상 실행 시간은 60초다.

- **Static placement**: 정적 위치를 지정한다. 현재 실측 방식에 대응한다.
- **Move slowly to goal**: 가상 이동 시나리오를 확인하는 선택 기능이다.
- **FIXED 38 deg total**: 기본 ±19° 감지 모델.
- **IDEAL**: 각도 제한 없이 수식·알고리즘을 확인한다.
- **LEGACY measured profile**: 과거 20/30/40 cm에서의 ±39/29/22° 측정을 보간한다. 범위 밖은 미보정 구간이며 현재 기본 모델이 아니다.
- 거리 시나리오에서 이상적 거리, 잡음·양자화, 이상값·벽 반사를 비교할 수 있다.

화면은 GT와 LS·LMedS·MotionLMedS 위치를 함께 표시한다. Simulink의 `UltraSystem`이 MATLAB 함수를 PC에서 실행하는 구조이며 Arduino에 모델을 자동 배포하는 구성은 아니다. 기본 MATLAB 행렬 연산으로 추정하며 Optimization Toolbox는 필요하지 않다.

### 실제 Arduino 원문 로그 재생

```matlab
cfg = ultraConfig();
cfg.inputMode = 'replay';
cfg.inputFormat = 'experiment24';
cfg.replayFile = fullfile(cfg.root, 'results', ...
    'serial_experiment_20261103_201345_425.csv'); % 재생할 원문 로그
cfg.stopTime = 600; % 로그 길이에 맞게 설정
launchUltra(cfg)
```

재생 창에서 Run을 누른다. 입력에는 원문 `serial_experiment_*.csv`를 사용한다. 가공된 `*_samples.csv`, `*_stats.csv`, `.mat`는 이 입력 형식이 아니다. 기존 9항목 `M,1,...` 로그는 `inputFormat='legacy9'`로 지원한다.

합성 예제 재생:

```matlab
cfg = ultraConfig();
cfg.inputMode = 'replay';
cfg.replayFile = makeDemoReplay();
cfg.staticGT = [.48 .52];
cfg.staticGTConfirmed = true;
launchUltra(cfg)
```

## 7. 주요 파일과 검증

| 파일 | 역할 |
|---|---|
| `launchUltra.m` | 가상·재생·실측 모드 시작 |
| `ultraConfig.m` | 센서 좌표, 반지름, 감지각, ROI, 필터 설정 |
| `buildModel.m`, `UltraSystem.m` | Simulink 모델 생성 및 실행 |
| `ultraTick.m`, `ultraInitialize.m` | 가상 데이터 생성 및 상태 초기화 |
| `UltraExperimentInput.m`, `parseUltraExperimentLine.m` | 24항목 데이터 해석과 실험 명령 처리 |
| `UltraLiveSession.m` | 실제 USB 수신 타이머, 측정 상태 및 저장 관리 |
| `ultraExternalTick.m` | 수신 데이터를 공통 추정기로 전달하고 기록 |
| `ultraEstimate.m`, `ultraRange.m` | 거리 모델과 LS·LMedS·MotionLMedS 추정 |
| `ultraSensorFrame.m` | 벽 기준과 Arduino 센서 기준 좌표 변환 |
| `ultraSaveRunCsv.m` | RAW·GT·위치 결과·통계 CSV 저장 |
| `ultraView.m` | 조작 창과 결과 시각화 |
| `UltraInput.m` | 기존 9항목 로그 입력 지원 |

회귀 검증 명령:

```matlab
tests = {'testUltra.m', 'testBeam38.m', 'testFourWalls.m', ...
    'testArduinoExperiment.m', 'testRunCsv.m', 'testRangeExtra.m'};
testResults = runtests(tests);
assertSuccess(testResults)
```

`validateProject.m`은 별도로 모델 생성·실행·미리보기를 검증한다. 소프트웨어 테스트 통과와 실제 센서 정확도 검증은 구분한다. 현재 센서 4개 중 유효 거리 3개 이상이 필요하며, 4개만으로 모든 이상값을 식별한다고 보장할 수는 없다.

추가 문서: [CSV 저장 형식](CSV_RESULTS.md), [Arduino 연결](ARDUINO_LIVE.md), [센서 좌표 변환](SENSOR_GEOMETRY.md), [모델링 상세계획](docs/MATLAB_초음파_위치추정_모델링_및_Arduino_연동_상세계획.md).
