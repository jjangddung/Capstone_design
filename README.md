# 초음파 위치추정 Simulink 구현

## 실행

MATLAB R2024b에서 이 폴더를 **Current Folder**로 선택한 후 다음 명령어를 실행한다.

```matlab
launchUltra
```

`ultrasonic_localization.slx`와 조작 창이 열린다. **X·Y 입력 → Apply X/Y + settings → Run / Restart** 순서로 실행한다. x·y 슬라이더로 값을 바꿔도 **Apply**를 눌러야 반영된다. 시뮬레이션은 기본 60초이며, 종료 후 **Run**으로 다시 실행할 수 있다.

한글 경로도 지원한다. 다른 폴더에서 실행하려면 이 폴더를 `addpath`로 등록한다. `.slx`만 열어 실행하기보다 `launchUltra`를 사용해야 설정과 함수 경로가 준비된다.

## 조작 화면

- **Static placement:** 좌표를 즉시 변경한다. 새 실험으로 취급해 추적 상태와 미완성 scan을 초기화한다.
- **Move slowly to goal:** 현재 위치에서 목표 좌표까지 벡터 속도 제한을 지키며 이동한다.
- **ASSUMED detection:** 전 센서가 타깃을 감지한다고 가정한다. 알고리즘 검증용이며 실제 감지범위를 보장하지 않는다.
- **MEASURED ONLY:** 정면 축 거리 20/30/40 cm에서 측정한 ±39/29/22°를 편측 폭으로 바꿔 선형 보간한다. 20~40 cm 밖은 `UNCALIBRATED`로 처리한다. 현재 중앙 위치에서는 6개 모두 미측정 구간일 수 있으므로 추정 실패가 정상이다.
- **Ideal distances:** 잡음 없이 수식 검증.
- **Noise + 1 cm quantization:** 표준편차 3 mm 가정 잡음과 1 cm 양자화.
- **Outlier S2 + wall S4:** S2에 +10 cm 이상값, S4에 가정한 벽 거리 입력.

검은 표식은 결과 기준시각의 GT, 파랑·주황·초록은 각각 LS·LMedS·이동 제약 LMedS이다. 검은 `+`는 현재 GT이다. 이동 중에는 현재 GT와 추정 기준시각의 GT가 다르다. 오차는 기준시각 GT와 비교한다. 회색 몸체와 작은 원형 타깃은 서로 다른 크기로 표시한다. 점선은 실측 구간의 감지경계이다.

## 반드시 실측으로 바꿀 값

`ultraConfig.m`의 다음 기본값은 **예시**이다.

| 설정 | 초기값 | 의미 |
|---|---|---|
| `radius` | 0.005 m | 타깃 반지름. 거리식에서 유지 |
| `bodySize` | `[0.4 0.4]` m | 가상 몸체 크기 |
| `roi` | x, y 각각 0.35~0.65 m | 중앙 이동영역 |
| `slot` | 0.06 s | 순차 Trigger 시작 간격 |
| `vSim` / `vMax` | 0.08 / 0.15 m/s | 가상 속도 / 이동 제약 최대속도 |
| `background` | 중심축과 벽의 교점 거리 | 실제 빈 공간 배경으로 교체 필요 |

몸체 크기와 ROI가 달라지면 서로 충돌하지 않도록 함께 변경해야 한다. 센서 좌표·높이와 타깃 형상도 확인한다. 거리식은 같은 높이의 원형 타깃 표면까지 `norm(p-s)-r`로 근사한다.

## 알고리즘과 Simulink 구성

`Digital Clock + UserCommand → UltraSystem → 로그/화면`으로 연결한다. `UltraSystem`은 PC에서 해석 실행하며 다음 MATLAB 함수를 호출한다.

- `ultraTick`: GT 이동, 센서별 순차 측정, 감지·잡음 생성.
- `ultraEstimate`: 공통 보정·배경·ROI 필터, LS/LMedS/이동 제약 추정.
- `ultraRange`: 반지름 포함 거리식.
- `UltraInput`, `ultraExternalTick`: 로그/실시간 측정 입력.
- `ultraView`: 좌표 설정과 평면도·오차·상태 표시.

최적화는 기본 MATLAB 행렬 연산과 감쇠 Gauss–Newton으로 구현했다. Optimization Toolbox는 필요하지 않다. 일반 MATLAB/Simulink와 PC 직렬 연결이 필요하며, Arduino에 이 모델을 자동 배포하는 구성은 아니다. SILS·하드 실시간 실행으로 검증한 모델도 아니다.

후보 3개 조합 최대 20개에서 기하조건을 검사하고 전체 유효 거리의 제곱 잔차 중앙값으로 선택한다. 선택된 inlier로 다시 추정한다. 유효 센서 부족·기하 퇴화·큰 측정 시각차·후보 소실은 실패 상태로 남긴다. GT는 추정기에 전달하지 않는다.

## 저장 결과

실행 종료 또는 Stop 시 `results/`에 다음 파일이 저장된다.

- `<mode>_<timestamp>.mat`: 설정, scan별 원시거리·측정시각·GT·추정결과, 사용자 명령 이력.
- `<mode>_<timestamp>_summary.csv`: 방법별 RMSE·최대오차·실패율·계산시간 및 GT 표본 수.
- 실시간 입력은 `serial_raw_*.csv` 원문 로그도 저장한다.

MATLAB Workspace의 `ultraHistory`에도 결과가 남는다. GT가 없으면 RMSE는 NaN이다. 실패를 0 오차로 계산하지 않는다. 전체 요약 외에 실험 구간별 분석은 `history.experiment`로 분리하면 된다.

`ultraSignals`의 32개 출력 채널: 1:2 현재 GT, 3:4 기준시각 GT, 5:10 방법별 x/y, 11:13 오차[m], 14:19 보정거리, 20:25 유효 mask, 26:28 추정 성공, 29 새 scan, 30 결과 기준시각, 31 모델시각, 32 실험 ID. 새 scan 플래그는 한 모델 step에서 한 번만 참이다. To Workspace의 배열은 출력 신호 차원에 따라 32×1×N일 수 있으므로 시간축과 출력 채널을 구분해서 읽는다.

## Arduino 로그 재생

실물 없이 먼저 호환 형식을 확인할 수 있다.

```matlab
cfg = ultraConfig();
cfg.inputMode = 'replay';
cfg.replayFile = makeDemoReplay(); % 합성 예제, 실제 실험 데이터 아님
cfg.staticGT = [.48 .52];
cfg.staticGTConfirmed = true; % 이 합성 예제의 알려진 고정 좌표
launchUltra(cfg)
```

CSV는 다음 **9개 필드**이며 한 파일에는 한 세션만 저장한다.

```text
M,1,session_id,scan_id,sensor_id,t_trigger_us,t_done_us,range_mm,status
M,1,7,120,1,12000000,12004000,640,OK
M,1,7,120,2,12060000,12085000,nan,TIMEOUT
```

인정 상태는 OK/TIMEOUT/OUT_OF_RANGE/DEVICE_ERROR이다. 중복·잘못된 ID·잘못된 줄은 제외하고, 센서 누락 scan은 대기시간이 지나면 남은 측정만으로 처리한다. µs 타임스탬프의 32비트 래핑을 처리한다. `.mat` 결과를 직접 CSV 입력 파일로 지정하지 않는다.

## Arduino 실시간 입력

```matlab
cfg = ultraConfig();
cfg.inputMode = 'serial';
cfg.serialPort = 'COM3'; % 실제 포트로 변경
cfg.baudRate = 115200;
cfg.staticGTConfirmed = false;
launchUltra(cfg)
```

Arduino가 위 형식으로 순차 측정 결과를 보내도록 펌웨어를 맞춰야 한다. 이 프로젝트는 실제 센서 SKU와 연결 핀이 확정되지 않아 펌웨어·전기 배선을 자동 변경하지 않는다. MATLAB이 센서에 Trigger 명령을 보내는 방식이 아니라 Arduino가 자체 스케줄로 측정한다.

이 모드에서는 GUI의 가상 위치 설정이 비활성화된다. 실제 로봇을 직접 움직이지 않는다. 타깃을 측정한 좌표에 정지시킨 실험에서만 `cfg.staticGT` 및 `staticGTConfirmed`를 설정한다. 이동 GT 자동 계측(카메라)은 아직 포함하지 않는다.

실제 빈 공간 배경 보정 전에는 synthetic 배경값으로 정상 데이터를 제거하지 않도록 `launchUltra`가 배경 필터를 비활성화한다. 보정된 배경을 `cfg.background`에 m 단위로 넣고 `liveWallCalibrated=true`로 설정하면 켜진다. 공통 ROI 필터는 계속 적용된다. 실측 계수 `cfg.alpha`/`cfg.beta`와 ROI도 실험 전에 맞춘다.

실시간 수신은 완성된 줄만 처리하며 CSV 일부가 도착했다고 `readline`으로 모델을 멈추지 않는다. Arduino 시각으로 알고리즘을 실행한다. USB 입력 지연·PC 실행 속도는 별도 문제이며 Pacing은 실시간 보장이 아니다. Serial Monitor가 같은 포트를 열고 있으면 닫는다.

## 검증

```matlab
results = runtests('testUltra.m');
assertSuccess(results)
run('validateProject.m') % 단위 테스트 + .slx 생성/실행 + 미리보기
```

실제 하드웨어 수신·감지율·배경 분포는 센서를 연결한 후 별도 검증해야 한다. 미측정 감지범위를 구현으로 채웠다고 간주하지 않는다.
