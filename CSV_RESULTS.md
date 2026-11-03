# GT 입력과 실험별 CSV 저장

## GT는 MATLAB 화면에서 입력한다

Arduino 연결 후 Ready 상태에서 `X / GT [m]`, `Y / GT [m]` 칸을 직접 수정한다. 값은 벽 기준 타깃 중심의 실제 위치이며 단위는 m이다. 예: 중앙은 X=0.5, Y=0.5. 현재 허용 영역은 각 축 0.4~0.6 m이다. GT로 로봇을 이동시키지 않는다.

Measure 30을 누르면 GT 입력을 잠그고 해당 위치에서 측정한다. 실험이 끝나 Ready로 돌아오면 다시 수정할 수 있다. 2.5 cm 센서 원점 변환은 내부적으로 처리되므로 사용자가 빼거나 더하지 않는다. GT는 추정기의 정답 입력으로 사용하지 않고 평가/표시에만 사용한다.

## 저장 위치와 시점

프로젝트의 `results`에 실험별로 두 파일을 저장한다. Arduino의 END 신호를 받으면 연결을 끊지 않아도 즉시 저장한다. 종료 신호 누락/연결 해제/창 닫기/새 실험 전환 시 수집한 부분을 INCOMPLETE로 저장한다. 같은 완료 실험은 연결 종료 시 중복 저장하지 않는다.

```
GT_X50.000cm_Y50.000cm_Run001_N30_20261103_195529_792_samples.csv
GT_X50.000cm_Y50.000cm_Run001_N30_20261103_195529_792_stats.csv
GT_X55.000cm_Y45.000cm_Run002_N28_INCOMPLETE_20261103_195529_792_samples.csv
```

날짜시각은 연결 세션의 시작 시각이다. Run은 해당 세션 내 실험 순번이며 보드의 run_id는 상세 CSV의 Arduino_Run_ID로 따로 기록한다. N은 수신·해석한 행 수이고 위치추정 성공 수가 아니다. 종료 신호, 예정 횟수, 측정 번호 연속성을 모두 확인해야 Complete=true이다. 기존 파일과 이름이 같으면 v2 등의 접미사로 구분하여 덮어쓰지 않는다. GT가 없거나 한 실험 안에서 서로 다르면 GT_UNKNOWN으로 표시한다.

화면에 파일명, 수신 횟수, LS 평균 좌표·표준편차·오차 통계를 표시한다. 해당 표시 위에 마우스를 올리면 두 파일의 전체 경로를 볼 수 있고 MATLAB 명령창에도 경로가 출력된다.

## samples.csv: 한 행 = 한 순회

- Run_ID, Arduino_Run_ID, Sample_ID, Time_s(추정 기준 시각), ScanPeriod_ms, GT_X_cm, GT_Y_cm, UsedSensor_N
- S1~S4 각각 RAW_cm(수신 거리), Echo_us, Time_us(Arduino micros 원값), Status, Used(추정 전 유효성 필터 통과), Corrected_cm(보정 거리)
- LS/LMedS/MotionLMedS 각각 X_cm, Y_cm, ErrorX_cm, ErrorY_cm, Error_cm, Success, Status
- Arduino_X_cm, Arduino_Y_cm: Arduino의 단순 계산 결과를 벽 기준으로 변환한 비교값

RAW는 MATLAB 보정·이상값 제외 전 값이다. 위치추정에서 제외해도 원래 수신 거리는 남긴다. TIMEOUT은 원문이 NaN이면 NaN을 보존한다. 센서의 아날로그 수신 파형은 수집하지 않는다. 위치추정 실패 행은 삭제하지 않고 해당 알고리즘의 좌표·오차를 NaN으로 저장한다.

## stats.csv: 알고리즘별 한 행

Expected_N, Received_N, Complete, Success_N, GT_Samples, Failure_Rate를 기록한다. 평균 좌표(Mean_X/Y_cm), 좌표 표준편차(Std_X/Y_cm), GT에 대한 축별 평균 오차(Bias_X/Y_cm), 위치오차 평균·표준편차(Mean_Error_cm, Std_Error_cm), RMSE_cm, Max_Error_cm를 기록한다.

좌표 통계는 해당 알고리즘이 성공한 유한 좌표만 사용한다. 오차 통계는 성공하고 GT도 있는 표본만 사용한다. 표준편차는 표본 표준편차(N-1)이며 유효 표본 2개 미만이면 NaN이다. 실패율은 수신 행 중 해당 알고리즘의 실패 비율이다. 수신 자체가 누락된 경우는 Expected_N과 Received_N 및 Complete로 구분한다.

S1~S4 RAW 거리의 Valid_N, Mean_cm, Std_cm도 기록한다. 이 통계는 OK이고 유한한 수신 거리로 계산하며, 위치추정에서 제외된 이상값도 포함한다. 센서 통계 열은 알고리즘 행마다 같은 값으로 반복한다.

기존 원문 serial_experiment CSV, 전체 요약 CSV 및 MAT 저장은 유지한다. 새 형식은 적용 이후 실행한 실험부터 생성되며 과거 결과 파일은 자동 변환하거나 덮어쓰지 않는다.
