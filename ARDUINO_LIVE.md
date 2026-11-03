# launchUltra에서 실제 Arduino 연결

첨부한 4센서 Arduino 스케치(24열 CSV)를 그대로 사용한다. Arduino 코드는 변경하지 않았다.

1. Arduino를 USB에 연결하고 업로드를 마친다. Serial Monitor/Plotter는 닫는다.
2. MATLAB Current Folder를 capstone_design으로 설정하고 기존 실행을 중지한다.
3. `launchUltra`를 실행한다. 상단 **Connect Arduino (USB)**를 누르고 실제 포트를 선택한다.
4. 연결이 완료되어 **Ready**가 나타날 때까지 기다린다. 통신 속도는 115200이다.
5. 타깃을 정지 상태로 놓고 **X / GT [m], Y / GT [m]**에 실제 중심 좌표를 입력한다.
6. **Measure 30**을 누른다. 공간 치수 G 명령의 응답을 확인한 뒤 P1 명령을 보내 30회 측정한다.
7. 화면의 거리 4개와 위치추정 결과가 매 순회 갱신된다. 완료 후 다른 위치에서 다시 Measure 30을 누를 수 있다.
8. **Disconnect** 또는 창 닫기로 종료하면 results 폴더에 원문 CSV, MAT 및 전체/실험별 오차 요약 CSV가 저장된다.

실시간 모드는 같은 화면과 위치추정 함수를 사용하며 PC 타이머로 USB를 수신한다. Simulink Run을 별도로 누를 필요가 없다. 가상 입력으로 자동 전환하지 않는다. 재연결은 Disconnect 후 Connect를 누른다. Arduino 스케치에는 중간 측정 취소 기능이 없어 Disconnect 후에도 보드의 현재 30회 측정은 끝까지 진행될 수 있다.

## 배선과 좌표

| 센서 | 핀 | 좌표 [m] | 방향 |
|---|---|---|---|
| S1 | D8 | (0, 0.5) | 오른쪽 |
| S2 | D9 | (1, 0.5) | 왼쪽 |
| S3 | D10 | (0.5, 0) | 위쪽 |
| S4 | D7 | (0.5, 1) | 아래쪽 |

기본 공간은 1×1 m, ROI는 x/y 각각 0.4~0.6 m이다. 타깃 반지름은 예시 0.005 m이므로 실측값으로 ultraConfig에서 수정한다. 실제 측정 기준면이 벽에서 안쪽에 있으면 센서 좌표도 조정한다.

## 데이터 처리

- 스케치의 한 줄(24항목)을 센서 4개의 한 순회로 처리한다. cm는 m로 변환한다.
- OK는 Echo 수신 여부이며 타깃 반사가 확실하다는 뜻은 아니다. NaN/TIMEOUT/비정상 행을 검증하고 유효 거리 3개 미만이면 추정 실패로 남긴다.
- 센서별 실제 시작시각을 사용하며 micros 래핑을 처리한다. 표시 주기는 같은 실험 내 연속된 첫 센서 측정시각 차이다. 240 ms 고정으로 가정하지 않는다.
- Arduino의 GT 필드는 평가/표시 전용이다. 위치추정 함수에는 GT를 전달하지 않는다. 입력한 좌표로 로봇이 움직이지 않는다.
- 새 run 또는 보드 재시작 시 추적 상태를 초기화한다. 원문 및 외부 데이터 기록에 Arduino의 단순 추정값, Echo 시간도 보존한다.
- 38° 감지각 선은 실측 모드에서 참고용이다. 실제 거리를 GT나 가상 감지 판정으로 대체하지 않는다.
- 미측정한 가상의 벽 거리를 사용해 실제 데이터를 버리지 않도록, 실시간 연결 시 배경 필터는 기본 비활성화한다.
- 이번 작업의 검증은 합성 데이터와 모의 전송으로 수행했다. 실물 오차와 간섭은 실제 측정으로 확인해야 한다.

## 포트를 알고 있는 경우

```matlab
cfg = ultraConfig();
cfg.inputMode = 'serial';
cfg.serialPort = 'COM3'; % 실제 포트
launchUltra(cfg)
```

## 저장한 24항목 CSV 재생

```matlab
cfg = ultraConfig();
cfg.inputMode = 'replay';
cfg.inputFormat = 'experiment24';
cfg.replayFile = 'C:/실제경로/serial_experiment_날짜.csv';
cfg.background(:) = NaN;
launchUltra(cfg)
```

재생 시 Run을 누른다. 긴 로그는 cfg.stopTime을 늘린다. 기존 9항목 M,1 프로토콜은 inputFormat='legacy9'인 재생에서 계속 지원한다. 이전 대화의 가상 데이터 예제와 새 24항목 형식을 혼합하지 않는다.

## 연결 문제

- 포트 목록이 비면 USB 연결과 Serial Monitor/Plotter 점유를 확인한다.
- READY 시간 초과면 올바른 스케치/포트/115200 설정을 확인하고 다시 연결한다.
- Measure 30은 READY 상태에서만 가능하다. 30회 측정 중 추가 명령을 보내지 않는다.
- 창에 오래된 결과가 보이면 새 run 데이터 수신 여부와 상태 표시를 먼저 확인한다. 유효 센서 수와 실패 상태를 함께 판단한다.
