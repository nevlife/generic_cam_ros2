# generic_cam

일반 USB 카메라(UVC)를 위한 ROS 2 Jazzy 드라이버다. 컴포넌트 노드 하나가 카메라 N대를 다룬다. 카메라 한 대는 N=1이다.

```text
/dev/videoN -> V4L2 mmap -> 프레임 검증과 진단 -> libjpeg 또는 GStreamer 디코더 -> BGR -> dds 또는 flux
```

## 실행

```bash
ros2 launch generic_cam generic_cam.launch.py
```

| launch 인자 | 기본값        | 뜻                                                                                                                             |
| ----------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `config`    | `generic_cam` | `config/<config>.param.yaml`를 읽는다. 책상의 C922는 `config:=c922`로 띄운다. 전송과 mirror를 포함한 설정은 이 파일에서 바꾼다 |
| `transport` | 빈 값         | 값을 주면 yaml의 `transport`를 덮어쓴다. 한 경로의 양 끝을 인자 하나로 맞출 때 쓴다                                            |

launch는 flux_components 컨테이너에 노드를 올린다. `ros2 run generic_cam generic_cam_node`로 단독 실행해도 발행은 같게 동작한다.

`/dev/videoN` 번호는 USB가 다시 잡히면 바뀐다. 시험 중에도 C922가 `/dev/video0`에서 `/dev/video1`로 옮겨 갔다. 고정 경로가 필요하면 udev 이름이나 `/dev/v4l/by-id/`를 `device`에 쓴다.

## 토픽

| 노드 안 이름                    | launch가 연결하는 이름         | 타입                              | 전송           |
| ------------------------------- | ------------------------------ | --------------------------------- | -------------- |
| `~/output/<카메라>/image_raw`   | `/camera/<카메라>/image_raw`   | `sensor_msgs/Image`               | `transport` 값 |
| `~/output/<카메라>/camera_info` | `/camera/<카메라>/camera_info` | `sensor_msgs/CameraInfo`          | 항상 dds       |
| `/diagnostics`                  | 그대로                         | `diagnostic_msgs/DiagnosticArray` | 항상 dds       |

- 이미지와 camera_info의 stamp는 같은 값이다.
- camera_info는 작은 메시지라서 flux로 보내지 않는다. flux_tools의 DepthCloud도 CameraInfo를 DDS에서 받는다.
- dds QoS는 reliable, volatile, depth 10이다. 구독자는 reliable로 받는다.
- mirror의 DDS QoS는 flux가 정한 best effort, depth 1이다. reliable 구독자는 이 토픽에 연결되지 않으므로 best effort로 구독한다. 시험에서 `ros2 topic hz`, `ros2 topic echo`, `ros2 topic bw`는 그대로 동작했다.

## 파라미터

코드에 기본값이 없다. 값이 빠지면 노드가 시작하지 못한다.

| 이름                        | 타입     | 뜻                                                                                          |
| --------------------------- | -------- | ------------------------------------------------------------------------------------------- |
| `cameras`                   | string[] | 카메라 이름 목록                                                                            |
| `transport`                 | string   | `dds` 또는 `flux`                                                                           |
| `flux_mirror_to_ros`        | bool     | flux 발행자의 `mirror_to_ros()`를 켠다                                                      |
| `flux_slot_count`           | int      | flux 링의 슬롯 수. 슬롯 크기는 이미지 크기로 계산한다                                       |
| `reconnect_ms`              | int      | 카메라가 끊기면 이 주기로 다시 연다. 0이면 끄고, 켜져 있으면 포맷과 control을 다시 적용한다 |
| `diagnostic_updater.period` | double   | 진단 주기(초). diagnostic_updater가 선언한다                                                |
| `<카메라>.device`           | string   | 장치 경로                                                                                   |
| `<카메라>.frame_id`         | string   | `header.frame_id`                                                                           |

아래 키는 `common.<키>`에 둔다. 카메라 블록에 같은 키를 쓰면 그 카메라만 덮어쓴다.

| 키                               | 타입   | 뜻                                                              |
| -------------------------------- | ------ | --------------------------------------------------------------- |
| `pixel_format`                   | string | `mjpeg` 또는 `yuyv`                                             |
| `decoder`                        | string | `libjpeg` 또는 GStreamer 디코더 요소 이름. MJPEG에만 쓴다       |
| `width`, `height`, `fps`         | int    | 장치에 요청할 형식. 장치가 다른 값으로 바꾸면 시작에 실패한다   |
| `rotation`                       | int    | 시계 방향 0, 90, 180, 270도. 0이 아니면 프레임을 한 번 복사한다 |
| `camera_info_path`               | string | 보정 yaml 경로. 빈 문자열이면 크기만 채운다                     |
| `stamp_offset_ns`                | int    | stamp에 더하는 값                                               |
| `frozen_frame_limit`             | int    | 내용이 같은 프레임이 이 수만큼 이어지면 frozen으로 판정한다     |
| `rate.min_ok`, `rate.max_ok`     | double | 이 범위 안이면 OK다                                             |
| `rate.min_warn`, `rate.max_warn` | double | 이 범위 안이면 WARN이고 밖이면 ERROR다                          |
| `rate.hysteresis_updates`        | int    | 상태를 바꾸기 전에 같은 판정을 몇 번 볼지 정한다                |
| `rate.immediate_error_report`    | bool   | ERROR를 hysteresis 없이 바로 낸다                               |
| `rate.immediate_relax_state`     | bool   | 나아진 상태를 hysteresis 없이 바로 낸다                         |

### V4L2 control

노드는 장치의 control을 열거해 `<카메라>.controls.<이름>`을 선언한다. 이름은 `v4l2-ctl -L`의 이름과 같다. 값은 정수이고 bool은 0과 1이다.

- yaml의 `<카메라>.controls.*`나 `common.controls.*`에 쓴 값만 장치에 쓴다. 쓰지 않은 control은 장치의 값을 그대로 둔다. 이 값을 파라미터의 초기값으로 쓰는 것이 코드 기본값 금지의 유일한 예외다.
- 값은 STREAMON 전후로 두 번 쓴다. STREAMON 뒤에 값을 잃는 ISP가 있기 때문이다.
- bool과 menu control을 정수 control보다 먼저 쓴다. 자동 모드가 켜져 있으면 수동 값이 쓰이지 않기 때문이다.
- `ros2 param set`으로 실행 중에 바꿀 수 있다. 범위 밖 값과 장치가 거절한 값은 실패로 돌아온다.
- 장치에 없는 이름은 WARN으로 남긴다. `common`을 카메라 여러 대가 함께 쓰기 때문이다.

## 시각

| 항목           | 내용                                                                                                                                                                       |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| stamp의 시계   | ROS 시계(`RCL_ROS_TIME`). `use_sim_time`이 꺼져 있으면 `CLOCK_REALTIME`이다                                                                                                |
| 원본 값        | `v4l2_buffer.timestamp`. uvcvideo에서는 `CLOCK_MONOTONIC`이다(`ts-monotonic`, `ts-src-soe`)                                                                                |
| 변환           | `timestamp + (CLOCK_REALTIME - CLOCK_MONOTONIC) + stamp_offset_ns`. 차이는 프레임마다 다시 읽으므로 시스템 시각이 바뀌어도 따라간다                                        |
| 뜻             | 드라이버가 프레임의 첫 USB 패킷을 처리한 시각이다. 노출 시각이 아니다. 실제 노출은 이보다 앞서고 그 차이는 재지 못했다. 아는 값이 있으면 `stamp_offset_ns`에 음수로 넣는다 |
| 알갱이         | uvcvideo는 URB가 끝날 때 패킷을 처리한다. URB 하나가 32패킷(4 ms)이라서 stamp 간격이 32 ms와 36 ms로 나온다. 표준편차는 1.9 ms다                                           |
| `use_sim_time` | stamp는 시뮬레이션 시각이 되지 않는다. 카메라 입력은 실제 시간이기 때문이다                                                                                                |

시계 검증은 `latency_probe`로 했다. 구독자가 `node->now()`에서 stamp를 뺀 값은 모든 프레임에서 양수이고, flux에서 11.6 ms에서 16.9 ms 사이다. 같은 시계에 있다는 뜻이다. `/diagnostics`의 `stamp_age_at_publish_*`가 같은 값을 실행 중에 보여 준다.

이전 코드는 디코딩 뒤의 `now()`를 stamp로 썼다. 그 값에는 위 지연 약 12 ms가 빠져 있었고, 프레임 하나가 늦으면 stamp도 그만큼 밀렸다(이전 코드 측정 max 61.6 ms 간격, 현재 36.1 ms).

## 진단

카메라마다 `generic_cam: <카메라>` 상태를 낸다.

| 검사         | 수준                                | 근거                                                                                          |
| ------------ | ----------------------------------- | --------------------------------------------------------------------------------------------- |
| 연결         | ERROR                               | 장치가 끊기면 바로 `camera disconnected`를 낸다. 값으로 `connected`, `reconnects`를 함께 낸다 |
| 프레임률     | OK, WARN, ERROR, 첫 프레임 전 STALE | 진단 주기 동안 캡처 시각으로 잰 평균. hysteresis를 거친다                                     |
| 순서         | ERROR                               | sequence나 timestamp가 늘지 않았다                                                            |
| frozen       | ERROR                               | 버퍼 내용의 해시가 `frozen_frame_limit`번 이어서 같다                                         |
| sequence gap | WARN                                | sequence가 건너뛰었다. 노드가 밀려 오래된 프레임을 건너뛴 경우도 여기에 든다                  |
| 버퍼 오류    | WARN                                | `V4L2_BUF_FLAG_ERROR`. 이 프레임은 발행하지 않는다                                            |
| 프레임 검증  | WARN                                | 크기가 다르거나, JPEG가 깨졌거나, 버퍼가 짧다. 이 프레임은 발행하지 않는다                    |
| 발행 실패    | WARN                                | flux의 빈 슬롯이 없다                                                                         |

frozen은 내용만 본다. 렌즈를 완전히 가려 모든 값이 같아지면 정상 카메라도 frozen으로 나올 수 있다.

장치가 사라지면 ERROR 로그를 남기고 그 카메라의 캡처를 끝낸다. 진단은 프레임률 ERROR로 바뀐다. 자동으로 다시 연결하지 않는다.

## flux

- 슬롯을 먼저 빌리고 디코더가 슬롯에 바로 쓴다. `rotation: 0`과 `decoder: libjpeg`에서 memcpy가 없다. 회전과 GStreamer 디코더에서는 한 번 복사한다.
- flux에는 ROS로 넘어가는 자동 경로가 없다. ROS 도구로 보려면 `flux_mirror_to_ros`를 켜거나 `flux_bridge`를 띄운다.

## 측정

조건은 C922, 1920x1080 MJPEG 30 fps, i7-14700K, Cyclone DDS다. 노드는 CPU 8-9, `latency_probe`는 CPU 10-11에 고정했다. 격리 코어는 아니고 다른 프로세스가 함께 돌았다. 앞 30프레임을 뺀 1770프레임을 썼다.

재현 방법은 다음과 같다. `latency_probe`는 시험 빌드에서만 설치된다.

```bash
ros2 run generic_cam latency_probe flux /camera/c922/image_raw 1800 out.csv
```

단위는 ms다. capture는 stamp이고, publish는 flux의 commit 시각과 DDS의 source timestamp다.

| 경로          | 구간                   | p50   | p90   | p99   | max   |
| ------------- | ---------------------- | ----- | ----- | ----- | ----- |
| dds           | capture -> publish     | 12.16 | 13.18 | 16.10 | 18.82 |
| dds           | capture -> 구독자      | 15.44 | 16.99 | 21.18 | 58.03 |
| flux          | capture -> publish     | 11.80 | 12.67 | 15.81 | 16.88 |
| flux          | capture -> 구독자      | 11.82 | 12.71 | 15.84 | 16.90 |
| flux + mirror | capture -> flux 구독자 | 11.90 | 12.82 | 14.06 | 17.43 |
| flux + mirror | capture -> DDS 구독자  | 16.61 | 17.80 | 19.52 | 22.00 |

- dds는 실행마다 꼬리가 크게 다르다. 같은 코드의 앞선 실행에서 구독자까지 p99는 59.87 ms, max는 101.91 ms였다.
- mirror의 DDS 구독자는 best effort라서 1813프레임 가운데 62곳에서 프레임을 잃었고 평균 28.96 Hz였다.
- capture -> publish의 약 12 ms 가운데 디코딩은 3.6 ms다. 나머지는 USB 전송과 드라이버 처리로 본다. 따로 재지는 않았다.
- 노드의 CPU 사용량은 코어 하나 기준으로 dds 19.6%, flux 11.9%, flux와 mirror 구독자 21.9%, 이전 코드 24.0%다.

### 디코더 평가

C922의 MJPEG는 4:2:2다. 같은 프레임 60장을 20번 반복해 한 장씩 동기로 디코딩했다.

| 디코더           | 프레임         | p50 ms | p99 ms | CPU ms/프레임 | 결과                                                                                              |
| ---------------- | -------------- | ------ | ------ | ------------- | ------------------------------------------------------------------------------------------------- |
| libjpeg(기본)    | 4:2:2          | 3.6    | 4.8    | 3.7           | 슬롯에 바로 쓴다                                                                                  |
| jpegdec          | 4:2:2          | 4.5    | 4.7    | 4.5           | 사용 가능                                                                                         |
| vajpegdec(Intel) | 4:2:2          | 5.4    | 6.9    | 4.3           | 사용 가능                                                                                         |
| nvjpegdec        | 4:2:2          |        |        |               | 실패. `unhandled chroma format 2`                                                                 |
| libjpeg          | 4:2:0 재인코딩 | 3.0    | 4.1    | 3.0           |                                                                                                   |
| vajpegdec        | 4:2:0 재인코딩 | 4.8    | 6.1    | 3.9           |                                                                                                   |
| nvjpegdec        | 4:2:0 재인코딩 | 36.6   | 39.5   | 5.4           | 프레임 하나를 쥐고 내보낸다. Python 시험 도구에서 같은 조건의 jpegdec는 3.6 ms, 3.9 ms/프레임이다 |

노드에서 flux로 재면 capture -> 구독자 p50과 CPU는 libjpeg 11.8 ms와 11.6%, jpegdec 12.6 ms와 13.9%, vajpegdec 14.0 ms와 14.5%다.

결론은 다음과 같다.

- 기본값은 `libjpeg`로 둔다. GPU가 필요 없고 지연과 CPU가 모두 가장 낮다.
- 하드웨어 디코더는 이 PC에서 이득이 없다. 결과를 시스템 메모리로 옮겨 BGR로 바꾸는 비용까지 더하면 소프트웨어보다 느리다.
- `nvjpegdec`는 쓰지 않는다. 이 노드는 프레임마다 결과를 기다리므로 프레임을 쥐는 요소와 맞지 않는다. 4:2:2에서는 캡처가 ERROR로 끝나고, 4:2:0에서는 모든 프레임이 시간 초과로 버려질 것이다. 4:2:0 노드 실행은 시험하지 않았다.
- ffmpeg의 `mjpeg_cuvid`는 4:2:2를 디코딩했다. 처리량 실행에서 CPU 2.5 ms/프레임이었지만 출력이 NV12라서 BGR 변환이 더 든다. libavcodec 의존성을 더할 이유가 되지 않는다.

## 시험

```bash
colcon test --packages-select generic_cam
```

단위 시험은 카메라 없이 돈다. 프레임률 판정과 hysteresis, sequence gap, frozen, JPEG와 YUYV 디코딩, 깨진 프레임, 회전, control 이름, 시계 변환을 다룬다.

C922로 확인하지 못한 것은 다음과 같다.

- 카메라 2대 이상의 동시 실행. 설정 파일이 launch에서 풀리는 데까지만 확인했다.
- frozen, 버퍼 오류 플래그, 순서 오류의 실제 발생. 단위 시험으로만 확인했다.
- `flux_bridge`. 이 패키지는 `generic_cam`의 의존성이 아니다.
- stamp와 실제 노출 시각의 차이.
