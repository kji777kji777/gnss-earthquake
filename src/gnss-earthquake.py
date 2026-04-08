import socket
import base64
import time
import math
import csv
import os
from collections import Counter, deque
from datetime import datetime

# =============================================================================
# [1] 설정 정보
# =============================================================================
HOST            = 'igs-ip.net'
PORT            = 2101
MOUNTPOINT      = 'GMSD00JPN0'
USER            = '{igs-ip.net 계정}'
PASSWORD        = '{igs-ip.net 비번}'

# 데이터 저장 파일명 (실행 날짜별로 생성)
LOG_FILE        = f"gnss_log_{MOUNTPOINT}_{datetime.now().strftime('%Y%m%d')}.csv"
ALERT_LOG_FILE  = f"gnss_alert_{MOUNTPOINT}_{datetime.now().strftime('%Y%m%d')}.csv"

# =============================================================================
# [2] 파라미터 설정
# =============================================================================
THRESHOLD_MM        = 30.0   # 경보 임계값 (mm)
INIT_SAMPLES        = 30     # 기준점 확정에 필요한 초기 샘플 수 (약 5분치)
SMOOTHING_WINDOW    = 10     # 이동 평균 필터 윈도우 크기
ALERT_CONSEC_COUNT  = 3      # 연속 N회 초과 시 경보 발령 (오경보 방지)
MAX_INTERVAL_SEC    = 30     # 이 시간(초) 이상 수신 없으면 데이터 누락 경고
MAX_BUFFER_BYTES    = 65536  # 버퍼 최대 크기 (64KB), 초과 시 리셋
MAX_RECONNECT_TRY   = 5      # 자동 재연결 최대 시도 횟수


# =============================================================================
# [3] CRC-24Q 검증 함수 (RTCM 표준)
# =============================================================================
def crc24q(data: bytes) -> int:
    """RTCM 3.x CRC-24Q 계산"""
    crc = 0
    for byte in data:
        crc ^= (byte << 16)
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


# =============================================================================
# [4] GeodeticTracker 클래스
# =============================================================================
class GeodeticTracker:
    def __init__(self, station_name, threshold_mm=THRESHOLD_MM):
        self.station_name       = station_name
        self.threshold          = threshold_mm / 1000.0     # mm → m
        self.anchor_xyz         = None
        self.init_samples       = INIT_SAMPLES
        self.temp_coords        = []
        self.last_msg_time      = None
        self.displacement_buf   = deque(maxlen=SMOOTHING_WINDOW)  # 이동 평균용
        self.consec_warning     = 0                               # 연속 경보 카운터
        self.total_frames       = 0
        self.crc_fail_count     = 0

        # CSV 메인 로그 초기화
        if not os.path.exists(LOG_FILE):
            with open(LOG_FILE, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'Timestamp', 'X', 'Y', 'Z',
                    'Raw_Disp_mm', 'Smooth_Disp_mm',
                    'Interval_sec', 'Status'
                ])

        # CSV 경보 로그 초기화
        if not os.path.exists(ALERT_LOG_FILE):
            with open(ALERT_LOG_FILE, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'Timestamp', 'X', 'Y', 'Z',
                    'Smooth_Disp_mm', 'Consec_Count'
                ])

    # -------------------------------------------------------------------------
    # RTCM 1006 디코딩 (비트 오프셋 버그 수정)
    # -------------------------------------------------------------------------
    def decode_1006(self, payload: bytes):
        """
        RTCM 1006 페이로드에서 ECEF X, Y, Z 좌표 추출
        
        RTCM 1006 비트 구조 (총 169 bits + Antenna Height 16bits):
          [  0- 11] Message Number        (12 bits)
          [ 12- 23] Station ID            (12 bits)
          [ 24- 29] Reserved/ITRF Year    ( 6 bits)
          [ 30- 32] Indicator flags       ( 3 bits)
          [ 33- 35] Reserved              ( 3 bits)
          [ 36- 73] ECEF-X               (38 bits, 0.1mm 단위, 부호 있음)
          [ 74- 74] Oscillator Type       ( 1 bit )
          [ 75- 75] Reserved              ( 1 bit )
          [ 76-113] ECEF-Y               (38 bits, 0.1mm 단위, 부호 있음)
          [114-114] Reserved              ( 1 bit )
          [115-152] ECEF-Z               (38 bits, 0.1mm 단위, 부호 있음)
          [153-168] Antenna Height        (16 bits)
        """
        b = int.from_bytes(payload, byteorder='big')
        total_bits = len(payload) * 8

        def extract_signed_38(bit_offset: int) -> float:
            shift = total_bits - bit_offset - 38
            if shift < 0:
                raise ValueError(f"페이로드 길이 부족: offset={bit_offset}, total={total_bits}")
            raw = (b >> shift) & 0x3FFFFFFFFF
            if raw & 0x2000000000:          # 부호 비트 처리 (2의 보수)
                raw -= 0x4000000000
            return raw * 0.0001             # 0.1mm → m

        x = extract_signed_38(36)
        y = extract_signed_38(76)
        z = extract_signed_38(115)
        return x, y, z

    # -------------------------------------------------------------------------
    # 좌표 업데이트 및 변위 계산
    # -------------------------------------------------------------------------
    def update(self, x, y, z):
        """
        좌표 업데이트, 수신 간격 체크, 이동 평균 변위 반환
        Returns: (raw_dist, smoothed_dist, interval)  |  기준점 미확정 시 (None, None, interval)
        """
        now = time.time()
        interval = (now - self.last_msg_time) if self.last_msg_time else 0
        self.last_msg_time = now

        # 수신 간격 이상 감지
        if self.last_msg_time and interval > MAX_INTERVAL_SEC:
            print(f"\n[!] 경고: 수신 간격 이상 ({interval:.1f}s) — 데이터 누락 가능")

        current_xyz = (x, y, z)

        # --- 기준점(Anchor) 수집 단계 ---
        if self.anchor_xyz is None:
            self.temp_coords.append(current_xyz)
            remaining = self.init_samples - len(self.temp_coords)
            print(f"\r[초기화] 기준 샘플 수집 중... {len(self.temp_coords)}/{self.init_samples} "
                  f"(남은 샘플: {remaining})", end="", flush=True)

            if len(self.temp_coords) >= self.init_samples:
                n = self.init_samples
                self.anchor_xyz = (
                    sum(c[0] for c in self.temp_coords) / n,
                    sum(c[1] for c in self.temp_coords) / n,
                    sum(c[2] for c in self.temp_coords) / n,
                )
                print(f"\n\n[✔] {self.station_name} 기준 좌표 확정:")
                print(f"    X={self.anchor_xyz[0]:.4f}  "
                      f"Y={self.anchor_xyz[1]:.4f}  "
                      f"Z={self.anchor_xyz[2]:.4f}\n")
            return None, None, interval

        # --- 유클리드 거리 계산 ---
        raw_dist = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(self.anchor_xyz, current_xyz))
        )

        # --- 이동 평균 필터 적용 ---
        self.displacement_buf.append(raw_dist)
        smoothed = sum(self.displacement_buf) / len(self.displacement_buf)

        # --- 연속 경보 카운터 관리 ---
        if smoothed >= self.threshold:
            self.consec_warning += 1
        else:
            self.consec_warning = 0

        return raw_dist, smoothed, interval

    # -------------------------------------------------------------------------
    # 경보 발령 판단
    # -------------------------------------------------------------------------
    def is_alert(self) -> bool:
        """연속 N회 임계값 초과 시 True"""
        return self.consec_warning >= ALERT_CONSEC_COUNT

    # -------------------------------------------------------------------------
    # 데이터 아카이빙
    # -------------------------------------------------------------------------
    def archive(self, x, y, z, raw_dist, smoothed, interval):
        """메인 로그 CSV에 기록"""
        raw_mm      = (raw_dist  * 1000) if raw_dist  is not None else 0
        smooth_mm   = (smoothed  * 1000) if smoothed  is not None else 0
        status      = "INIT"
        if raw_dist is not None:
            if self.is_alert():
                status = "ALERT"
            elif smoothed >= self.threshold:
                status = "WARNING"
            else:
                status = "OK"

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        with open(LOG_FILE, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                f"{raw_mm:.2f}", f"{smooth_mm:.2f}",
                f"{interval:.2f}", status
            ])

    def archive_alert(self, x, y, z, smoothed):
        """경보 발령 시 별도 alert 로그에 기록"""
        smooth_mm = smoothed * 1000 if smoothed is not None else 0
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        with open(ALERT_LOG_FILE, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                f"{smooth_mm:.2f}", self.consec_warning
            ])


# =============================================================================
# [5] RTCM 프레임 파싱 (CRC 검증 추가)
# =============================================================================
def parse_rtcm_frames(data: bytes, msg_counter: Counter, tracker: GeodeticTracker):
    i = 0
    found = []

    while i < len(data) - 5:
        if data[i] != 0xD3:        # RTCM 3.x 프리앰블
            i += 1
            continue

        length = ((data[i + 1] & 0x03) << 8) | data[i + 2]
        end    = i + 3 + length + 3

        if end > len(data):
            break                   # 아직 데이터 부족 — 다음 청크 대기

        # --- CRC-24Q 검증 ---
        frame_data   = data[i : i + 3 + length]
        crc_received = int.from_bytes(data[i + 3 + length : end], 'big')
        crc_calc     = crc24q(frame_data)

        tracker.total_frames += 1
        if crc_calc != crc_received:
            tracker.crc_fail_count += 1
            print(f"\n[!] CRC 불일치 — 프레임 스킵 "
                  f"(누적 실패: {tracker.crc_fail_count}/{tracker.total_frames})")
            i += 1
            continue

        # --- 메시지 타입 추출 ---
        payload  = data[i + 3 : i + 3 + length]
        msg_type = ((payload[0] << 4) | (payload[1] >> 4)) & 0xFFF
        msg_counter[msg_type] += 1

        # --- RTCM 1006 처리 ---
        if msg_type == 1006:
            try:
                x, y, z = tracker.decode_1006(payload)
                raw_dist, smoothed, interval = tracker.update(x, y, z)

                # 아카이빙
                tracker.archive(x, y, z, raw_dist, smoothed, interval)

                if raw_dist is not None:
                    alert   = tracker.is_alert()
                    s_mm    = smoothed * 1000
                    r_mm    = raw_dist * 1000
                    status  = "🚨 ALERT"   if alert                        else \
                              "⚠️  WARNING" if smoothed >= tracker.threshold else \
                              "OK"

                    print(f"\n[STATION] {tracker.station_name} | 간격: {interval:.2f}s")
                    print(f"[COORD  ] X:{x:.4f}  Y:{y:.4f}  Z:{z:.4f}")
                    print(f"[RESULT ] 원시변위: {r_mm:.2f}mm | "
                          f"평균변위: {s_mm:.2f}mm | 상태: {status}")

                    # 경보 발령
                    if alert:
                        print(f"\n{'='*60}")
                        print(f"  🚨 지진 경보 발령! 연속 {tracker.consec_warning}회 임계값 초과")
                        print(f"  평균 변위: {s_mm:.2f} mm  (기준: {THRESHOLD_MM:.0f} mm)")
                        print(f"{'='*60}\n")
                        tracker.archive_alert(x, y, z, smoothed)

            except Exception as e:
                print(f"\n[!] 1006 파싱 에러: {e}")

        found.append((msg_type, i, length))
        i = end

    return found


# =============================================================================
# [6] NTRIP 연결 (자동 재연결 포함)
# =============================================================================
def connect_ntrip():
    tracker = GeodeticTracker(MOUNTPOINT, threshold_mm=THRESHOLD_MM)
    auth    = base64.b64encode(f"{USER}:{PASSWORD}".encode()).decode()

    ntrip_request = (
        f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
        f"User-Agent: NTRIP PythonClient/2.0\r\n"
        f"Authorization: Basic {auth}\r\n"
        f"Connection: close\r\n\r\n"
    )

    # GMSD (게센누마) 기준국 근처 좌표
    gga_message = "$GPGGA,000001,3033.000,N,13100.600,E,1,12,1.0,0.0,M,0.0,M,,*60\r\n"

    reconnect_count = 0

    while reconnect_count <= MAX_RECONNECT_TRY:
        if reconnect_count > 0:
            wait = min(5 * reconnect_count, 30)   # 최대 30초 대기 (지수 백오프)
            print(f"\n[재연결] {wait}초 후 재시도... ({reconnect_count}/{MAX_RECONNECT_TRY})")
            time.sleep(wait)

        buffer      = b""
        msg_counter = Counter()
        header_done = False
        chunk_count = 0

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(15)
                s.connect((HOST, PORT))
                s.sendall(ntrip_request.encode())
                s.sendall(gga_message.encode())

                print(f"[*] {MOUNTPOINT} 연결 성공 (시도 #{reconnect_count + 1})")
                print(f"[*] 메인 로그:  {LOG_FILE}")
                print(f"[*] 경보 로그:  {ALERT_LOG_FILE}")
                print(f"[*] 임계값:     {THRESHOLD_MM:.0f} mm | "
                      f"초기 샘플: {INIT_SAMPLES}개 | "
                      f"경보 연속: {ALERT_CONSEC_COUNT}회\n")

                reconnect_count = 0     # 정상 연결 시 카운터 리셋

                while True:
                    try:
                        chunk = s.recv(4096)
                    except socket.timeout:
                        print("\n[!] 수신 타임아웃 — 재연결 시도")
                        break
                    if not chunk:
                        print("\n[!] 서버 연결 종료 — 재연결 시도")
                        break

                    buffer += chunk

                    # --- HTTP 헤더 처리 ---
                    if not header_done:
                        if b"\r\n\r\n" in buffer or b"ICY 200 OK" in buffer:
                            header_done = True
                            # 헤더 이후 남은 데이터만 유지
                            sep = buffer.find(b"\r\n\r\n")
                            buffer = buffer[sep + 4:] if sep != -1 else b""
                            print("[*] RTCM 데이터 스트림 수신 중...\n")
                        elif b"401" in buffer or b"403" in buffer:
                            print("[!] 인증 실패. USER/PASSWORD를 확인하세요.")
                            return
                        continue

                    # --- 버퍼 크기 제한 ---
                    if len(buffer) > MAX_BUFFER_BYTES:
                        print(f"\n[!] 버퍼 초과 ({len(buffer)} bytes) — 버퍼 리셋")
                        buffer = b""
                        continue

                    # --- 프레임 파싱 ---
                    chunk_count += 1
                    frames = parse_rtcm_frames(buffer, msg_counter, tracker)

                    if frames:
                        last_frame = frames[-1]
                        last_end   = last_frame[1] + 3 + last_frame[2] + 3
                        buffer     = buffer[last_end:]

                    # --- 상태 요약 출력 (상위 5개 메시지) ---
                    if msg_counter:
                        top5 = sorted(msg_counter.items(), key=lambda x: x[1], reverse=True)[:5]
                        summary = " | ".join(f"MSG{k}:{v}" for k, v in top5)
                        crc_info = (f" | CRC실패:{tracker.crc_fail_count}"
                                    if tracker.crc_fail_count else "")
                        print(f"\r[#{chunk_count:5}] {summary}{crc_info} ",
                              end="", flush=True)

        except KeyboardInterrupt:
            print(f"\n\n[*] 사용자 중단. 최종 데이터 → {LOG_FILE}")
            return
        except ConnectionRefusedError:
            print(f"\n[!] 연결 거부됨: {HOST}:{PORT}")
        except socket.gaierror:
            print(f"\n[!] 호스트 조회 실패: {HOST}")
        except Exception as e:
            print(f"\n[!] 연결 에러: {e}")

        reconnect_count += 1

    print(f"\n[!] 최대 재연결 횟수({MAX_RECONNECT_TRY}회) 초과. 프로그램 종료.")


# =============================================================================
# [7] 진입점
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print(f"  GNSS 지진 변위 모니터 — {MOUNTPOINT}")
    print(f"  시작 시각: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60 + "\n")
    connect_ntrip()
