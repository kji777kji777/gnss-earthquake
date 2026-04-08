# 🛰️ GNSS 실시간 지진 변위 감지 모니터

> **일본 GNSS 기준국 데이터를 실시간으로 수신·분석하여 지각 변위를 감지하는 지진 조기경보 프로토타입**

---

## 📌 목차

1. [프로그램 목적](#-프로그램-목적)  
2. [전체 구성 및 동작 흐름](#-전체-구성-및-동작-흐름)  
3. [핵심 알고리즘 상세 설명](#-핵심-알고리즘-상세-설명)  
4. [정확도 및 한계점](#-정확도-및-한계점)  
5. [설치 방법 (초심자용)](#-설치-방법-초심자용)  
6. [실행 방법](#-실행-방법)  
7. [출력 결과 해석](#-출력-결과-해석)  
8. [파라미터 튜닝 가이드](#-파라미터-튜닝-가이드)  

---

## 🎯 프로그램 목적

대규모 지진이 발생하기 직전, 지각은 수 mm ~ 수 cm 단위의 **미세한 수평·수직 변위**를 일으킵니다.  
이 프로그램은 일본 내 **GNSS(GPS) 기준국**에서 브로드캐스트되는 **RTCM 실시간 데이터**를 직접 수신하고,  
기준 좌표(Anchor)로부터의 **3차원 변위량**을 밀리미터 단위로 연속 추적합니다.

| 항목 | 내용 |
|------|------|
| 📡 데이터 소스 | IGS NTRIP Caster (`igs-ip.net`) |
| 📍 감시 기준국 | `GMSD00JPN0` (일본 미야기현 게센누마 인근) |
| 📐 측위 좌표계 | ECEF (지구 중심 직교 좌표계, X/Y/Z) |
| ⚡ 수신 주기 | 약 10초 간격 (RTCM 1006 메시지 기준) |
| 🚨 경보 방식 | 이동 평균 변위가 **임계값(기본 30mm)** 을 연속 초과 시 알림 |

---

## 🗂️ 전체 구성 및 동작 흐름

```
gnss-earthquake.py
│
├── [1] 설정 정보          ← NTRIP 서버 주소, 계정, 마운트포인트
├── [2] 파라미터           ← 임계값, 초기 샘플 수, 경보 조건 등
├── [3] crc24q()           ← RTCM 프레임 무결성 검증 함수
├── [4] GeodeticTracker    ← 핵심 클래스 (좌표 추적 + 변위 계산 + 경보)
├── [5] parse_rtcm_frames()← RTCM 바이너리 스트림 파싱
├── [6] connect_ntrip()    ← NTRIP 서버 연결 + 자동 재연결 루프
└── [7] __main__           ← 프로그램 진입점
```

### 🔄 전체 동작 흐름

```
프로그램 시작
    │
    ▼
[NTRIP 서버 연결] ─── 실패 시 자동 재연결 (최대 5회, 지수 백오프)
    │
    ▼
[HTTP/ICY 헤더 파싱] → RTCM 바이너리 스트림 수신 시작
    │
    ▼
[CRC-24Q 검증] ── 불일치 프레임은 스킵
    │
    ▼
[RTCM 1006 메시지 감지] → ECEF 좌표(X, Y, Z) 디코딩
    │
    ▼
[기준점(Anchor) 수집] ── 초기 30샘플 평균 → 기준 좌표 확정
    │
    ▼
[3D 유클리드 변위 계산] → 이동 평균 필터 (10샘플)
    │
    ▼
[경보 판단] ── 연속 3회 이상 임계값 초과 → 🚨 ALERT
    │
    ▼
[CSV 로그 저장] ── 메인 로그 / 경보 로그 분리 기록
```

---

## 🔬 핵심 알고리즘 상세 설명

### 1️⃣ RTCM 1006 메시지 디코딩

RTCM(Radio Technical Commission for Maritime) 3.x 프로토콜의 **1006번 메시지**는  
GNSS 기준국의 **ECEF 3차원 절대 좌표**를 0.1mm(= 0.0001m) 정밀도로 담고 있습니다.

```
RTCM 1006 비트 레이아웃 (총 169 bits)
┌──────────┬──────────┬────────┬───────┬──────────────────────┐
│ Msg No.  │ Sta. ID  │ ITRF   │ Flags │     ECEF-X (38 bit)  │
│ 12 bits  │ 12 bits  │ 6 bits │ 6bit  │   부호있음, 0.1mm 단위 │
├──────────┴──────────┴────────┴───────┴──────────────────────┤
│  [Osc]  │ ECEF-Y (38 bits)  │ [Rsv] │ ECEF-Z (38 bits)     │
│         │   부호있음, 0.1mm  │       │  부호있음, 0.1mm      │
├─────────┴───────────────────┴───────┴──────────────────────┤
│              Antenna Height (16 bits)                       │
└─────────────────────────────────────────────────────────────┘
```

```python
# 2의 보수(Two's Complement)로 부호 처리 후 단위 변환
raw = (big_integer >> shift) & 0x3FFFFFFFFF   # 38비트 추출
if raw & 0x2000000000:                         # 최상위 비트가 1 → 음수
    raw -= 0x4000000000
coordinate = raw * 0.0001                      # 0.1mm → m 변환
```

---

### 2️⃣ CRC-24Q 무결성 검증

네트워크 노이즈나 패킷 손실로 인한 잘못된 데이터가 분석에 섞이지 않도록,  
모든 RTCM 프레임에 대해 **CRC-24Q** 체크섬 검증을 수행합니다.

```python
def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= (byte << 16)
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB   # CRC-24Q 생성 다항식
    return crc & 0xFFFFFF
```

> ✅ 검증 실패 프레임은 자동으로 스킵하고, 누적 실패율을 실시간 표시합니다.

---

### 3️⃣ 기준 좌표(Anchor) 확정

단일 순간의 좌표를 기준으로 삼으면 **측위 노이즈**에 취약합니다.  
따라서 초기 **30샘플(약 5분치)의 평균값**을 기준 좌표로 설정합니다.

```
기준 좌표 = (ΣX / 30,  ΣY / 30,  ΣZ / 30)
```

---

### 4️⃣ 3차원 유클리드 변위 계산

기준 좌표에서 현재 좌표까지의 **직선 거리(3D)**를 계산합니다.

```
변위(m) = √[(X_현재 - X_기준)² + (Y_현재 - Y_기준)² + (Z_현재 - Z_기준)²]
```

이 방식은 수평·수직 방향을 구분하지 않고 **총 변위량**을 한 번에 포착합니다.

---

### 5️⃣ 이동 평균 필터 (노이즈 제거)

순간 측위 오차(수mm~수cm)로 인한 **오경보를 방지**하기 위해,  
최근 10개 샘플의 이동 평균(Moving Average)을 적용합니다.

```
평균 변위 = (D_t + D_t-1 + ... + D_t-9) / 10
```

```python
self.displacement_buf = deque(maxlen=10)  # 자동으로 오래된 값 제거
smoothed = sum(self.displacement_buf) / len(self.displacement_buf)
```

---

### 6️⃣ 연속 초과 경보 로직

이동 평균이 임계값을 초과해도 **1회만으로는 경보를 발령하지 않습니다.**  
**연속 3회 이상 초과** 시에만 경보를 발령하여 순간적 스파이크로 인한 오경보를 최소화합니다.

```
측정 1회차: 평균 변위 35mm > 30mm → 연속 카운터 = 1 (경고만)
측정 2회차: 평균 변위 38mm > 30mm → 연속 카운터 = 2 (경고만)
측정 3회차: 평균 변위 41mm > 30mm → 연속 카운터 = 3 → 🚨 ALERT 발령!
```

```
상태 단계: INIT → OK → ⚠️ WARNING (1~2회) → 🚨 ALERT (3회 이상)
```

---

## ⚖️ 정확도 및 한계점

### ✅ 잘 된 부분

| 항목 | 내용 |
|------|------|
| 실시간 수신 | NTRIP 프로토콜로 공개 GNSS 데이터 직접 수신 |
| 데이터 무결성 | CRC-24Q 검증으로 손상 데이터 자동 필터링 |
| 노이즈 억제 | 이동 평균 + 연속 경보 조건으로 오경보 감소 |
| 안정성 | 자동 재연결, 버퍼 관리, 타임아웃 처리 |
| 로그 분리 | 전체 로그 / 경보 로그 CSV 이중 저장 |

---

### ⚠️ 부족한 부분 및 개선 필요 사항

#### 🔴 정확도 한계

**1. 단일 기준국 의존**  
현재는 `GMSD00JPN0` 1개 기준국만 모니터링합니다.  
실제 지진 감지를 위해서는 최소 3~5개 이상의 기준국을 동시에 모니터링해야 합니다.  
단일 기준국의 변위는 지진이 아닌 **수신기 장비 문제, 다중경로 오류** 등으로도 발생할 수 있습니다.

**2. GNSS 측위 오차 미보정**  
현재 코드는 원시(Raw) ECEF 좌표를 그대로 사용합니다.  
실제 정밀 측위에서는 다음 오차를 반드시 보정해야 합니다:
- 대류권·전리층 지연 오차 (수 mm ~ 수십 mm)
- 다중경로(Multipath) 오차
- 위성 궤도·시계 오차 (RTK 보정 필요)

**3. 임계값의 비과학적 설정**  
현재 30mm라는 임계값은 경험적 수치입니다.  
계절적 변동, 기상 조건, 기준국 특성에 따라 정상 변동 범위가 달라지므로  
**통계적 기법(예: 3σ 규칙, 적응형 임계값)** 적용이 필요합니다.

**4. 선행 변위와 지진의 인과관계 미검증**  
GNSS 변위가 지진과 직접적인 인과관계를 가지는지에 대한 **과학적 검증**이 필요합니다.  
이 프로그램은 교육·연구 목적의 **프로토타입**이며, 실제 방재 용도로 사용할 수 없습니다.

#### 🟡 기능 개선 제안

```
[ ] 다중 기준국 동시 모니터링 및 비교 분석
[ ] 수평(E-N) / 수직(U) 성분 분리 계산 (ENU 좌표 변환)
[ ] 지진파(P파·S파) 주파수 분석 추가
[ ] 대시보드 UI (matplotlib 실시간 그래프)
[ ] 이메일·슬랙 경보 알림 연동
[ ] 머신러닝 기반 이상 감지 모델 적용
[ ] 데이터베이스 저장 (SQLite / InfluxDB)
```

---

## 💻 설치 방법 (초심자용)

### Step 1 — Python 설치

> Python 3.8 이상이 필요합니다.

#### 🪟 Windows

1. [https://www.python.org/downloads/](https://www.python.org/downloads/) 접속  
2. **Download Python 3.x.x** 버튼 클릭  
3. 설치 시 반드시 ✅ **"Add Python to PATH"** 체크 후 Install  
4. 설치 확인:

```cmd
python --version
```

#### 🍎 macOS

```bash
# Homebrew가 없다면 먼저 설치
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Python 설치
brew install python

# 설치 확인
python3 --version
```

#### 🐧 Linux (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install python3 python3-pip -y
python3 --version
```

---

### Step 2 — 프로젝트 파일 준비

```bash
# 방법 A: GitHub에서 클론
git clone https://github.com/YOUR_USERNAME/gnss-earthquake-monitor.git
cd gnss-earthquake-monitor

# 방법 B: 파일 직접 다운로드 후 폴더 이동
mkdir gnss-monitor
cd gnss-monitor
# gnss-earthquake.py 파일을 이 폴더에 복사
```

---

### Step 3 — 필요 라이브러리 확인

이 프로그램은 **Python 표준 라이브러리만** 사용하므로 별도 패키지 설치가 필요 없습니다! 🎉

| 라이브러리 | 용도 | 설치 필요 |
|-----------|------|-----------|
| `socket` | TCP 네트워크 통신 | ❌ 기본 내장 |
| `base64` | NTRIP 인증 인코딩 | ❌ 기본 내장 |
| `math` | 유클리드 거리 계산 | ❌ 기본 내장 |
| `csv` | 로그 파일 저장 | ❌ 기본 내장 |
| `collections` | 이동 평균 버퍼 | ❌ 기본 내장 |
| `datetime` | 타임스탬프 기록 | ❌ 기본 내장 |

---

### Step 4 — NTRIP 계정 발급

1. [https://www.igs-ip.net](https://www.igs-ip.net) 접속  
2. 회원가입 (무료)  
3. 발급받은 **사용자명 / 비밀번호** 확인

---

### Step 5 — 코드 설정

`gnss-earthquake.py` 파일을 텍스트 편집기로 열고 아래 부분 수정:

```python
# ====== 이 부분만 수정하세요 ======
USER     = 'your_username'    # ← igs-ip.net 아이디
PASSWORD = 'your_password'    # ← igs-ip.net 비밀번호
# ==================================
```

> 💡 **마운트포인트 변경** (다른 기준국 사용 시):  
> `MOUNTPOINT = 'GMSD00JPN0'` 부분을 원하는 기준국 코드로 변경  
> IGS 기준국 목록: [https://igs-ip.net/home](https://igs-ip.net/home)

---

## ▶️ 실행 방법

### 🪟 Windows

```cmd
cd gnss-monitor
python gnss-earthquake.py
```

### 🍎 macOS / 🐧 Linux

```bash
cd gnss-monitor
python3 gnss-earthquake.py
```

### 🖥️ 백그라운드 실행 (Linux / macOS)

```bash
# 터미널 종료 후에도 계속 실행
nohup python3 gnss-earthquake.py > run.log 2>&1 &
echo "PID: $!"
```

### ⏹️ 프로그램 종료

```
Ctrl + C
```

> 종료 시 수집된 데이터 파일 경로가 출력됩니다.

---

## 📊 출력 결과 해석

### 터미널 출력 예시

```
============================================================
  GNSS 지진 변위 모니터 — GMSD00JPN0
  시작 시각: 2025-04-08 09:00:00
============================================================

[초기화] 기준 샘플 수집 중... 15/30 (남은 샘플: 15)

[✔] GMSD00JPN0 기준 좌표 확정:
    X=-3738755.1234  Y=3444918.5678  Z=3822317.9012

[STATION] GMSD00JPN0 | 간격: 10.23s
[COORD  ] X:-3738755.1302  Y:3444918.5641  Z:3822317.9087
[RESULT ] 원시변위: 8.72mm | 평균변위: 6.34mm | 상태: OK

[STATION] GMSD00JPN0 | 간격: 10.18s
[RESULT ] 원시변위: 34.21mm | 평균변위: 31.05mm | 상태: ⚠️  WARNING

============================================================
  🚨 지진 경보 발령! 연속 3회 임계값 초과
  평균 변위: 45.22 mm  (기준: 30 mm)
============================================================
```

### 📁 생성되는 파일

| 파일명 | 내용 |
|--------|------|
| `gnss_log_GMSD00JPN0_20250408.csv` | 전체 측정 기록 (타임스탬프, 좌표, 변위, 상태) |
| `gnss_alert_GMSD00JPN0_20250408.csv` | 경보 발령 시 기록만 별도 저장 |

### CSV 컬럼 설명

| 컬럼 | 설명 |
|------|------|
| `Timestamp` | 측정 시각 (밀리초 포함) |
| `X / Y / Z` | ECEF 좌표 (단위: m) |
| `Raw_Disp_mm` | 순간 변위량 (mm) |
| `Smooth_Disp_mm` | 이동 평균 변위량 (mm) |
| `Interval_sec` | 직전 메시지로부터 경과 시간 |
| `Status` | `INIT` / `OK` / `WARNING` / `ALERT` |

---

## ⚙️ 파라미터 튜닝 가이드

```python
THRESHOLD_MM        = 30.0   # 경보 임계값 (mm)
                             # ↑ 올리면 오경보 감소, 민감도 저하
                             # ↓ 내리면 민감도 증가, 오경보 증가

INIT_SAMPLES        = 30     # 기준 좌표 확정용 샘플 수
                             # 많을수록 안정적이지만 초기화 시간 증가

SMOOTHING_WINDOW    = 10     # 이동 평균 윈도우 크기
                             # 클수록 노이즈 감소, 반응 속도 저하

ALERT_CONSEC_COUNT  = 3      # 연속 몇 회 초과 시 경보 발령
                             # 크게 하면 오경보 감소, 반응 느려짐

MAX_RECONNECT_TRY   = 5      # 자동 재연결 최대 횟수
```

---

## ⚠️ 주의사항 및 면책 조항

> 🚫 이 프로그램은 **교육 및 연구 목적**의 프로토타입입니다.  
> 실제 방재, 대피 판단, 공식 지진 조기경보 시스템으로 사용할 수 없습니다.  
> 실제 지진 정보는 [기상청](https://www.kma.go.kr) 또는 [일본 기상청(JMA)](https://www.jma.go.jp)을 이용하세요.

---

## 📚 참고 자료

- [RTCM 표준 문서](https://www.rtcm.org/)
- [IGS (International GNSS Service)](https://www.igs.org/)
- [NTRIP 프로토콜 개요](https://www.rtcm.org/differential-global-navigation-satellite-dgnss-standards.html)
- [일본 GEONET GNSS 관측망](https://www.gsi.go.jp/ENGLISH/geonet_english.html)
- [GNSS 기반 지진 선행 변위 연구 (GSI Japan)](https://www.gsi.go.jp/)

---

<div align="center">

Made with ❤️ for earthquake research and education  
⭐ Star this repo if you found it helpful!

</div>
