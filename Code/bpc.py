"""
BPC 电波钟信号发生器 for ESP32-S3 (MicroPython)
=================================================
硬件配置:
  IO7  → BPC 载波输出 (68.5kHz PWM, 接谐振线圈天线)
  GP5  → OLED SDA  (I2C, SSD1306 128×32)
  GP6  → OLED SCL
  GP13 → LED 状态灯 (高电平点亮)

状态显示:
  LED 常亮  → WiFi + NTP 均正常，正在发射
  LED 闪烁  → WiFi 或 NTP 失败 (500ms 间隔)

OLED 布局 (128×32, 4行×约21字符):
  启动阶段  逐步显示连接状态
  运行阶段  Line1: HH:MM:SS  Wday
            Line2: YYYY-MM-DD
            Line3: BPC TX  frm#NNNN
            Line4: [===...===] 进度条 (20秒一帧)

BPC 编码规范 (专利原文):
  帧周期 20 秒，每分钟三帧 (0/20/40 秒)
  四进制: 低电平 100/200/300/400ms → 0/1/2/3
  P0 = 帧间隔 (整秒空缺)
  P1 = 帧序号 (sec01, weights 2,1)
  P2 = 预留位 (sec02, 固定 0)
  P3 = am/pm 复用 + sec01~09 奇偶校验 (sec10)
  P4 = year[6] 复用 + sec11~18 奇偶校验 (sec19)
"""

import machine
import network
import ntptime
import utime

# ─────────────────────────────────────────────
#  用户配置区
# ─────────────────────────────────────────────
WIFI_SSID     = "你的WiFi名称"
WIFI_PASSWORD = "你的WiFi密码"
BPC_PIN       = 7
CARRIER_FREQ  = 68500
PWM_DUTY      = 512        # 0-1023, 50%
NTP_HOST      = "pool.ntp.org"
CST_OFFSET    = 8 * 3600
OLED_SDA      = 5
OLED_SCL      = 6
LED_PIN       = 13
# ─────────────────────────────────────────────

# ── 全局硬件对象 ──
pwm_carrier = None
led         = machine.Pin(LED_PIN, machine.Pin.OUT)
oled        = None          # 初始化后赋值
_oled_ok    = False

# ── OLED 驱动 (内联 SSD1306, 无需额外库) ──────────────────────────────────────
# 最小化 SSD1306 128×32 I2C 驱动 + 5×7 字体渲染

_FONT5X7 = {
    ' ': 0x00000000000000,
    '!': 0x00000000005f00, '"': 0x00000003000300, '#': 0x00001414fe1414,
    '$': 0x00002449fe4900, '%': 0x00002313086462, '&': 0x00003649292600,
    "'": 0x00000000030000, '(': 0x0000001c22410000, ')': 0x00004122001c00,
    '*': 0x00001408fe0814, '+': 0x00001010fe1010, ',': 0x00005030000000,
    '-': 0x00001010101010, '.': 0x00006060000000, '/': 0x00002010080402,
    '0': 0x003e514941513e, '1': 0x00000042ff4000, '2': 0x00426151494600,
    '3': 0x00212941493600, '4': 0x00181412ff1000, '5': 0x00274545453900,
    '6': 0x003c4a49493000, '7': 0x000103714905,   '8': 0x003649494936,
    '9': 0x000649494f3e,   ':': 0x00000036360000, ';': 0x00005636000000,
    '<': 0x00000814224100, '=': 0x00002828282800, '>': 0x00412214080000,
    '?': 0x00020151090600, '@': 0x003e415d594e00, 'A': 0x007e1111117e00,
    'B': 0x007f4949493600, 'C': 0x003e4141412200, 'D': 0x007f4141223c00,
    'E': 0x007f4949412200, 'F': 0x007f090909010,  'G': 0x003e414151720,
    'H': 0x007f0808087f00, 'I': 0x00004141ff4141, 'J': 0x00203e21212100,
    'K': 0x007f08142241,   'L': 0x007f4040404000, 'M': 0x007f020c027f00,
    'N': 0x007f040804007f, 'O': 0x003e4141413e00, 'P': 0x007f0909090600,
    'Q': 0x003e41512e5c00, 'R': 0x007f09192946,   'S': 0x002649494932,
    'T': 0x000101ff010100, 'U': 0x003f4040403f00, 'V': 0x001f2040201f00,
    'W': 0x003f402030403f, 'X': 0x006314081463,   'Y': 0x000304f8030400,
    'Z': 0x006151494543,   '[': 0x00007f41410000, '\\':0x00020408102000,
    ']': 0x00004141ff0000, '^': 0x00000203020000, '_': 0x004040404040,
    'a': 0x002028282810,   'b': 0x007f2424241800, 'c': 0x001824242400,
    'd': 0x001824247f0000, 'e': 0x001828282400,   'f': 0x007e090101020,
    'g': 0x000c525252,     'h': 0x007f0808087000, 'i': 0x00407d40000000,
    'j': 0x004040404000,   'k': 0x007f10284400,   'l': 0x00003f40400000,
    'm': 0x007c04087c0000, 'n': 0x007c04047800,   'o': 0x003844443800,
    'p': 0x007c14141800,   'q': 0x001814147c0000, 'r': 0x007c040408,
    's': 0x002448482400,   't': 0x00043e04040000, 'u': 0x003c4040207c,
    'v': 0x001c2040201c,   'w': 0x003c4030403c,   'x': 0x004428102844,
    'y': 0x000c525050,     'z': 0x004464544c4400, '~': 0x000008040804,
}

# 用完整准确的 5×8 点阵替换
_FONT = [
    0x00,0x00,0x00,0x00,0x00,  # ' '
    0x00,0x00,0x5F,0x00,0x00,  # '!'
    0x00,0x07,0x00,0x07,0x00,  # '"'
    0x14,0x7F,0x14,0x7F,0x14,  # '#'
    0x24,0x2A,0x7F,0x2A,0x12,  # '$'
    0x23,0x13,0x08,0x64,0x62,  # '%'
    0x36,0x49,0x55,0x22,0x50,  # '&'
    0x00,0x05,0x03,0x00,0x00,  # '''
    0x00,0x1C,0x22,0x41,0x00,  # '('
    0x00,0x41,0x22,0x1C,0x00,  # ')'
    0x08,0x2A,0x1C,0x2A,0x08,  # '*'
    0x08,0x08,0x3E,0x08,0x08,  # '+'
    0x00,0x50,0x30,0x00,0x00,  # ','
    0x08,0x08,0x08,0x08,0x08,  # '-'
    0x00,0x30,0x30,0x00,0x00,  # '.'
    0x20,0x10,0x08,0x04,0x02,  # '/'
    0x3E,0x51,0x49,0x45,0x3E,  # '0'
    0x00,0x42,0x7F,0x40,0x00,  # '1'
    0x42,0x61,0x51,0x49,0x46,  # '2'
    0x21,0x41,0x45,0x4B,0x31,  # '3'
    0x18,0x14,0x12,0x7F,0x10,  # '4'
    0x27,0x45,0x45,0x45,0x39,  # '5'
    0x3C,0x4A,0x49,0x49,0x30,  # '6'
    0x01,0x71,0x09,0x05,0x03,  # '7'
    0x36,0x49,0x49,0x49,0x36,  # '8'
    0x06,0x49,0x49,0x29,0x1E,  # '9'
    0x00,0x36,0x36,0x00,0x00,  # ':'
    0x00,0x56,0x36,0x00,0x00,  # ';'
    0x00,0x08,0x14,0x22,0x41,  # '<'
    0x14,0x14,0x14,0x14,0x14,  # '='
    0x41,0x22,0x14,0x08,0x00,  # '>'
    0x02,0x01,0x51,0x09,0x06,  # '?'
    0x32,0x49,0x79,0x41,0x3E,  # '@'
    0x7E,0x11,0x11,0x11,0x7E,  # 'A'
    0x7F,0x49,0x49,0x49,0x36,  # 'B'
    0x3E,0x41,0x41,0x41,0x22,  # 'C'
    0x7F,0x41,0x41,0x22,0x1C,  # 'D'
    0x7F,0x49,0x49,0x49,0x41,  # 'E'
    0x7F,0x09,0x09,0x01,0x01,  # 'F'
    0x3E,0x41,0x41,0x51,0x32,  # 'G'
    0x7F,0x08,0x08,0x08,0x7F,  # 'H'
    0x00,0x41,0x7F,0x41,0x00,  # 'I'
    0x20,0x40,0x41,0x3F,0x01,  # 'J'
    0x7F,0x08,0x14,0x22,0x41,  # 'K'
    0x7F,0x40,0x40,0x40,0x40,  # 'L'
    0x7F,0x02,0x04,0x02,0x7F,  # 'M'
    0x7F,0x04,0x08,0x10,0x7F,  # 'N'
    0x3E,0x41,0x41,0x41,0x3E,  # 'O'
    0x7F,0x09,0x09,0x09,0x06,  # 'P'
    0x3E,0x41,0x51,0x21,0x5E,  # 'Q'
    0x7F,0x09,0x19,0x29,0x46,  # 'R'
    0x46,0x49,0x49,0x49,0x31,  # 'S'
    0x01,0x01,0x7F,0x01,0x01,  # 'T'
    0x3F,0x40,0x40,0x40,0x3F,  # 'U'
    0x1F,0x20,0x40,0x20,0x1F,  # 'V'
    0x3F,0x40,0x38,0x40,0x3F,  # 'W'
    0x63,0x14,0x08,0x14,0x63,  # 'X'
    0x03,0x04,0x78,0x04,0x03,  # 'Y'
    0x61,0x51,0x49,0x45,0x43,  # 'Z'
    0x00,0x00,0x7F,0x41,0x41,  # '['
    0x02,0x04,0x08,0x10,0x20,  # '\'
    0x41,0x41,0x7F,0x00,0x00,  # ']'
    0x04,0x02,0x01,0x02,0x04,  # '^'
    0x40,0x40,0x40,0x40,0x40,  # '_'
    0x00,0x01,0x02,0x04,0x00,  # '`'
    0x20,0x54,0x54,0x54,0x78,  # 'a'
    0x7F,0x48,0x44,0x44,0x38,  # 'b'
    0x38,0x44,0x44,0x44,0x20,  # 'c'
    0x38,0x44,0x44,0x48,0x7F,  # 'd'
    0x38,0x54,0x54,0x54,0x18,  # 'e'
    0x08,0x7E,0x09,0x01,0x02,  # 'f'
    0x08,0x14,0x54,0x54,0x3C,  # 'g'
    0x7F,0x08,0x04,0x04,0x78,  # 'h'
    0x00,0x44,0x7D,0x40,0x00,  # 'i'
    0x20,0x40,0x44,0x3D,0x00,  # 'j'
    0x00,0x7F,0x10,0x28,0x44,  # 'k'
    0x00,0x41,0x7F,0x40,0x00,  # 'l'
    0x7C,0x04,0x18,0x04,0x78,  # 'm'
    0x7C,0x08,0x04,0x04,0x78,  # 'n'
    0x38,0x44,0x44,0x44,0x38,  # 'o'
    0x7C,0x14,0x14,0x14,0x08,  # 'p'
    0x08,0x14,0x14,0x18,0x7C,  # 'q'
    0x7C,0x08,0x04,0x04,0x08,  # 'r'
    0x48,0x54,0x54,0x54,0x20,  # 's'
    0x04,0x3F,0x44,0x40,0x20,  # 't'
    0x3C,0x40,0x40,0x20,0x7C,  # 'u'
    0x1C,0x20,0x40,0x20,0x1C,  # 'v'
    0x3C,0x40,0x30,0x40,0x3C,  # 'w'
    0x44,0x28,0x10,0x28,0x44,  # 'x'
    0x0C,0x50,0x50,0x50,0x3C,  # 'y'
    0x44,0x64,0x54,0x4C,0x44,  # 'z'
    0x00,0x08,0x36,0x41,0x00,  # '{'
    0x00,0x00,0x7F,0x00,0x00,  # '|'
    0x00,0x41,0x36,0x08,0x00,  # '}'
    0x08,0x08,0x2A,0x1C,0x08,  # '→' (用作右箭头占位)
    0x08,0x1C,0x2A,0x08,0x08,  # '←'
]

class SSD1306_I2C:
    """最小 SSD1306 128×32 驱动，不依赖 framebuf"""
    W = 128
    H = 32
    ADDR = 0x3C

    def __init__(self, i2c):
        self.i2c = i2c
        self.buf = bytearray(self.W * self.H // 8)
        self._init_display()

    def _cmd(self, c):
        self.i2c.writeto(self.ADDR, bytes([0x00, c]))

    def _init_display(self):
        for c in (
            0xAE,0x20,0x00,0xB0,0xC8,0x00,0x10,0x40,
            0x81,0xCF,0xA1,0xA6,0xA8,0x1F,
            0xD3,0x00,0xD5,0xF0,0xD9,0x22,0xDA,0x02,
            0xDB,0x20,0x8D,0x14,0xAF
        ):
            self._cmd(c)
        self.fill(0)
        self.show()

    def fill(self, v):
        b = 0xFF if v else 0x00
        for i in range(len(self.buf)):
            self.buf[i] = b

    def pixel(self, x, y, v):
        if 0 <= x < self.W and 0 <= y < self.H:
            page = y >> 3
            bit  = y & 7
            idx  = page * self.W + x
            if v:
                self.buf[idx] |=  (1 << bit)
            else:
                self.buf[idx] &= ~(1 << bit)

    def text(self, s, x, y, c=1):
        """Draw ASCII string at pixel (x,y), each char 6px wide (5+1 gap)"""
        cx = x
        for ch in s:
            code = ord(ch)
            if 32 <= code <= 125:
                idx = (code - 32) * 5
                for col in range(5):
                    byte = _FONT[idx + col]
                    for row in range(8):
                        self.pixel(cx + col, y + row, (byte >> row) & 1 if c else 0)
            cx += 6

    def hline(self, x, y, w, c=1):
        for i in range(w):
            self.pixel(x + i, y, c)

    def rect(self, x, y, w, h, c=1):
        self.hline(x, y, w, c)
        self.hline(x, y+h-1, w, c)
        for i in range(h):
            self.pixel(x, y+i, c)
            self.pixel(x+w-1, y+i, c)

    def show(self):
        self._cmd(0x21); self._cmd(0); self._cmd(self.W-1)
        self._cmd(0x22); self._cmd(0); self._cmd(self.H//8-1)
        chunk = 16
        hdr = bytes([0x40])
        for off in range(0, len(self.buf), chunk):
            self.i2c.writeto(self.ADDR, hdr + self.buf[off:off+chunk])


# ── OLED 辅助 ─────────────────────────────────────────────────────────────────

def oled_init():
    global oled, _oled_ok
    try:
        i2c = machine.I2C(1, sda=machine.Pin(OLED_SDA), scl=machine.Pin(OLED_SCL), freq=400000)
        oled = SSD1306_I2C(i2c)
        _oled_ok = True
        print("OLED OK")
    except Exception as e:
        print("OLED init failed:", e)
        _oled_ok = False


def oled_clear():
    if _oled_ok:
        oled.fill(0)


def oled_show():
    if _oled_ok:
        oled.show()


def oled_line(row, text, clear_first=False):
    """
    row 0-3, 每行 8px 高。
    row0: y=0  row1: y=8  row2: y=16  row3: y=24
    """
    if not _oled_ok:
        return
    y = row * 8
    # 清该行
    for yy in range(y, y+8):
        oled.hline(0, yy, 128, 0)
    oled.text(text[:21], 0, y)


def oled_status(line0="", line1="", line2="", line3=""):
    if not _oled_ok:
        return
    oled.fill(0)
    oled.text(line0[:21], 0,  0)
    oled.text(line1[:21], 0,  8)
    oled.text(line2[:21], 0, 16)
    oled.text(line3[:21], 0, 24)
    oled.show()


def oled_draw_tx_icon(active):
    """
    在屏幕右侧 (x=100..127, y=16..31) 画发射图案
    active=True: 实心矩形 + 小三角 (发射中)
    active=False: 空心矩形 (待机)
    """
    if not _oled_ok:
        return
    # 清图标区
    for yy in range(16, 32):
        for xx in range(98, 128):
            oled.pixel(xx, yy, 0)
    if active:
        # 天线塔 (小三角)
        oled.pixel(113, 16, 1)
        oled.hline(112, 17, 3, 1)
        oled.hline(111, 18, 5, 1)
        # 发射波纹 (弧线)
        for y_off, x_span in [(19,1),(20,3),(21,5)]:
            oled.pixel(108,  y_off, 1)
            oled.pixel(118,  y_off, 1)
        # TX 文字
        oled.text("TX", 104, 24)
    else:
        oled.text("--", 104, 24)


def oled_update_runtime(cst, frame_count, sec_in_frame):
    """运行时刷新 OLED 显示"""
    if not _oled_ok:
        return
    year, month, day, hour, minute, second, weekday, _ = cst
    days = ["Mon","Tue","Wed","Thu","Fri","Sat","Sun"]
    wday = days[weekday]

    oled.fill(0)

    # Line 0: 时间 + 星期
    oled.text(f"{hour:02d}:{minute:02d}:{second:02d} {wday}", 0, 0)

    # Line 1: 日期
    oled.text(f"{year}-{month:02d}-{day:02d}", 0, 8)

    # Line 2: 帧信息 (留右侧给图标)
    oled.text(f"BPC frm#{frame_count:04d}", 0, 16)

    # Line 3: 进度条 (20秒，98px 宽)
    BAR_X = 0
    BAR_Y = 25
    BAR_W = 98
    BAR_H = 6
    oled.rect(BAR_X, BAR_Y, BAR_W, BAR_H)
    fill_w = int(sec_in_frame / 20.0 * (BAR_W - 2))
    if fill_w > 0:
        for yy in range(BAR_Y+1, BAR_Y+BAR_H-1):
            oled.hline(BAR_X+1, yy, fill_w)

    # 右侧发射图标
    oled_draw_tx_icon(True)

    oled.show()


# ── LED 控制 ──────────────────────────────────────────────────────────────────

def led_on():
    led.value(1)

def led_off():
    led.value(0)

_blink_timer  = None
_blink_state  = False

def led_start_blink():
    """500ms 间隔闪烁，用软件定时器实现，不阻塞主线程"""
    global _blink_timer, _blink_state
    _blink_state = False
    if _blink_timer is None:
        _blink_timer = machine.Timer(0)
    _blink_timer.init(period=500, mode=machine.Timer.PERIODIC,
                      callback=_blink_cb)

def _blink_cb(t):
    global _blink_state
    _blink_state = not _blink_state
    led.value(1 if _blink_state else 0)

def led_stop_blink():
    global _blink_timer
    if _blink_timer:
        _blink_timer.deinit()
        _blink_timer = None
    led_off()


# ── WiFi / NTP ────────────────────────────────────────────────────────────────

def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        return True
    print(f"WiFi connecting: {WIFI_SSID}")
    oled_status("BPC Transmitter", f"WiFi: {WIFI_SSID[:14]}", "Connecting...", "")
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    for i in range(20):
        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            print("WiFi OK:", ip)
            oled_status("BPC Transmitter", f"WiFi: {WIFI_SSID[:14]}", "Connected!", ip)
            utime.sleep_ms(800)
            return True
        oled_line(2, f"Connecting {'.'*(i%4+1)}")
        oled_show()
        utime.sleep_ms(1000)
    print("WiFi FAILED")
    oled_status("BPC Transmitter", f"WiFi: {WIFI_SSID[:14]}", "FAILED!", "Check settings")
    return False


def sync_ntp():
    try:
        ntptime.host = NTP_HOST
        ntptime.settime()
        print("NTP OK")
        return True
    except Exception as e:
        print("NTP failed:", e)
        return False


# ── 时间工具 ──────────────────────────────────────────────────────────────────

def get_cst():
    t = utime.time() + CST_OFFSET
    return utime.localtime(t)


# ── BPC 载波控制 ──────────────────────────────────────────────────────────────

def carrier_on():
    global pwm_carrier
    if pwm_carrier is None:
        pwm_carrier = machine.PWM(machine.Pin(BPC_PIN), freq=CARRIER_FREQ, duty=PWM_DUTY)
    else:
        pwm_carrier.duty(PWM_DUTY)

def carrier_off():
    global pwm_carrier
    if pwm_carrier is not None:
        pwm_carrier.duty(0)


# ── BPC 编码 ──────────────────────────────────────────────────────────────────

def encode_frame(cst_tuple):
    year, month, day, hour, minute, second, weekday, _ = cst_tuple
    dow   = weekday + 1
    am_pm = 0 if hour < 12 else 1
    hour12 = hour % 12
    year2  = year % 100

    def bits_to_ms(msb, lsb):
        return (msb * 2 + lsb + 1) * 100

    sec40_bit = 1 if second >= 40 else 0
    sec20_bit = 1 if (second % 40) >= 20 else 0

    def collect_bits(s_list, sec_range):
        bits = []
        for i in sec_range:
            pv = (s_list[i] // 100) - 1
            bits.append((pv >> 1) & 1)
            bits.append((pv >> 0) & 1)
        return bits

    def odd_parity(bits_list):
        return sum(bits_list) % 2

    s = [None] * 20
    s[0]  = 0
    s[1]  = bits_to_ms(sec40_bit, sec20_bit)
    s[2]  = bits_to_ms(0, 0)
    s[3]  = bits_to_ms((hour12 >> 3) & 1, (hour12 >> 2) & 1)
    s[4]  = bits_to_ms((hour12 >> 1) & 1, (hour12 >> 0) & 1)
    s[5]  = bits_to_ms((minute >> 5) & 1, (minute >> 4) & 1)
    s[6]  = bits_to_ms((minute >> 3) & 1, (minute >> 2) & 1)
    s[7]  = bits_to_ms((minute >> 1) & 1, (minute >> 0) & 1)
    s[8]  = bits_to_ms(0, (dow >> 2) & 1)
    s[9]  = bits_to_ms((dow >> 1) & 1, (dow >> 0) & 1)
    p3_parity = odd_parity(collect_bits(s, range(1, 10)))
    s[10] = bits_to_ms(am_pm, p3_parity)
    s[11] = bits_to_ms(0, (day >> 4) & 1)
    s[12] = bits_to_ms((day >> 3) & 1, (day >> 2) & 1)
    s[13] = bits_to_ms((day >> 1) & 1, (day >> 0) & 1)
    s[14] = bits_to_ms((month >> 3) & 1, (month >> 2) & 1)
    s[15] = bits_to_ms((month >> 1) & 1, (month >> 0) & 1)
    s[16] = bits_to_ms((year2 >> 5) & 1, (year2 >> 4) & 1)
    s[17] = bits_to_ms((year2 >> 3) & 1, (year2 >> 2) & 1)
    s[18] = bits_to_ms((year2 >> 1) & 1, (year2 >> 0) & 1)
    year64 = (year2 >> 6) & 1
    p4_parity = odd_parity(collect_bits(s, range(11, 19)))
    s[19] = bits_to_ms(year64, p4_parity)
    return s


# ── BPC 发射 ─────────────────────────────────────────────────────────────────

def _wait_until(t_start, ms_target):
    while True:
        remaining = ms_target - utime.ticks_diff(utime.ticks_ms(), t_start)
        if remaining <= 0:
            break
        if remaining > 2:
            utime.sleep_ms(remaining - 2)


def transmit_frame(durations, frame_count, cst_at_start):
    """
    发射一帧并同步刷新 OLED。
    OLED 更新在每秒高电平阶段的中间进行，不影响时序关键点。
    """
    for sec_idx, low_ms in enumerate(durations):
        t_sec_start = utime.ticks_ms()

        if low_ms == 0:
            carrier_on()
        else:
            carrier_off()
            _wait_until(t_sec_start, low_ms)
            carrier_on()

        # OLED 更新放在高电平阶段（约 t+low_ms+200ms 处），远离时序边界
        _wait_until(t_sec_start, low_ms + 200 if low_ms else 500)
        oled_update_runtime(get_cst(), frame_count, sec_idx)

        # 等待本秒结束
        _wait_until(t_sec_start, 1000)


def wait_for_next_bpc_boundary():
    print("Waiting for BPC boundary (sec=0/20/40)...")
    oled_status("BPC Transmitter", "Waiting for", "frame boundary...", "")
    while get_cst()[5] in (0, 20, 40):
        utime.sleep_ms(50)
    while True:
        cst = get_cst()
        if cst[5] in (0, 20, 40):
            print(f"Aligned: {cst[3]:02d}:{cst[4]:02d}:{cst[5]:02d}")
            return cst
        utime.sleep_ms(10)


# ── 主程序 ───────────────────────────────────────────────────────────────────

def main():
    print("=" * 40)
    print("BPC Transmitter — ESP32-S3")
    print("=" * 40)

    # 初始化外设
    oled_init()
    led_off()
    carrier_on(); carrier_off()   # 初始化 PWM 对象

    oled_status("BPC Transmitter", "Initializing...", "", "")
    utime.sleep_ms(500)

    # ── WiFi ──
    wifi_ok = wifi_connect()
    if not wifi_ok:
        led_start_blink()
        oled_status("BPC Transmitter", "WiFi FAILED", "Check SSID/PWD", "Retrying...")
        # 持续重试，不死等
        while not wifi_ok:
            utime.sleep_ms(10000)
            wifi_ok = wifi_connect()
        led_stop_blink()

    # ── NTP ──
    oled_status("BPC Transmitter", "WiFi OK", "Syncing NTP...", "")
    ntp_ok = sync_ntp()
    if not ntp_ok:
        led_start_blink()
        oled_status("BPC Transmitter", "WiFi OK", "NTP FAILED", "Retrying...")
        retries = 0
        while not ntp_ok:
            utime.sleep_ms(5000)
            ntp_ok = sync_ntp()
            retries += 1
            oled_line(3, f"Retry #{retries}")
            oled_show()
        led_stop_blink()

    # WiFi + NTP 均成功 → LED 常亮
    led_on()
    cst = get_cst()
    oled_status("BPC Transmitter",
                "NTP OK",
                f"{cst[3]:02d}:{cst[4]:02d}:{cst[5]:02d} CST",
                "Starting TX...")
    utime.sleep_ms(1000)

    # ── 对齐帧边界 ──
    cst = wait_for_next_bpc_boundary()

    print("Transmitting continuously...\n")
    frame_count = 0

    while True:
        y, mo, d, h, mi, s, wd, _ = cst
        print(f"Frame #{frame_count+1:04d}  {y}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d}")

        durations = encode_frame(cst)
        transmit_frame(durations, frame_count + 1, cst)
        frame_count += 1

        cst = get_cst()

        # 每 30 帧做一次 NTP 校时（帧间隙，约 0ms 停顿）
        if frame_count % 30 == 0:
            print("[NTP resync]")
            oled_line(0, "NTP resyncing...")
            oled_show()
            if not sync_ntp():
                # NTP 失败时 LED 闪，但继续发射
                led_start_blink()
                oled_line(0, "NTP failed!")
                oled_show()
                utime.sleep_ms(3000)
                led_stop_blink()
                led_on()
            cst = get_cst()


if __name__ == "__main__":
    main()