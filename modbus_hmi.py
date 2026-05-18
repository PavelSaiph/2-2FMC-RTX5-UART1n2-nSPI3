"""
自主中频电源控制板 — Modbus RTU 上位机
对接：STM32H750 从机 (默认 COM10 @ 115200, slave=0x01)

功能：
  • 串口连接/断开
  • 分页显示所有 Modbus 寄存器（按 modbus_regs.h 分区）
  • 每行可独立读/写
  • 32-bit 数值、字符串自动组合/拆分
  • 实时显示 TX/RX HEX 和异常码
"""
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import struct
import time
from datetime import datetime

# ============================================================
# 默认连接参数
# ============================================================
DEFAULT_PORT   = "COM10"
DEFAULT_BAUD   = 115200
DEFAULT_SLAVE  = 1
REQUEST_WAIT_S = 0.12
READ_TIMEOUT_S = 0.25

# ============================================================
# CRC16 Modbus
# ============================================================
def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

EXCEPTION_MSG = {
    0x01: "非法功能码 (Illegal Function)",
    0x02: "非法地址 (只读或越界)",
    0x03: "非法数据 (超出 [min,max])",
    0x04: "从机设备故障",
}

# ============================================================
# 主题配色（Tailwind slate/sky 风格）
# ============================================================
CLR_BG         = "#f1f5f9"   # 窗口背景
CLR_CARD       = "#ffffff"   # 卡片内区
CLR_BORDER     = "#cbd5e1"
CLR_TEXT       = "#0f172a"
CLR_TEXT_MUTE  = "#64748b"
CLR_TEXT_HINT  = "#94a3b8"

CLR_PRIMARY    = "#0284c7"   # 主色（sky-600）
CLR_PRIMARY_HV = "#0369a1"
CLR_ACCENT     = "#f59e0b"   # 写按钮（amber-500）
CLR_ACCENT_HV  = "#d97706"
CLR_SUCCESS    = "#10b981"
CLR_SUCCESS_HV = "#059669"
CLR_DANGER     = "#ef4444"
CLR_DANGER_HV  = "#dc2626"

CLR_HEAD_BG    = "#1e293b"   # 顶栏深色背景
CLR_HEAD_FG    = "#f8fafc"
CLR_HEAD_MUTE  = "#cbd5e1"

CLR_LOG_BG     = "#0f172a"
CLR_LOG_FG     = "#e2e8f0"
CLR_LOG_TX     = "#7dd3fc"
CLR_LOG_RX     = "#86efac"
CLR_LOG_ERR    = "#fca5a5"
CLR_LOG_INFO   = "#d8b4fe"
CLR_LOG_OK     = "#bef264"
CLR_LOG_TIME   = "#64748b"

# ============================================================
# Modbus RTU 主机
# ============================================================
class ModbusMaster:
    def __init__(self, port, baud, slave):
        self.ser = serial.Serial(port, baud, bytesize=8, parity='N',
                                 stopbits=1, timeout=READ_TIMEOUT_S)
        self.slave = slave

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _txrx(self, pdu):
        req = pdu + struct.pack("<H", crc16(pdu))
        self.ser.reset_input_buffer()
        self.ser.write(req)
        self.ser.flush()
        time.sleep(REQUEST_WAIT_S)
        rsp = bytes(self.ser.read(512))
        return req, rsp

    def _check(self, rsp, fc):
        if len(rsp) < 5:
            raise TimeoutError(f"响应过短 ({len(rsp)} 字节)")
        if crc16(rsp[:-2]) != (rsp[-2] | (rsp[-1] << 8)):
            raise ValueError("CRC 校验失败")
        if rsp[0] != self.slave:
            raise ValueError(f"从机地址不匹配 got={rsp[0]}")
        if rsp[1] == (fc | 0x80):
            return None, rsp[2]
        if rsp[1] != fc:
            raise ValueError(f"功能码不匹配 got=0x{rsp[1]:02X}")
        return rsp[2:-2], None

    def read_holding(self, addr, count):
        pdu = struct.pack(">BBHH", self.slave, 0x03, addr, count)
        req, rsp = self._txrx(pdu)
        payload, excep = self._check(rsp, 0x03)
        if excep is not None:
            return None, excep, req, rsp
        regs = [struct.unpack(">H", payload[1+i*2:3+i*2])[0] for i in range(count)]
        return regs, None, req, rsp

    def write_single(self, addr, value):
        pdu = struct.pack(">BBHH", self.slave, 0x06, addr, value & 0xFFFF)
        req, rsp = self._txrx(pdu)
        _, excep = self._check(rsp, 0x06)
        return excep, req, rsp

    def write_multi(self, addr, values):
        pdu = struct.pack(">BBHHB", self.slave, 0x10, addr, len(values), len(values)*2)
        for v in values:
            pdu += struct.pack(">H", v & 0xFFFF)
        req, rsp = self._txrx(pdu)
        _, excep = self._check(rsp, 0x10)
        return excep, req, rsp

# ============================================================
# HMI 主类
# ============================================================
class HMI:
    def __init__(self, root):
        self.root = root
        self.root.title("自主中频电源控制板 — Modbus 上位机")
        self.root.geometry("1280x800")
        self.root.configure(bg=CLR_BG)
        self.root.minsize(1100, 650)
        self.master = None
        self._all_fields = []

        # ----- 字体（默认都放大到 13pt，标题 14pt） -----
        ui_font   = ("Microsoft YaHei UI", 13)
        ui_bold   = ("Microsoft YaHei UI", 13, "bold")
        ui_big    = ("Microsoft YaHei UI", 15, "bold")
        hint_font = ("Microsoft YaHei UI", 11)
        mono_font = ("Consolas", 12)
        title_font= ("Microsoft YaHei UI", 18, "bold")
        self._ui_font    = ui_font
        self._ui_bold    = ui_bold
        self._ui_big     = ui_big
        self._hint_font  = hint_font
        self._mono_font  = mono_font
        self._title_font = title_font

        # ----- ttk 样式 -----
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass

        # 全局背景/字体
        style.configure(".", background=CLR_BG, foreground=CLR_TEXT, font=ui_font)
        style.configure("TFrame", background=CLR_BG)
        style.configure("TLabel", background=CLR_BG, foreground=CLR_TEXT)
        style.configure("Hint.TLabel", background=CLR_BG, foreground=CLR_TEXT_MUTE, font=hint_font)
        style.configure("Card.TFrame", background=CLR_CARD)
        style.configure("Card.TLabel", background=CLR_CARD)
        style.configure("CardHint.TLabel", background=CLR_CARD, foreground=CLR_TEXT_HINT, font=hint_font)

        # 顶栏
        style.configure("Head.TFrame", background=CLR_HEAD_BG)
        style.configure("Head.TLabel", background=CLR_HEAD_BG, foreground=CLR_HEAD_FG, font=ui_font)
        style.configure("HeadMute.TLabel", background=CLR_HEAD_BG, foreground=CLR_HEAD_MUTE, font=ui_font)
        style.configure("Title.TLabel", background=CLR_HEAD_BG, foreground="#ffffff", font=title_font)

        # 输入控件
        style.configure("TEntry", fieldbackground=CLR_CARD, bordercolor=CLR_BORDER,
                        lightcolor=CLR_BORDER, darkcolor=CLR_BORDER, padding=4)
        style.configure("TCombobox", fieldbackground=CLR_CARD, bordercolor=CLR_BORDER,
                        padding=3, arrowsize=14)
        style.map("TCombobox", fieldbackground=[("readonly", CLR_CARD)])

        # Notebook
        style.configure("TNotebook", background=CLR_BG, borderwidth=0, tabmargins=(0,0,0,0))
        style.configure("TNotebook.Tab", padding=(22, 10), font=ui_font,
                        background=CLR_BG, foreground=CLR_TEXT_MUTE, borderwidth=0)
        style.map("TNotebook.Tab",
                  background=[("selected", CLR_PRIMARY), ("active", "#e0f2fe")],
                  foreground=[("selected", "#ffffff"), ("active", CLR_PRIMARY)],
                  expand=[("selected", (0, 0, 0, 0))])

        # LabelFrame
        style.configure("TLabelframe", background=CLR_BG, borderwidth=1, relief="solid",
                        bordercolor=CLR_BORDER)
        style.configure("TLabelframe.Label", background=CLR_BG, foreground=CLR_TEXT, font=ui_bold)

        # 按钮 —— 默认
        style.configure("TButton", font=ui_font, padding=(12, 6), borderwidth=0,
                        background=CLR_CARD, foreground=CLR_TEXT)
        style.map("TButton", background=[("active", "#e2e8f0")])

        # 按钮 —— 读（Primary 蓝）
        style.configure("Read.TButton", font=ui_font, padding=(10, 5), borderwidth=0,
                        background=CLR_PRIMARY, foreground="#ffffff")
        style.map("Read.TButton", background=[("active", CLR_PRIMARY_HV)])

        # 按钮 —— 写（Accent 橙）
        style.configure("Write.TButton", font=ui_font, padding=(10, 5), borderwidth=0,
                        background=CLR_ACCENT, foreground="#ffffff")
        style.map("Write.TButton", background=[("active", CLR_ACCENT_HV)])

        # 按钮 —— 连接（绿）/ 断开（红）
        style.configure("Connect.TButton", font=ui_bold, padding=(20, 8), borderwidth=0,
                        background=CLR_SUCCESS, foreground="#ffffff")
        style.map("Connect.TButton", background=[("active", CLR_SUCCESS_HV)])
        style.configure("Disconnect.TButton", font=ui_bold, padding=(20, 8), borderwidth=0,
                        background=CLR_DANGER, foreground="#ffffff")
        style.map("Disconnect.TButton", background=[("active", CLR_DANGER_HV)])

        # 顶栏上的按钮（深色背景里的 outline 风格）
        style.configure("Head.TButton", font=ui_font, padding=(10, 5), borderwidth=1,
                        background=CLR_HEAD_BG, foreground=CLR_HEAD_FG,
                        bordercolor=CLR_HEAD_MUTE)
        style.map("Head.TButton", background=[("active", "#334155")])
        style.configure("HeadBig.TButton", font=ui_bold, padding=(18, 8), borderwidth=0,
                        background=CLR_PRIMARY, foreground="#ffffff")
        style.map("HeadBig.TButton", background=[("active", CLR_PRIMARY_HV)])

        # 状态徽章（右边 "● 已连接"）
        style.configure("LedOn.TLabel", background=CLR_HEAD_BG, foreground=CLR_SUCCESS,
                        font=("Arial", 20, "bold"))
        style.configure("LedOff.TLabel", background=CLR_HEAD_BG, foreground=CLR_DANGER,
                        font=("Arial", 20, "bold"))

        # 下拉框 Listbox 字体同步
        self.root.option_add("*TCombobox*Listbox.font", ui_font)
        self.root.option_add("*TCombobox*Listbox.background", CLR_CARD)
        self.root.option_add("*TCombobox*Listbox.foreground", CLR_TEXT)
        self.root.option_add("*TCombobox*Listbox.selectBackground", CLR_PRIMARY)

        self.build_ui()
        self.refresh_ports()
        self.set_connected(False)

    # ------------------- UI 构建 -------------------
    def build_ui(self):
        # ========== 顶栏（深色条）==========
        head = tk.Frame(self.root, bg=CLR_HEAD_BG)
        head.pack(fill=tk.X)

        # 左侧：标题
        left = tk.Frame(head, bg=CLR_HEAD_BG)
        left.pack(side=tk.LEFT, padx=22, pady=10)
        ttk.Label(left, text="⚡ 自主中频电源控制板", style="Title.TLabel").pack(anchor="w")
        ttk.Label(left, text="Modbus RTU 上位机 · 通讯卡交互", style="HeadMute.TLabel").pack(anchor="w")

        # 右侧：连接区
        right = tk.Frame(head, bg=CLR_HEAD_BG)
        right.pack(side=tk.RIGHT, padx=22, pady=12)

        row1 = tk.Frame(right, bg=CLR_HEAD_BG); row1.pack(anchor="e")
        ttk.Label(row1, text="串口", style="HeadMute.TLabel").pack(side=tk.LEFT, padx=(0,4))
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.port_cb = ttk.Combobox(row1, textvariable=self.port_var, width=10)
        self.port_cb.pack(side=tk.LEFT, padx=(0,4))
        ttk.Button(row1, text="↻", width=3, command=self.refresh_ports,
                   style="Head.TButton").pack(side=tk.LEFT, padx=(0,10))

        ttk.Label(row1, text="波特率", style="HeadMute.TLabel").pack(side=tk.LEFT, padx=(4,4))
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Combobox(row1, textvariable=self.baud_var,
                     values=["9600","19200","38400","57600","115200"],
                     width=7).pack(side=tk.LEFT, padx=(0,10))

        ttk.Label(row1, text="从机", style="HeadMute.TLabel").pack(side=tk.LEFT, padx=(4,4))
        self.slave_var = tk.StringVar(value=str(DEFAULT_SLAVE))
        ttk.Entry(row1, textvariable=self.slave_var, width=4).pack(side=tk.LEFT, padx=(0,12))

        self.conn_btn = ttk.Button(row1, text="连接", command=self.toggle_conn,
                                    style="Connect.TButton")
        self.conn_btn.pack(side=tk.LEFT)

        row2 = tk.Frame(right, bg=CLR_HEAD_BG); row2.pack(anchor="e", pady=(6,0))
        self.led = ttk.Label(row2, text="●", style="LedOff.TLabel")
        self.led.pack(side=tk.LEFT)
        self.led_txt = ttk.Label(row2, text="未连接", style="HeadMute.TLabel")
        self.led_txt.pack(side=tk.LEFT, padx=(5,14))
        ttk.Button(row2, text="全部读取", command=self.read_all,
                   style="HeadBig.TButton").pack(side=tk.LEFT)

        # ========== 中部：标签页 ==========
        wrap = ttk.Frame(self.root, padding=(12, 10, 12, 4))
        wrap.pack(fill=tk.X)
        self.nb = ttk.Notebook(wrap)
        self.nb.pack(fill=tk.X)

        self._tab_fields = {}

        self._build_tab_sysbasic()
        self._build_tab_power_control()
        self._build_tab_user_limit()
        self._build_tab_comm_card()
        self._build_tab_pulse()
        self._build_tab_pwr_pulse()
        self._build_tab_ramp()

        # ========== 日志区（扩展） ==========
        lf_wrap = ttk.Frame(self.root, padding=(12, 0, 12, 12))
        lf_wrap.pack(fill=tk.BOTH, expand=True)

        lf_header = ttk.Frame(lf_wrap)
        lf_header.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(lf_header, text="📋 通讯日志", font=self._ui_big).pack(side=tk.LEFT)
        ttk.Button(lf_header, text="清空日志",
                   command=lambda: self.log_txt.delete("1.0", tk.END)).pack(side=tk.RIGHT)

        # 日志文本框（加边框）
        log_frame = tk.Frame(lf_wrap, bg=CLR_BORDER, bd=1)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_txt = scrolledtext.ScrolledText(log_frame, height=10, font=self._mono_font,
                                                 bg=CLR_LOG_BG, fg=CLR_LOG_FG,
                                                 insertbackground=CLR_LOG_FG,
                                                 relief="flat", bd=0, padx=10, pady=8)
        self.log_txt.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        self.log_txt.tag_config("tx",   foreground=CLR_LOG_TX,   font=self._mono_font)
        self.log_txt.tag_config("rx",   foreground=CLR_LOG_RX,   font=self._mono_font)
        self.log_txt.tag_config("err",  foreground=CLR_LOG_ERR,  font=self._mono_font)
        self.log_txt.tag_config("info", foreground=CLR_LOG_INFO, font=self._mono_font)
        self.log_txt.tag_config("ok",   foreground=CLR_LOG_OK,   font=self._mono_font)
        self.log_txt.tag_config("time", foreground=CLR_LOG_TIME, font=self._mono_font)

    # ------- 通用行生成器 -------
    def _grid_header(self, f):
        hdr = ttk.Frame(f)
        hdr.grid(row=0, column=0, columnspan=5, sticky="ew", pady=(2, 10))
        hdr.grid_columnconfigure(0, weight=1)
        ttk.Label(hdr, text="字段",  font=self._ui_bold).grid(row=0, column=0, sticky="w", padx=(6,0))
        ttk.Label(hdr, text="地址",  font=self._ui_bold, width=8, anchor="center").grid(row=0, column=1, padx=4)
        ttk.Label(hdr, text="当前值", font=self._ui_bold, width=30, anchor="w").grid(row=0, column=2, padx=4, sticky="w")
        ttk.Label(hdr, text="操作",  font=self._ui_bold).grid(row=0, column=3, columnspan=2, padx=4)
        # 分隔线
        sep = tk.Frame(f, bg=CLR_BORDER, height=1)
        sep.grid(row=1, column=0, columnspan=5, sticky="ew", pady=(0, 4))

    def _row(self, parent, row, label, addr, bits=16, readonly=False, combo=None, unit=""):
        """普通数值字段行"""
        # row 从 2 起始（0=表头 Frame, 1=分隔线）
        r = row + 1
        ttk.Label(parent, text=label, anchor="w").grid(row=r, column=0, sticky="w", padx=(10, 8), pady=5)
        ttk.Label(parent, text=f"0x{addr:04X}", style="Hint.TLabel",
                  width=8, anchor="center").grid(row=r, column=1, padx=4)
        var = tk.StringVar()
        val_frm = ttk.Frame(parent); val_frm.grid(row=r, column=2, padx=6, sticky="w")
        if combo:
            vals = [f"{k}  {v}" for k, v in combo.items()]
            ttk.Combobox(val_frm, textvariable=var, values=vals, width=24,
                         state="readonly" if readonly else "normal").pack(side=tk.LEFT)
        else:
            ttk.Entry(val_frm, textvariable=var, width=22,
                      state="readonly" if readonly else "normal").pack(side=tk.LEFT, ipady=2)
            if unit: ttk.Label(val_frm, text=unit, style="Hint.TLabel").pack(side=tk.LEFT, padx=6)
        regs = 2 if bits == 32 else 1
        kind = "int32" if bits == 32 else "int16"
        ttk.Button(parent, text="读", width=5, style="Read.TButton",
                   command=lambda: self._read_field(addr, regs, kind, var, combo)).grid(row=r, column=3, padx=3, pady=3)
        if not readonly:
            ttk.Button(parent, text="写", width=5, style="Write.TButton",
                       command=lambda: self._write_field(addr, regs, kind, var, combo)).grid(row=r, column=4, padx=3, pady=3)
        else:
            ttk.Label(parent, text="🔒 只读", style="Hint.TLabel").grid(row=r, column=4, padx=6)
        return (addr, regs, var, kind, label, combo)

    def _row_string(self, parent, row, label, addr, byte_count):
        r = row + 1
        regs = byte_count // 2
        ttk.Label(parent, text=label, anchor="w").grid(row=r, column=0, sticky="w", padx=(10, 8), pady=5)
        ttk.Label(parent, text=f"0x{addr:04X}", style="Hint.TLabel",
                  width=8, anchor="center").grid(row=r, column=1, padx=4)
        var = tk.StringVar()
        val_frm = ttk.Frame(parent); val_frm.grid(row=r, column=2, padx=6, sticky="w")
        ttk.Entry(val_frm, textvariable=var, width=30, state="readonly").pack(side=tk.LEFT, ipady=2)
        ttk.Button(parent, text="读", width=5, style="Read.TButton",
                   command=lambda: self._read_field(addr, regs, "string", var, None)).grid(row=r, column=3, padx=3, pady=3)
        ttk.Label(parent, text="🔒 只读", style="Hint.TLabel").grid(row=r, column=4, padx=6)
        return (addr, regs, var, "string", label, None)

    def _finalize_tab(self, f, fields, tab_name):
        max_row = 0
        for w in f.winfo_children():
            info = w.grid_info()
            if info:
                try: max_row = max(max_row, int(info.get('row', 0)))
                except Exception: pass
        r = max_row + 1
        # 底部分割 + 按钮
        sep = tk.Frame(f, bg=CLR_BORDER, height=1)
        sep.grid(row=r, column=0, columnspan=5, sticky="ew", pady=(10, 8))
        btns = ttk.Frame(f); btns.grid(row=r+1, column=0, columnspan=5, sticky="e", padx=10, pady=(0,4))
        ttk.Button(btns, text="📥 读取本页全部", style="Read.TButton",
                   command=lambda: self._read_many(fields)).pack(side=tk.RIGHT)
        self._tab_fields[tab_name] = fields
        for c in range(5): f.grid_columnconfigure(c, weight=0)
        f.grid_columnconfigure(0, weight=1)

    # ------- 各 Tab -------
    def _build_tab_sysbasic(self):
        f = ttk.Frame(self.nb, padding=(10, 12, 10, 10)); self.nb.add(f, text="   设备信息   ")
        self._grid_header(f)
        fields = []
        specs_str = [
            ("Unit Type 电源型号",     0x0000, 20),
            ("Unit Number 序列号",     0x000A, 10),
            ("FPGA1 版本",             0x000F, 10),
            ("FPGA2 版本",             0x0014, 10),
            ("ARM1 版本",              0x0019, 10),
            ("ARM2 版本",              0x001E, 10),
        ]
        r = 1
        for lbl, a, bc in specs_str:
            fields.append(self._row_string(f, r, lbl, a, bc)); r += 1
        for lbl, a, bits, u in [
            ("Unit Power Limit 机器功率",   0x0023, 32, "W"),
            ("Unit Voltage Limit 机器电压", 0x0025, 16, "V"),
            ("Unit Current Limit 机器电流", 0x0026, 16, "0.1A"),
        ]:
            fields.append(self._row(f, r, lbl, a, bits=bits, readonly=True, unit=u)); r += 1
        self._finalize_tab(f, fields, "设备信息")

    def _build_tab_power_control(self):
        f = ttk.Frame(self.nb, padding=(10, 12, 10, 10)); self.nb.add(f, text="   电源控制   ")
        self._grid_header(f)
        fields = []
        r = 1
        fields.append(self._row(f, r, "output 开关机", 0x0100,
                                combo={0:"关 OFF", 1:"开 ON"})); r += 1
        fields.append(self._row(f, r, "regulation mode 调节模式", 0x0101,
                                combo={6:"功率模式", 7:"电压模式", 8:"电流模式"})); r += 1
        fields.append(self._row(f, r, "setpoint 设定值 (32-bit)", 0x0102, bits=32,
                                unit="W/V/0.1A")); r += 1
        fields.append(self._row(f, r, "control mode 控制模式", 0x0104,
                                combo={2:"主机控制(通讯)", 4:"用户端口(模拟量)"})); r += 1
        self._finalize_tab(f, fields, "电源控制")

    def _build_tab_user_limit(self):
        f = ttk.Frame(self.nb, padding=(10, 12, 10, 10)); self.nb.add(f, text="   用户限值   ")
        self._grid_header(f)
        fields = []
        r = 1
        for lbl, a, bits, u in [
            ("user power limit 用户功率限值",   0x0140, 32, "W"),
            ("user voltage limit 用户电压限值", 0x0142, 16, "V"),
            ("user current limit 用户电流限值", 0x0143, 16, "0.1A"),
        ]:
            fields.append(self._row(f, r, lbl, a, bits=bits, unit=u)); r += 1
        fields.append(self._row(f, r, "ignition switch 点火开关", 0x0144,
                                combo={0:"关", 1:"开"})); r += 1
        fields.append(self._row(f, r, "ignition setpoint 点火设定值", 0x0145,
                                combo={0:"低", 1:"中", 2:"高"})); r += 1
        fields.append(self._row(f, r, "Ripple Level 纹波水平", 0x0146,
                                combo={0:"高纹波", 1:"低纹波"})); r += 1
        self._finalize_tab(f, fields, "用户限值")

    def _build_tab_comm_card(self):
        f = ttk.Frame(self.nb, padding=(10, 12, 10, 10)); self.nb.add(f, text="   通讯卡   ")
        self._grid_header(f)
        fields = []
        fields.append(self._row(f, 1, "comm watchdog 通讯看门狗", 0x0120, unit="(0=禁用)"))
        self._finalize_tab(f, fields, "通讯卡")

    def _build_tab_pulse(self):
        f = ttk.Frame(self.nb, padding=(10, 12, 10, 10)); self.nb.add(f, text="   脉冲控制   ")
        self._grid_header(f)
        fields = []
        r = 1
        fields.append(self._row(f, r, "pulsing 启用/禁用方波", 0x0160,
                                combo={0:"DC 直流", 1:"pulsing 方波"})); r += 1
        fields.append(self._row(f, r, "DC polarity 直流极性", 0x0161,
                                combo={0:"正极性", 1:"负极性"})); r += 1
        fields.append(self._row(f, r, "boost voltage setpoint", 0x0162)); r += 1
        fields.append(self._row(f, r, "output deadtime 1", 0x0163)); r += 1
        fields.append(self._row(f, r, "output deadtime 2", 0x0164)); r += 1
        fields.append(self._row(f, r, "output frequency 输出频率", 0x0165)); r += 1
        fields.append(self._row(f, r, "pulse duty cycle 占空比", 0x0166, unit="%")); r += 1
        self._finalize_tab(f, fields, "脉冲控制")

    def _build_tab_pwr_pulse(self):
        f = ttk.Frame(self.nb, padding=(10, 12, 10, 10)); self.nb.add(f, text="   功率脉冲   ")
        self._grid_header(f)
        fields = []
        r = 1
        fields.append(self._row(f, r, "power pulsing 启用/禁用", 0x0180,
                                combo={0:"禁用", 1:"启用"})); r += 1
        fields.append(self._row(f, r, "on time 打开时长",  0x0181)); r += 1
        fields.append(self._row(f, r, "off time 关闭时长", 0x0182)); r += 1
        self._finalize_tab(f, fields, "功率脉冲")

    def _build_tab_ramp(self):
        f = ttk.Frame(self.nb, padding=(10, 12, 10, 10)); self.nb.add(f, text="   缓启动   ")
        self._grid_header(f)
        fields = []
        r = 1
        fields.append(self._row(f, r, "Ramp 启用/禁用", 0x0190,
                                combo={0:"禁用", 1:"启用"})); r += 1
        fields.append(self._row(f, r, "step 步长", 0x0191, unit="% 设定值")); r += 1
        fields.append(self._row(f, r, "total ramp time 总时长", 0x0192, unit="ms")); r += 1
        self._finalize_tab(f, fields, "缓启动")

    # ------------------- 日志 -------------------
    def log(self, msg, tag="info"):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_txt.insert(tk.END, f"[{ts}]  ", "time")
        self.log_txt.insert(tk.END, msg + "\n", tag)
        self.log_txt.see(tk.END)

    def _log_txrx(self, label, req, rsp, excep):
        self.log(f"── {label}")
        self.log(f"  TX: {req.hex(' ')}", "tx")
        if rsp:
            self.log(f"  RX: {rsp.hex(' ')}", "rx")
        else:
            self.log("  RX: (无响应)", "err"); return
        if excep is not None:
            self.log(f"  ⚠ 异常 0x{excep:02X} — {EXCEPTION_MSG.get(excep,'未知')}", "err")
        else:
            self.log("  ✓ 成功", "ok")

    # ------------------- 连接 -------------------
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if self.port_var.get() not in ports and ports:
            self.port_var.set(ports[0])

    def set_connected(self, ok):
        if ok:
            self.led.config(style="LedOn.TLabel")
            self.led_txt.config(text=f"已连接  {self.port_var.get()}")
            self.conn_btn.config(text="断开", style="Disconnect.TButton")
        else:
            self.led.config(style="LedOff.TLabel")
            self.led_txt.config(text="未连接")
            self.conn_btn.config(text="连接", style="Connect.TButton")

    def toggle_conn(self):
        if self.master:
            self.master.close()
            self.master = None
            self.set_connected(False)
            self.log("已断开", "info")
        else:
            try:
                port = self.port_var.get()
                baud = int(self.baud_var.get())
                slave = int(self.slave_var.get())
                self.master = ModbusMaster(port, baud, slave)
                self.set_connected(True)
                self.log(f"已连接 {port} @ {baud} slave={slave}", "ok")
            except Exception as e:
                messagebox.showerror("连接失败", str(e))
                self.log(f"连接失败: {e}", "err")

    # ------------------- 字段读/写 -------------------
    def _check_conn(self):
        if not self.master:
            messagebox.showwarning("未连接", "请先点击 [连接] 按钮")
            return False
        return True

    def _read_field(self, addr, regs, kind, var, combo):
        if not self._check_conn(): return
        try:
            regs_val, excep, req, rsp = self.master.read_holding(addr, regs)
            self._log_txrx(f"读 0x{addr:04X} × {regs}", req, rsp, excep)
            if excep is not None: return
            if kind == "string":
                s = b''.join(struct.pack(">H", r) for r in regs_val)
                txt = s.rstrip(b"\x00").decode("ascii", errors="replace")
                var.set(txt)
            elif kind == "int32":
                val = (regs_val[0] << 16) | regs_val[1]
                var.set(str(val))
            else:   # int16
                val = regs_val[0]
                if combo and val in combo:
                    var.set(f"{val} - {combo[val]}")
                else:
                    var.set(str(val))
        except Exception as e:
            self.log(f"读 0x{addr:04X} 失败: {e}", "err")

    def _parse_input(self, text, kind, combo):
        """从输入框文本解析成整数/字符串"""
        text = text.strip()
        if combo:
            # "6 - 功率模式" → 6
            try:
                return int(text.split("-")[0].strip()), None
            except:
                return None, "下拉值格式错误"
        if kind == "string":
            return text.encode("ascii", errors="replace"), None
        try:
            if text.lower().startswith("0x"):
                return int(text, 16), None
            return int(text), None
        except ValueError:
            return None, "不是合法数字"

    def _write_field(self, addr, regs, kind, var, combo):
        if not self._check_conn(): return
        val, err = self._parse_input(var.get(), kind, combo)
        if err:
            messagebox.showerror("输入错误", err); return
        try:
            if kind == "int32":
                if val < 0 or val > 0xFFFFFFFF:
                    messagebox.showerror("输入错误", "32-bit 值超范围"); return
                hi, lo = (val >> 16) & 0xFFFF, val & 0xFFFF
                excep, req, rsp = self.master.write_multi(addr, [hi, lo])
                self._log_txrx(f"写 0x{addr:04X} = {val} (32-bit)", req, rsp, excep)
            else:
                if val < 0 or val > 0xFFFF:
                    messagebox.showerror("输入错误", "16-bit 值超范围"); return
                excep, req, rsp = self.master.write_single(addr, val)
                self._log_txrx(f"写 0x{addr:04X} = {val}", req, rsp, excep)
            if excep is None:
                # 写成功后自动回读
                self._read_field(addr, regs, kind, var, combo)
        except Exception as e:
            self.log(f"写 0x{addr:04X} 失败: {e}", "err")

    def _read_many(self, fields):
        if not self._check_conn(): return
        for (addr, regs, var, kind, label, combo) in fields:
            self._read_field(addr, regs, kind, var, combo)

    def read_all(self):
        if not self._check_conn(): return
        self.log("── 读取所有 tab ──", "info")
        for tab_name, fields in self._tab_fields.items():
            self.log(f"── {tab_name} ──", "info")
            self._read_many(fields)

# ============================================================
def main():
    root = tk.Tk()
    try:
        root.tk.call("tk", "scaling", 1.0)
    except Exception:
        pass
    app = HMI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
