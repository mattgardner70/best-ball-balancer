"""Wiring diagram and wire list, generated from one connection list.

    uv run python wiring/wiring.py        -> wiring/wiring.html

Edit CONNECTIONS (and the panel layouts) here; never hand-edit the HTML.
Pin map matches code/firmware/balance/balance.ino.
"""

from html import escape
from pathlib import Path

OUT = Path(__file__).resolve().parent / "wiring.html"

# Wire classes -> (CSS colour variable, what to use)
WIRES = {
    "12v": ("--w12", "red, 20 AWG"),
    "5v": ("--w5", "red/orange, 20 AWG"),
    "gnd": ("--wgnd", "black, 20 AWG"),
    "link": ("--wlink", "short jumper, same colour as its bus"),
    "mot+": ("--wmotp", "motor pin 1, M+ (yellow)"),
    "mot-": ("--wmotn", "motor pin 6, M− (white)"),
    "encA": ("--wencA", "motor pin 3, A (blue) to Dupont"),
    "encB": ("--wencB", "motor pin 4, B (purple) to Dupont"),
    "pwm": ("--wpwm", "Dupont"),
    "dir": ("--wdir", "Dupont"),
    "stby": ("--wstby", "Dupont"),
    "touch": ("--wtouch", "Dupont"),
    "btn": ("--wbtn", "button harness to Dupont"),
    "ctl": ("--wctl", "Dupont"),
    "plug": ("--wgnd", "barrel plug"),
}

# (group, from device, from pin, to device, to pin, wire class)
CONNECTIONS = [
    # --- power in ---------------------------------------------------------------------------
    ("power", "ADAPTER", "plug", "JACK", "socket", "plug"),
    ("power", "JACK", "+", "SW", "VIN", "12v"),
    ("power", "JACK", "−", "G1", "1", "gnd"),
    ("power", "SW", "GND", "G1", "2", "gnd"),
    ("power", "SW", "VOUT", "P12a", "1", "12v"),
    # --- +12 V bus (2x 221-415) -------------------------------------------------------------
    ("power", "P12a", "2", "MD1", "VB+", "12v"),
    ("power", "P12a", "3", "MD2", "VB+", "12v"),
    ("power", "P12a", "4", "MEGA", "VIN", "12v"),
    ("power", "P12a", "5", "P12b", "1", "link"),
    ("power", "P12b", "2", "BUCK", "IN+", "12v"),
    ("power", "P12b", "3", "BTNP", "LED+", "12v"),
    ("power", "P12b", "4", "BTNM", "LED+", "12v"),
    # --- +5 V bus (2x 221-415), fed by the buck --------------------------------------------
    ("power", "BUCK", "OUT+", "P5a", "1", "5v"),
    ("power", "P5a", "2", "M1", "5 5V", "5v"),
    ("power", "P5a", "3", "M2", "5 5V", "5v"),
    ("power", "P5a", "4", "M3", "5 5V", "5v"),
    # --- GND bus (4x 221-415) ---------------------------------------------------------------
    ("power", "MD1", "VB−", "G1", "3", "gnd"),
    ("power", "MD2", "VB−", "G1", "4", "gnd"),
    ("power", "G1", "5", "G2", "1", "link"),
    ("power", "MEGA", "GND", "G2", "2", "gnd"),
    ("power", "BUCK", "IN−", "G2", "3", "gnd"),
    ("power", "BUCK", "OUT−", "G2", "4", "gnd"),
    ("power", "G2", "5", "G3", "1", "link"),
    ("power", "M1", "2 GND", "G3", "2", "gnd"),
    ("power", "M2", "2 GND", "G3", "3", "gnd"),
    ("power", "M3", "2 GND", "G3", "4", "gnd"),
    ("power", "G3", "5", "G4", "1", "link"),
    ("power", "BTNP", "LED−", "G4", "2", "gnd"),
    ("power", "BTNP", "C", "G4", "3", "gnd"),
    ("power", "BTNM", "LED−", "G4", "4", "gnd"),
    ("power", "BTNM", "C", "G4", "5", "gnd"),
    # --- motors: Mega -> drivers ------------------------------------------------------------
    # Cytron MDD3A: each motor takes two PWM inputs (A high = forward, B high = backward, both low = brake).
    # D6/D7/D8 are Timer4, D44/D45/D46 Timer5; both set to 20 kHz (the MDD3A's PWM limit).
    ("motors", "MEGA", "D6", "MD1", "M1A in", "pwm"),
    ("motors", "MEGA", "D7", "MD1", "M1B in", "pwm"),
    ("motors", "MEGA", "D8", "MD1", "M2A in", "pwm"),
    ("motors", "MEGA", "D44", "MD1", "M2B in", "pwm"),
    ("motors", "MEGA", "D45", "MD2", "M1A in", "pwm"),
    ("motors", "MEGA", "D46", "MD2", "M1B in", "pwm"),
    # --- motors: drivers -> motors ----------------------------------------------------------
    ("motors", "MD1", "M1A", "M1", "1 M+", "mot+"),
    ("motors", "MD1", "M1B", "M1", "6 M−", "mot-"),
    ("motors", "MD1", "M2A", "M2", "1 M+", "mot+"),
    ("motors", "MD1", "M2B", "M2", "6 M−", "mot-"),
    ("motors", "MD2", "M1A", "M3", "1 M+", "mot+"),
    ("motors", "MD2", "M1B", "M3", "6 M−", "mot-"),
    # --- motors: encoders -> Mega -----------------------------------------------------------
    ("motors", "M1", "3 A", "MEGA", "D2", "encA"),
    ("motors", "M1", "4 B", "MEGA", "D19", "encB"),
    ("motors", "M2", "3 A", "MEGA", "D3", "encA"),
    ("motors", "M2", "4 B", "MEGA", "D20", "encB"),
    ("motors", "M3", "3 A", "MEGA", "D18", "encA"),
    ("motors", "M3", "4 B", "MEGA", "D21", "encB"),
    # --- touch panel ------------------------------------------------------------------------
    ("inputs", "FPC", "1 Y−", "MEGA", "A3", "touch"),
    ("inputs", "FPC", "2 X−", "MEGA", "A2", "touch"),
    ("inputs", "FPC", "3 Y+", "MEGA", "A1", "touch"),
    ("inputs", "FPC", "4 X+", "MEGA", "A0", "touch"),
    # --- buttons and soft power ---------------------------------------------------------------
    ("inputs", "BTNP", "NO", "SW", "A", "btn"),
    ("inputs", "BTNP", "NC", "MEGA", "D31", "btn"),
    ("inputs", "BTNM", "NO", "MEGA", "D30", "btn"),
    ("inputs", "MEGA", "D32", "SW", "OFF", "ctl"),
]

NAMES = {
    "ADAPTER": "12 V 5 A wall adapter", "JACK": "Panel jack (frame back tab)", "SW": "Pololu #2813 soft power switch",
    "P12a": "Wago +12 V (A)", "P12b": "Wago +12 V (B)", "P5a": "Wago +5 V",
    "G1": "Wago GND (1)", "G2": "Wago GND (2)", "G3": "Wago GND (3)", "G4": "Wago GND (4)",
    "BUCK": "DROK 12→5 V buck", "MEGA": "Arduino Mega 2560", "MD1": "Cytron MDD3A #1 (motors 1, 2)",
    "MD2": "Cytron MDD3A #2 (motor 3)", "M1": "Motor 1 (leg 0°)", "M2": "Motor 2 (leg 120°)", "M3": "Motor 3 (leg 240°)",
    "BTNP": "POWER button", "BTNM": "MODE button", "FPC": "Touch-panel tail connector",
}
WAGOS = {"P12a", "P12b", "P5a", "G1", "G2", "G3", "G4"}

# ---------------------------------------------------------------------------------------------
# Panel layouts: device -> (x, y, w, side of pins, [pins in order]). Wagos are drawn as 5-port blocks.

PW = 54  # pin pitch along a card edge
PANELS = {
    "power": dict(
        title="Power: what plugs into every Wago port",
        size=(1240, 900),
        devices={
            "ADAPTER": (20, 30, 150, "bottom", ["plug"]),
            "JACK": (20, 150, 150, "bottom", ["socket", "+", "−"]),
            "SW": (230, 150, 200, "bottom", ["VIN", "GND", "VOUT"]),
            "P12a": (330, 300, None, "wago", None), "P12b": (640, 300, None, "wago", None),
            "MD1": (20, 470, 200, "top", ["VB+", "VB−"]),
            "MD2": (250, 470, 200, "top", ["VB+", "VB−"]),
            "MEGA": (480, 470, 150, "top", ["VIN", "GND"]),
            "BUCK": (660, 470, 260, "top", ["IN+", "IN−", "OUT+", "OUT−"]),
            "BTNP": (950, 470, 200, "top", ["LED+", "LED−", "C"]),
            "BTNM": (950, 610, 200, "top", ["LED+", "LED−", "C"]),
            "P5a": (330, 640, None, "wago", None),
            "M1": (20, 640, 150, "bottom", ["5 5V", "2 GND"]),
            "M2": (20, 740, 150, "bottom", ["5 5V", "2 GND"]),
            "M3": (190, 740, 150, "bottom", ["5 5V", "2 GND"]),
            "G1": (330, 800, None, "wago", None), "G2": (560, 800, None, "wago", None),
            "G3": (790, 800, None, "wago", None), "G4": (1020, 800, None, "wago", None),
        }),
    "motors": dict(
        title="Motors: control signals, motor power and encoders",
        size=(1240, 880), channel_y=800,
        devices={
            "MEGA": (30, 40, 150, "right", ["D6", "D7", "D8", "D44", "D45", "D46",
                                           "D2", "D19", "D3", "D20", "D18", "D21"]),
            "MD1": (520, 40, 170, "both", (["M1A in", "M1B in", "M2A in", "M2B in"], ["M1A", "M1B", "M2A", "M2B"])),
            "MD2": (520, 420, 170, "both", (["M1A in", "M1B in"], ["M1A", "M1B"])),
            "M1": (980, 40, 170, "left", ["1 M+", "6 M−", "3 A", "4 B"]),
            "M2": (980, 300, 170, "left", ["1 M+", "6 M−", "3 A", "4 B"]),
            "M3": (980, 560, 170, "left", ["1 M+", "6 M−", "3 A", "4 B"]),
        }),
    "inputs": dict(
        title="Touch panel, buttons and soft power",
        size=(1240, 560),
        devices={
            "FPC": (30, 40, 170, "right", ["1 Y−", "2 X−", "3 Y+", "4 X+"]),
            "BTNP": (30, 300, 170, "right", ["NO", "NC"]),
            "BTNM": (30, 440, 170, "right", ["NO"]),
            "MEGA": (560, 40, 150, "left", ["A3", "A2", "A1", "A0", "D31", "D30", "D32"]),
            "SW": (980, 300, 200, "left", ["A", "OFF"]),
        }),
}

ROW = 30  # vertical pitch for side pins


def _wago_ports(x, y):
    """5 port centres along the block, plus top/bottom anchors."""
    return {str(i + 1): (x + 22 + i * 30, y) for i in range(5)}


def layout(panel):
    """Pin anchor points: (device, pin) -> (x, y, side)."""
    anchors, shapes = {}, []
    for dev, (x, y, w, side, pins) in panel["devices"].items():
        if side == "wago":
            ports = _wago_ports(x, y)
            for p, (px, py) in ports.items():
                anchors[(dev, p)] = (px, py, "wago", y)
            shapes.append(("wago", dev, x, y))
            continue
        if side in ("top", "bottom"):
            h = 70
            for i, p in enumerate(pins):
                px = x + 30 + i * PW
                anchors[(dev, p)] = (px, y if side == "top" else y + h, side, None)
            shapes.append(("card", dev, x, y, w, h, side, pins))
        else:
            left, right = (pins if side == "both" else ([], []))
            if side == "left":
                left, right = pins, []
            if side == "right":
                left, right = [], pins
            n = max(len(left), len(right))
            h = 48 + n * ROW
            for i, p in enumerate(left):
                anchors[(dev, p)] = (x, y + 44 + i * ROW, "left", None)
            for i, p in enumerate(right):
                anchors[(dev, p)] = (x + w, y + 44 + i * ROW, "right", None)
            shapes.append(("side", dev, x, y, w, h, left, right))
    return anchors, shapes


def route(a, b, lane):
    (x1, y1, s1, wy1), (x2, y2, s2, wy2) = a, b
    # Wago ports attach on whichever face points at the other end.
    if s1 == "wago":
        y1 = wy1 + (0 if y2 < wy1 else 34)
    if s2 == "wago":
        y2 = wy2 + (0 if y1 < wy2 else 34)
    if s1 in ("left", "right") or s2 in ("left", "right"):
        xm = (x1 + x2) / 2 + lane
        return f"M{x1},{y1} H{xm} V{y2} H{x2}", (xm, (y1 + y2) / 2)
    ym = (y1 + y2) / 2 + lane
    return f"M{x1},{y1} V{ym} H{x2} V{y2}", ((x1 + x2) / 2, ym)


BUS_ORDER = [("+12 V", ["P12a", "P12b"]), ("+5 V", ["P5a"]), ("GND", ["G1", "G2", "G3", "G4"])]


def svg_power():
    """Power as a flow strip plus a port map for every Wago (what plugs into each numbered port)."""
    W, H = 1240, 650
    o = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="Power distribution through the Wago buses">']
    # Flow strip
    boxes = [("12 V 5 A adapter", "barrel plug", 20), ("Panel jack", "frame back tab", 230),
             ("Pololu #2813", "VIN → VOUT", 440), ("+12 V Wagos", "A + B", 650),
             ("DROK buck", "12 V → 5 V", 860), ("+5 V Wagos", "A + B", 1070)]
    for name, sub, x in boxes:
        o.append(f'<g class="flow card"><rect x="{x}" y="20" width="160" height="56" rx="6"/>'
                 f'<text x="{x + 80}" y="44">{escape(name)}</text><text class="sub" x="{x + 80}" y="62">{escape(sub)}</text></g>')
    for (x1, x2, var) in ((180, 230, "--w12"), (390, 440, "--w12"), (600, 650, "--w12"), (810, 860, "--w12"), (1020, 1070, "--w5")):
        o.append(f'<path class="arrow" style="stroke:var({var})" d="M{x1},48 H{x2 - 6}"/>'
                 f'<path style="fill:var({var})" d="M{x2 - 8},42 L{x2},48 L{x2 - 8},54 Z"/>')
    o.append('<text class="flow sub" x="620" y="100" style="text-anchor:middle">Every ground goes to the GND Wagos below. '
             'Linked Wagos act as one bus: the dashed port joins them.</text>')

    # Port maps
    def port_rows(dev):
        rows = {str(i): None for i in range(1, 6)}
        for _, d1, p1, d2, p2, w in CONNECTIONS:
            if d1 == dev and p1 in rows:
                rows[p1] = (d2, p2, w)
            if d2 == dev and p2 in rows:
                rows[p2] = (d1, p1, w)
        return rows

    cols = [("P12a", 20, 140), ("P12b", 325, 140), ("P5a", 630, 140),
            ("G1", 20, 420), ("G2", 325, 420), ("G3", 630, 420), ("G4", 935, 420)]
    heads = {"P12a": "+12 V", "P5a": "+5 V", "G1": "Ground"}
    for dev, x, y in cols:
        if dev in heads:
            o.append(f'<text class="dname" x="{x}" y="{y - 8}" style="font-size:15px">{heads[dev]} bus</text>')
        o.append(f'<g class="wago"><rect x="{x}" y="{y + 10}" width="44" height="164" rx="4"/>')
        for i in range(5):
            o.append(f'<rect class="lever" x="{x + 6}" y="{y + 18 + i * 31}" width="32" height="25" rx="3"/>'
                     f'<text class="port" x="{x + 22}" y="{y + 35 + i * 31}">{i + 1}</text>')
        o.append(f'<text class="wlabel" x="{x + 22}" y="{y + 196}">{escape(NAMES[dev].replace("Wago ", ""))}</text></g>')
        for i, (port, conn) in enumerate(port_rows(dev).items()):
            py = y + 30 + i * 31
            if conn is None:
                o.append(f'<text class="pinL" x="{x + 60}" y="{py + 4}" style="fill:var(--muted)">spare</text>')
                continue
            d2, p2, w = conn
            var = WIRES[w][0]
            dash = ' stroke-dasharray="6 4"' if w == "link" else ""
            label = (f"link to {NAMES[d2].replace('Wago ', '')} port {p2}" if d2 in WAGOS else f"{NAMES[d2]}")
            o.append(f'<path class="wire" style="stroke:var({var})"{dash} d="M{x + 44},{py} H{x + 62}"/>'
                     f'<g class="tag"><rect x="{x + 62}" y="{py - 12}" width="225" height="24" rx="4"/>'
                     f'<text x="{x + 70}" y="{py + 4}">{escape(label)}</text>'
                     + ("" if d2 in WAGOS else f'<text class="tpin" x="{x + 279}" y="{py + 4}" style="text-anchor:end">{escape(p2)}</text>')
                     + "</g>")
    o.append("</svg>")
    return "\n".join(o)


def svg_panel(key):
    if key == "power":
        return svg_power()
    panel = PANELS[key]
    anchors, shapes = layout(panel)
    W, H = panel["size"]
    out = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{escape(panel["title"])}">']
    # wires first, under the devices
    wires = [c for c in CONNECTIONS if (c[1], c[2]) in anchors and (c[3], c[4]) in anchors]
    enc_k = 0
    for i, (_, d1, p1, d2, p2, wcls) in enumerate(wires):
        lane = ((i % 9) - 4) * 7
        if wcls in ("encA", "encB") and "channel_y" in panel:
            # Encoders: out of the motor, down a channel under the drivers, back along to the Mega.
            (x1, y1, *_), (x2, y2, *_) = anchors[(d1, p1)], anchors[(d2, p2)]
            xa, xc = x1 - 24 - enc_k * 7, x2 + 30 + enc_k * 9
            yb = panel["channel_y"] + enc_k * 9
            path = f"M{x1},{y1} H{xa} V{yb} H{xc} V{y2} H{x2}"
            enc_k += 1
        else:
            path, _mid = route(anchors[(d1, p1)], anchors[(d2, p2)], lane)
        var = WIRES[wcls][0]
        dash = ' stroke-dasharray="6 4"' if wcls == "link" else ""
        out.append(f'<path d="{path}" class="wire" style="stroke:var({var})"{dash}><title>'
                   f'{escape(NAMES[d1])} {escape(p1)} → {escape(NAMES[d2])} {escape(p2)}</title></path>')
    for s in shapes:
        if s[0] == "wago":
            _, dev, x, y = s
            out.append(f'<g class="wago"><rect x="{x}" y="{y}" width="164" height="34" rx="4"/>')
            for i in range(5):
                out.append(f'<rect class="lever" x="{x + 10 + i * 30}" y="{y + 6}" width="24" height="22" rx="3"/>'
                           f'<text class="port" x="{x + 22 + i * 30}" y="{y + 21}">{i + 1}</text>')
            out.append(f'<text class="wlabel" x="{x + 82}" y="{y + (52 if y < 790 else -8)}">{escape(NAMES[dev])}</text></g>')
        elif s[0] == "card":
            _, dev, x, y, w, h, side, pins = s
            out.append(f'<g class="card"><rect x="{x}" y="{y}" width="{w}" height="{h}" rx="6"/>'
                       f'<text class="dname" x="{x + 10}" y="{y + (h - 12 if side == "top" else 20)}">{escape(NAMES[dev])}</text>')
            for i, p in enumerate(pins):
                px = x + 30 + i * PW
                py = y + 20 if side == "top" else y + h - 26
                out.append(f'<text class="pin" x="{px}" y="{py}">{escape(p)}</text>'
                           f'<circle class="dot" cx="{px}" cy="{y if side == "top" else y + h}" r="4"/>')
            out.append("</g>")
        else:
            _, dev, x, y, w, h, left, right = s
            out.append(f'<g class="card"><rect x="{x}" y="{y}" width="{w}" height="{h}" rx="6"/>'
                       f'<text class="dname" x="{x + 10}" y="{y + 22}">{escape(NAMES[dev])}</text>')
            for i, p in enumerate(left):
                yy = y + 44 + i * ROW
                out.append(f'<circle class="dot" cx="{x}" cy="{yy}" r="4"/><text class="pinL" x="{x + 10}" y="{yy + 4}">{escape(p)}</text>')
            for i, p in enumerate(right):
                yy = y + 44 + i * ROW
                out.append(f'<circle class="dot" cx="{x + w}" cy="{yy}" r="4"/><text class="pinR" x="{x + w - 10}" y="{yy + 4}">{escape(p)}</text>')
            out.append("</g>")
    out.append("</svg>")
    return "\n".join(out)


def tables():
    by_dev = {}
    for grp, d1, p1, d2, p2, w in CONNECTIONS:
        by_dev.setdefault(d1, []).append((p1, d2, p2, w))
        by_dev.setdefault(d2, []).append((p2, d1, p1, w))
    order = ["SW", "JACK", "BUCK", "MEGA", "MD1", "MD2", "M1", "M2", "M3", "FPC", "BTNP", "BTNM",
             "P12a", "P12b", "P5a", "G1", "G2", "G3", "G4"]
    html = []
    for dev in order:
        rows = sorted(by_dev.get(dev, []), key=lambda r: (len(r[0]) > 3, r[0]))
        html.append(f'<section class="tbl"><h3>{escape(NAMES[dev])}</h3><div class="scroll"><table>'
                    "<thead><tr><th>Pin</th><th>Goes to</th><th>Wire</th></tr></thead><tbody>")
        for p, d2, p2, w in rows:
            var, desc = WIRES[w]
            html.append(f'<tr><td class="mono">{escape(p)}</td><td>{escape(NAMES[d2])} · <span class="mono">{escape(p2)}</span></td>'
                        f'<td><span class="sw" style="background:var({var})"></span>{escape(desc)}</td></tr>')
        html.append("</tbody></table></div></section>")
    return "\n".join(html)


CSS = """
:root {
  /* Layout: a drafting sheet. Three schematic panels, then one wire table per part. */
  --bg: #f7f7f4; --paper: #ffffff; --ink: #1f2326; --muted: #5d666d; --rule: #cfd5d8; --card: #f0f3f4;
  --w12: #d6332c; --w5: #e8890c; --wgnd: #24282b; --wlink: #8a939a; --wmotp: #e0a800; --wmotn: #9aa3a9;
  --wencA: #3bc9db; --wencB: #be4bdb; --wpwm: #1971c2; --wdir: #7048e8; --wstby: #868e96; --wtouch: #0c8f73;
  --wbtn: #c2255c; --wctl: #e8590c;
  --sans: "IBM Plex Sans", system-ui, sans-serif; --mono: "IBM Plex Mono", ui-monospace, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #141719; --paper: #1b1f22; --ink: #e6e9eb; --muted: #9aa4ab; --rule: #3a4248; --card: #22282c;
  --wgnd: #d5dadd; --wmotn: #c3cacf; --wlink: #9aa3a9; color-scheme: dark } }
:root[data-theme="dark"] {
  --bg: #141719; --paper: #1b1f22; --ink: #e6e9eb; --muted: #9aa4ab; --rule: #3a4248; --card: #22282c;
  --wgnd: #d5dadd; --wmotn: #c3cacf; --wlink: #9aa3a9; color-scheme: dark }
body { background: var(--bg); color: var(--ink); font-family: var(--sans); }
main { max-width: 1280px; margin: 0 auto; padding-inline: 16px; padding-block: 28px 56px; display: grid; gap: 28px; }
h1 { margin: 0; font-size: 1.6rem; text-wrap: balance; }
h2 { margin: 0 0 6px; font-size: 1.1rem; }
h3 { margin: 0 0 8px; font-size: .95rem; }
p { margin: 0; line-height: 1.55; max-width: 75ch; color: var(--muted); }
.panel { background: var(--paper); border: 1px solid var(--rule); border-radius: 8px; padding: 14px; min-width: 0; }
.scroll { overflow-x: auto; }
svg { width: 100%; min-width: 900px; height: auto; display: block; }
.wire { fill: none; stroke-width: 2.6; stroke-linejoin: round; }
.wire:hover { stroke-width: 5; }
.card rect { fill: var(--card); stroke: var(--rule); stroke-width: 1.2; }
.dname { font: 600 13px var(--sans); fill: var(--ink); }
.pin { font: 12px var(--mono); fill: var(--ink); text-anchor: middle; }
.pinL { font: 12px var(--mono); fill: var(--ink); text-anchor: start; }
.pinR { font: 12px var(--mono); fill: var(--ink); text-anchor: end; }
.tag rect { fill: var(--card); stroke: var(--rule); }
.tag text { font: 12px var(--sans); fill: var(--ink); }
.tag .tpin { font: 600 12px var(--mono); }
.flow text { font: 600 13px var(--sans); fill: var(--ink); text-anchor: middle; }
.flow .sub { font: 11px var(--mono); fill: var(--muted); }
.arrow { fill: none; stroke-width: 3; }
.dot { fill: var(--paper); stroke: var(--ink); stroke-width: 1.5; }
.wago rect { fill: #e9ecef; stroke: #868e96; stroke-width: 1.2; }
.wago .lever { fill: #f08c00; stroke: #c96f00; }
.port { font: 600 11px var(--mono); fill: #2b2b2b; text-anchor: middle; }
.wlabel { font: 600 12px var(--sans); fill: var(--ink); text-anchor: middle; }
.legend { display: flex; flex-wrap: wrap; gap: 8px 18px; font-size: .85rem; color: var(--muted); }
.sw { display: inline-block; width: 22px; height: 4px; border-radius: 2px; vertical-align: middle; margin-right: 6px; }
.tables { display: grid; grid-template-columns: repeat(auto-fill, minmax(330px, 1fr)); gap: 16px; }
.tbl { background: var(--paper); border: 1px solid var(--rule); border-radius: 8px; padding: 12px; min-width: 0; }
table { border-collapse: collapse; width: 100%; font-size: .85rem; }
th, td { text-align: left; padding: 5px 6px; border-bottom: 1px solid var(--rule); vertical-align: top; }
th { font: 500 .72rem var(--mono); text-transform: uppercase; letter-spacing: .06em; color: var(--muted); }
.mono { font-family: var(--mono); }
.notes { display: grid; gap: 8px; }
.notes li { line-height: 1.5; }
"""


def main():
    legend = "".join(f'<span><span class="sw" style="background:var({v})"></span>{escape(k)}</span>'
                     for k, v in [("+12 V", "--w12"), ("+5 V", "--w5"), ("GND", "--wgnd"), ("Wago link", "--wlink"),
                                  ("motor +", "--wmotp"), ("motor −", "--wmotn"), ("encoder A", "--wencA"),
                                  ("encoder B", "--wencB"), ("PWM", "--wpwm"), ("touch panel", "--wtouch"), ("buttons", "--wbtn"),
                                  ("power-off control", "--wctl")])
    panels = "".join(f'<section class="panel"><h2>{escape(PANELS[k]["title"])}</h2><div class="scroll">{svg_panel(k)}</div></section>'
                     for k in ("power", "motors", "inputs"))
    html = f"""<meta charset="utf-8">
<title>Ball Balancer Wiring</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;600&display=swap">
<style>{CSS}</style>
<main>
<header style="display:grid;gap:8px">
  <h1>Ball balancer wiring</h1>
  <p>Every connection, generated from the same list as the tables below (<span class="mono">wiring/wiring.py</span>).
  Power and ground run through eight WAGO 221-415 lever connectors: two for +12 V, two for +5 V, four for ground,
  each pair or group linked by a short jumper. Signals are Dupont leads. Hover a wire to see its two ends.</p>
  <div class="legend">{legend}</div>
</header>
{panels}
<section class="panel notes"><h2>Before you power it on</h2><ul>
<li>Fit the motors' leads last. First power-up: no motors connected, check 12 V on the +12 V Wagos and 5.0 V on the +5 V Wagos with a meter.</li>
<li>Never connect the buck's 5 V output and the Mega's USB 5 V together; the Mega takes 12 V on VIN only and makes its own 5 V.</li>
<li>Touch-panel tail order (Y−, X−, Y+, X+) is from a similar panel's datasheet: confirm with a meter (about 200–900 Ω across each X pair and each Y pair).</li>
<li>Motor connector pins, left to right: 1 M+, 2 GND, 3 A, 4 B, 5 5V, 6 M−. Port labels give the pin number.</li>
<li>Our motor cable colours: M+ yellow, GND black, A blue, B purple, 5V red, M− white.</li>
<li>Our button harness colours: NO blue, C green, NC yellow, LED+ red, LED− black.</li>
<li>Button harness colours vary by batch: identify LED+, LED−, C, NO and NC with a meter before connecting.</li>
<li>Program the Mega over USB with the shell lifted; the soft switch can stay off while USB powers the Mega's logic.</li>
<li>Second MDD3A's channel 2 is unused. Leave both MDD3As' 5V pins unconnected: they are outputs from the board's own regulator.</li>
</ul></section>
<h2>Wire list by part</h2>
<div class="tables">{tables()}</div>
</main>
"""
    OUT.write_text(html, encoding="utf-8")
    print(OUT)


if __name__ == "__main__":
    main()
