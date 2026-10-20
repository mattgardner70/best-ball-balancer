"""Every wire in the build: the list the wiring video is drawn from (and the wiring PDF was made from)."""

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
