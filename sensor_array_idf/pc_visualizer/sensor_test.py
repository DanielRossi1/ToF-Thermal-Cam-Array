import serial
from sensor_array_idf.pc_visualizer.protocol import SlipDecoder, parse_message, parse_frame, MSG_FRAME

def on_packet(raw):
    msg = parse_message(raw)
    if msg:
        mtype, seq, ts, payload = msg
        if mtype == MSG_FRAME:
            f = parse_frame(payload)
            if f and f.cam_jpeg:
                print(f"CAM FRAME seq={f.seq} len={len(f.cam_jpeg)}")
            else:
                print(f"frame seq={seq} flags={f.flags if f else '?'} ({f.flags:08b})")  # ← change this line

dec = SlipDecoder(on_packet)
with serial.Serial('/dev/ttyACM0', 2000000, timeout=1) as s:
    while True:
        dec.feed(s.read(4096))