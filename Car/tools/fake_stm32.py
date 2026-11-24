# Fake STM32 on a pty (test only, no hardware): answers M,l,r with ENC,l,r every 50 ms.
# Motor model: rev/s = 0.1 * (|duty| - 8) above an 8% dead zone. Raw left encoder counts DOWN when
# the wheel goes forward (matches invert_enc_left=true). --reverse-right makes the right motor wired backwards.
# --enc-dies-after S stops sending ENC after S seconds. 300 ms UART watchdog like the real board.
import os, sys, time, tty, select, argparse
ap = argparse.ArgumentParser(); ap.add_argument('--reverse-right', action='store_true'); ap.add_argument('--enc-dies-after', type=float, default=0)
ap.add_argument('--link', required=True); a = ap.parse_args()
master, slave = os.openpty(); tty.setraw(master); tty.setraw(slave)
name = os.ttyname(slave)
if os.path.lexists(a.link): os.unlink(a.link)
os.symlink(name, a.link); print(name, flush=True); os.close(slave)
TICKS = (687, 685); duty = [0, 0]; last_cmd = 0.0; pos = [0.0, 0.0]; buf = b''; t0 = time.monotonic(); nxt = t0
log = open(a.link + '.cmds', 'w')
while True:
    r, _, _ = select.select([master], [], [], 0.005)
    if r:
        try:
            buf += os.read(master, 1024)
        except OSError:
            time.sleep(0.01); continue
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1); s = line.decode().strip(); log.write(f'{time.monotonic()-t0:.3f} {s}\n'); log.flush()
            p = s.split(',')
            if len(p) == 3 and p[0] == 'M': duty = [int(p[1]), int(p[2])]; last_cmd = time.monotonic()
    now = time.monotonic()
    if now - last_cmd > 0.3: duty = [0, 0]
    if now >= nxt:
        nxt += 0.05
        for i in (0, 1):
            d = duty[i] * (-1 if (i == 1 and a.reverse_right) else 1)
            rps = 0.1 * max(0, abs(d) - 8) * (1 if d > 0 else -1)
            pos[i] += rps * TICKS[i] * 0.05
        raw = [int(-pos[0]), int(pos[1])]           # left encoder counts down when forward
        if a.enc_dies_after and now - t0 > a.enc_dies_after: continue
        try:
            os.write(master, ('ENC,%d,%d\n' % tuple(v % 2**32 for v in raw)).encode())
        except OSError:
            pass
