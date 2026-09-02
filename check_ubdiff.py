#!/usr/bin/env python3
"""Parse [ub-diff] instrumentation out of a quad-sdk run and report pass/fail.

Usage:  python3 check_ubdiff.py [logfile]
Default logfile: ~/.ros/log/latest/launch.log
"""
import os, re, sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    '~/.ros/log/latest/launch.log')
txt = open(path, errors='replace').read().split('\n')

TS = re.compile(r'\[INFO\] \[(\d+\.\d+)\]')
V3 = r'\(([-+0-9.]+), *([-+0-9.]+), *([-+0-9.]+)\)'

td, foot, refd, cmd = [], [], [], []
switches, retracts, guards, efforts = [], [], [], []
cur_ts = None

for i, ln in enumerate(txt):
    m = TS.search(ln)
    if m:
        cur_ts = float(m.group(1))
    if 'exceeds threshold' in ln:
        efforts.append(ln.strip()[-110:])
    if '[ub-diff][0:guard]' in ln:
        guards.append(ln.strip()[-110:])
    if '[ub-diff][7:switch]' in ln:
        switches.append(ln.strip().split('[ub-diff]')[-1])
    if '[ub-diff][9:cmd-retract]' in ln:
        retracts.append(ln.strip().split('[ub-diff]')[-1])

    m = re.search(r'\[1:td-scan\] leg=(\d+) contact=(\d+) \| '
                  r't_to_TD=([-+0-9.]+) s t_since_LO=([-+0-9.]+)', ln)
    if m:
        td.append((cur_ts, int(m.group(1)), int(m.group(2)),
                   float(m.group(3)), float(m.group(4))))

    m = re.search(r'\[2:foot-ref\] leg=(\d+)', ln)
    if m:
        leg = int(m.group(1))
        blk = '\n'.join(txt[i:i + 6])
        f = re.search(r'footfall_err=' + V3, blk)
        z = re.search(r'z_gate=(\w+)', blk)
        if f:
            foot.append((cur_ts, leg, float(f.group(1)), float(f.group(2)),
                         float(f.group(3)), z.group(1) if z else '?'))

    m = re.search(r'\[5:ref-delta\] leg=(\d+) joint=(\d+)\S* \w* ?contact=(\d+).*?'
                  r'dpos=([-+0-9.]+) rad dvel=([-+0-9.]+)', ln)
    if m:
        refd.append((int(m.group(1)), int(m.group(2)), int(m.group(3)),
                     float(m.group(4)), float(m.group(5))))

    m = re.search(r'\[8:cmd-swing\] leg=(\d+) joint=(\d+).*?'
                  r'dpos=([-+0-9.]+) dvel=([-+0-9.]+) dtau=([-+0-9.]+)', ln)
    if m:
        cmd.append((int(m.group(1)), int(m.group(2)), float(m.group(3)),
                    float(m.group(4)), float(m.group(5))))

print('=' * 72)
print('log: %s' % path)
print('ticks parsed: td-scan=%d foot-ref=%d ref-delta=%d cmd-swing=%d'
      % (len(td), len(foot), len(refd), len(cmd)))
print('=' * 72)

# --- CHECK A: the time-base fix -------------------------------------------
# Join each foot-ref to the td-scan for the same leg on the same tick.
lo = {(round(t, 3), l): slo for (t, l, c, ttd, slo) in td}
early, late = [], []
for (t, leg, ex, ey, ez, zg) in foot:
    slo = lo.get((round(t, 3), leg))
    if slo is None:
        continue
    (early if slo < 0.06 else late).append((slo, leg, ex))

print('\n[A] TIME-BASE FIX -- early-swing footfall target')
print('    Expect: right after liftoff the target is a FULL STEP AHEAD.')
print('    Bug signature: |err_x| < 0.02 m (or negative) at t_since_LO < 0.06 s.')
if not early:
    print('    NO early-swing samples captured. Lower UBDBG_EVERY_N and re-run.')
else:
    bad = [e for e in early if e[2] < 0.02]
    for (slo, leg, ex) in sorted(early)[:10]:
        flag = '  <<< STILL BUGGY' if ex < 0.02 else '  ok'
        print('    t_since_LO=%.3f leg=%d err_x=%+.4f%s' % (slo, leg, ex, flag))
    print('    --> %d/%d early samples still show a stale/behind target'
          % (len(bad), len(early)))
    print('    VERDICT: %s' % ('FAIL - fix did not take' if bad else 'PASS'))

# --- CHECK B: mid-swing discontinuity -------------------------------------
print('\n[B] TARGET DISCONTINUITY -- err_x jump within one swing')
print('    Expect: shrinks monotonically. Bug signature: ~0.2 m forward jump.')
byleg = defaultdict(list)
for (t, leg, ex, ey, ez, zg) in foot:
    byleg[leg].append((t, ex))
worst = []
for leg, seq in byleg.items():
    seq.sort()
    for (t0, x0), (t1, x1) in zip(seq, seq[1:]):
        if t1 - t0 < 0.35 and x1 - x0 > 0.05:   # same swing, forward jump
            worst.append((x1 - x0, leg, t0, x0, x1))
worst.sort(reverse=True)
if not worst:
    print('    No forward jumps > 0.05 m detected.   VERDICT: PASS')
else:
    for (d, leg, t0, x0, x1) in worst[:5]:
        print('    leg=%d  err_x %+.4f -> %+.4f  (jump %+.4f m)' % (leg, x0, x1, d))
    print('    VERDICT: FAIL - target still teleports')

# --- CHECK C: residual UB-vs-ID divergence --------------------------------
print('\n[C] RESIDUAL DIVERGENCE (expected nonzero until the rewrite is gated)')
for label, rows, idx in (('ref-delta swing', [r for r in refd if r[2] == 0], (3, 4)),
                         ('ref-delta stance', [r for r in refd if r[2] == 1], (3, 4))):
    if rows:
        mp = max(abs(r[idx[0]]) for r in rows)
        mv = max(abs(r[idx[1]]) for r in rows)
        print('    %-18s max |dpos|=%.4f rad   max |dvel|=%.4f rad/s'
              % (label, mp, mv))
if cmd:
    print('    %-18s max |dpos|=%.4f  max |dvel|=%.4f  max |dtau|=%.3f Nm'
          % ('cmd-swing', max(abs(c[2]) for c in cmd),
             max(abs(c[3]) for c in cmd), max(abs(c[4]) for c in cmd)))
    print('    (stance ~1e-4 is the control group: that is what "equivalent" looks like)')

# --- CHECK D: retract path + guard + saturation ---------------------------
print('\n[D] RETRACT PATH / GUARD / SATURATION')
print('    [7:switch] mode transitions : %d' % len(switches))
print('    [9:cmd-retract] samples     : %d' % len(retracts))
if not switches:
    print('      -> retract path never exercised (expected at tau_contact_start=1000)')
for s in switches[:6]:
    print('      %s' % s)
print('    [0:guard] no-command ticks  : %d' % len(guards))
for g in guards[:3]:
    print('      %s' % g)
print('    effort saturation warnings  : %d' % len(efforts))
for e in efforts[:5]:
    print('      %s' % e)
print()
