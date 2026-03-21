import re
with open('tesla_can.dbc') as f:
    lines = f.readlines()
cm = None
ks = {}
for l in lines:
    m = re.match(r'BO_ (\d+) (\S+)', l)
    if m:
        cm = (int(m.group(1)), m.group(2))
    s = re.match(r' SG_ (\S+)', l)
    if s and cm:
        n = s.group(1)
        for k in ['vehicleSpeed','elecPower','torque','packVol','packCur','socUI','odom','gear','brakeTorq','steeringAngle','wheelSpeed']:
            if k in n:
                ks[n] = cm
                break
for n, (i, mn) in sorted(ks.items()):
    print(f'0x{i:03X} {mn:40s} -> {n}')
