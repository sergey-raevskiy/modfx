# Generate ICR values for tempo potentiometer
#
#   0 -> 75
# 128 -> 150
# 255 -> 300

import math

# 16 MHz CPU clock
fcpu = 16000000
note = 1.0/4

# Convert potentiometer position to bpm
def bpm(v):
    return 75 * math.pow(2, v / 127.5)

def bpm2icr(v):
    freq = v / 60.0 * (1.0/4 / note)
    return fcpu / (freq * 256)

for i in range(256):
    print "pos = %d, bpm = %d, icr = %d" % (i, bpm(i), bpm2icr(bpm(i)))

s = ""
for i in range(256):
    if (i % 16 == 0):
        s = s + "\n"
    s = s + "%d, " % (bpm2icr(bpm(i)))

print s
