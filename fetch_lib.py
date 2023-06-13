import subprocess as sp
import os
libs = "linear_algebra transform2d".split()

def run(cmd):
    if type(cmd) == list:
        cmd = " ".join(cmd)
    print(cmd)
    sp.run(cmd, shell=True)

for l in libs:
    run(["rm", l, "-r"])
    src = os.path.join("../../libwheels", l)
    run(["cp", src, l, "-r"])

