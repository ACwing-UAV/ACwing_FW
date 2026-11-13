#!/usr/bin/env python3
"""Compiles the control-law example of 03 - Docs/firmware/control-api.md against App/control/ctl_api.h
(-Wall -Wextra -Werror), so the guide cannot drift from the interface. Used by the CI."""
import os, re, subprocess, sys, tempfile

here = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(os.path.join(here, "..", "..", ".."))
doc = open(os.path.join(root, "03 - Docs", "firmware", "control-api.md"), encoding="utf-8").read()
m = re.search(r"```c\n(/\* my_law\.c.*?)```", doc, re.S)
if not m:
    sys.exit("no my_law.c example in control-api.md")
with tempfile.TemporaryDirectory() as d:
    src = os.path.join(d, "my_law.c")
    open(src, "w", encoding="utf-8").write(m.group(1))
    r = subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsyntax-only",
                        "-I", os.path.join(root, "00 - FC_FW", "App", "control"), src])
    if r.returncode:
        sys.exit("the example of control-api.md does not compile")
print("control-api.md example compiles")
