#!/usr/bin/env python3
"""Cross-build of the flight-controller firmware with arm-none-eabi-gcc, from the STM32CubeIDE project itself.

Include paths, defines, source folders, linker script and libraries are read from the chosen configuration of
`.cproject`, so a folder or a path missing there fails here as it would in the IDE. Used by the CI (job
`firmware-target`); locally:  python3 "00 - FC_FW/Tests/target/build_fw.py" [--config Debug] [-j N]

Output in Tests/target/build/<config>/: object files, ACwing_FC_FW.elf, .map, and the size report.
"""
import argparse, os, re, shutil, subprocess, sys, xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))   # 00 - FC_FW
CPU = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]


def config(name):
    tree = ET.parse(os.path.join(ROOT, ".cproject"))
    for c in tree.iter("cconfiguration"):
        mod = c.find("storageModule[@moduleId='cdtBuildSystem']/configuration")
        if mod is not None and mod.get("name") == name:
            return mod
    sys.exit(f"configuration {name!r} not found in .cproject")


def values(cfg, key, tool_kind):
    """listOptionValue values of the options whose superClass contains key, in the tools of tool_kind"""
    out = []
    for tool in cfg.iter("tool"):
        if tool_kind not in (tool.get("superClass") or ""):
            continue
        for opt in tool.iter("option"):
            if key in (opt.get("superClass") or ""):
                out += [v.get("value") for v in opt.iter("listOptionValue")]
    return out


def path(p):
    p = p.replace("&quot;", "").strip('"')
    p = re.sub(r"\$\{workspace_loc:/\$\{ProjName\}/?([^}]*)\}", r"\1", p)
    if p.startswith("../"):
        p = p[3:]
    return os.path.normpath(os.path.join(ROOT, p))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="Debug")
    ap.add_argument("-j", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--max-ram-pct", type=float, default=90.0, help="fail above this RAM use (static), %%")
    ap.add_argument("--max-flash-pct", type=float, default=90.0, help="fail above this FLASH use, %%")
    a = ap.parse_args()
    cfg = config(a.config)

    incs = [path(p) for p in values(cfg, "includepaths", "c.compiler")]
    defs = values(cfg, "definedsymbols", "c.compiler")
    missing = [p for p in incs if not os.path.isdir(p)]
    if missing:
        sys.exit("include paths in .cproject that do not exist:\n  " + "\n  ".join(missing))
    srcs_dirs = [e.get("name") for e in cfg.iter("entry") if e.get("kind") == "sourcePath"]
    excl = {}
    for e in cfg.iter("entry"):
        if e.get("kind") == "sourcePath" and e.get("excluding"):
            excl[e.get("name")] = e.get("excluding").split("|")
    srcs = []
    for d in srcs_dirs:
        base = os.path.join(ROOT, d)
        for dp, _, fs in os.walk(base):
            rel = os.path.relpath(dp, ROOT).replace(os.sep, "/")
            if any(rel == f"{d}/{x}" or rel.startswith(f"{d}/{x}/") for x in excl.get(d, [])):
                continue
            for f in fs:
                if f.endswith((".c", ".s")) and not any(f"{d}/{x}" == f"{rel}/{f}" for x in excl.get(d, [])):
                    srcs.append(os.path.join(dp, f))
    srcs.sort()
    libs = values(cfg, "libraries", "c.linker")
    libdirs = [path(p) for p in values(cfg, "directories", "c.linker")]
    ld = os.path.join(ROOT, "STM32WB55RGVX_FLASH.ld")

    out = os.path.join(ROOT, "Tests", "target", "build", a.config)
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    cflags = CPU + ["-std=gnu11", "-O2", "-g", "-ffunction-sections", "-fdata-sections", "-Wall",
                    "-Wno-unused-function", "-Wno-unused-variable", "--specs=nano.specs"] + \
             [f"-D{d}" for d in defs] + [f"-I{i}" for i in incs]
    print(f"{a.config}: {len(srcs)} sources, {len(incs)} include paths, defines {' '.join(defs)}")

    def compile_one(src):
        obj = os.path.join(out, os.path.relpath(src, ROOT).replace(os.sep, "_").replace(" ", "_") + ".o")
        cmd = ["arm-none-eabi-gcc"] + cflags + (["-x", "assembler-with-cpp"] if src.endswith(".s") else []) + \
              ["-c", src, "-o", obj]
        r = subprocess.run(cmd, capture_output=True, text=True)
        return src, obj, r.returncode, r.stderr

    objs, failed = [], 0
    with ThreadPoolExecutor(a.j) as ex:
        for src, obj, rc, err in ex.map(compile_one, srcs):
            if err.strip():
                print(err.rstrip())
            if rc:
                failed += 1
            else:
                objs.append(obj)
    if failed:
        sys.exit(f"{failed} source(s) failed to compile")
    elf = os.path.join(out, "ACwing_FC_FW.elf")
    cmd = ["arm-none-eabi-gcc"] + CPU + ["-T", ld, "--specs=nano.specs", "--specs=nosys.specs",
           "-Wl,--gc-sections", f"-Wl,-Map={elf[:-4]}.map", "-Wl,--print-memory-usage", "-o", elf] + objs + \
          [f"-L{d}" for d in libdirs] + ["-Wl,--start-group"] + [f"-l{l}" for l in libs] + ["-lc", "-lm", "-Wl,--end-group"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    sys.stdout.write(r.stdout)
    if r.returncode:
        sys.stderr.write(r.stderr)
        sys.exit("link failed")
    subprocess.run(["arm-none-eabi-size", elf])
    # memory budget: the static RAM leaves the boot stack (also the interrupt stack) and the heap headroom
    over = []
    for region, limit in (("RAM", a.max_ram_pct), ("FLASH", a.max_flash_pct)):
        m = re.search(rf"^\s*{region}:\s+\d+ \w+\s+\d+ \w+\s+([\d.]+)%", r.stdout, re.M)
        if not m:
            sys.exit(f"no {region} line in the memory report")
        pct = float(m.group(1))
        print(f"{region} {pct:.2f} % (budget {limit:.0f} %)")
        if pct > limit:
            over.append(f"{region} {pct:.2f} % > {limit:.0f} %")
    if over:
        sys.exit("memory budget exceeded: " + ", ".join(over))


if __name__ == "__main__":
    main()
