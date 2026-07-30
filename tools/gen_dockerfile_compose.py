#!/usr/bin/env python3
"""
drone-stack generator: read a stack (list of modules) + each module's manifest,
union their declared deps (arch-aware), and emit ONE Dockerfile + ONE compose file
for that stack. Driven by setup.sh.

    python3 tools/gen_dockerfile_compose.py <stack> [--arch arm64|amd64] [--gpu-arch sm120]

Writes .build/<stack>/Dockerfile and .build/<stack>/compose.yml.

GPU-arch dimension (added 2026-07-14, backward compatible):
CPU arch (arm64/amd64) alone isn't enough to pick GPU deps — the same amd64 host can
be sm_75 (2080 Ti), sm_86/89 (30/40-series), or sm_120 (50-series/Blackwell), and
arm64 Jetson Orin is sm_87. A stack may declare `gpu_arch: sm120` (etc.); modules can
then add a flat combo key `deps.<cpu_arch>_<gpu_arch>` (e.g. `deps.amd64_sm120`) on
top of the existing `deps.<cpu_arch>` — see `arch_merged()`. If a stack does NOT set
`gpu_arch` (e.g. the existing arm64 Jetson stacks), this combo key is never consulted
and generation is byte-identical to before this change — verified by diffing
d435i-voxblox's generated Dockerfile/compose.yml pre/post this patch.

(2026-07-25 to 2026-07-26: a `build_env:` dimension briefly existed here to scope
sim-x86's source-built ABI=1 torch wheel to one stack without touching the shared
(cpu_arch, gpu_arch) combo keys — removed 2026-07-26 once that wheel became
compute/torch's unconditional sm89/sm75 default, see docs/ETE_TRAIN_GPU_HOSTS.md's
"torch unification" section.)
"""
import os, sys, argparse, yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_env(path):
    env = dict(os.environ)
    # `<path>.local` is an untracked per-host override that WINS over the tracked
    # file. Without it every host has to edit config/stack.env itself, which makes
    # a tracked file permanently dirty and conflicts on every pull -- im hit exactly
    # that with ETE_DATA_DIR (2026-07-26) on a checkout that is supposed to be
    # pull-only. Read the override FIRST: setdefault is first-writer-wins, so the
    # real environment still beats both, then .local, then the shared defaults.
    # (The shell side is handled inside stack.env itself, which sources its own
    # .local at the end -- so setup.sh and the in-container run.sh's need no change.
    # That sourcing line has no "=" so the loop below skips it, as intended.)
    for p in (path + ".local", path):
        if not os.path.isfile(p):
            continue
        for line in open(p):
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            env.setdefault(k.strip(), v.split("#", 1)[0].strip())
    return env


def expand(s, env):
    out = s
    for k, v in env.items():
        out = out.replace("${%s}" % k, v).replace("$%s" % k, v)
    return out


def module_dir(path):
    return os.path.join(ROOT, "modules", path)


def load_module(path):
    f = os.path.join(module_dir(path), "module.yml")
    if not os.path.isfile(f):
        sys.exit("ERROR: module manifest not found: %s" % f)
    m = yaml.safe_load(open(f)) or {}
    m["_path"] = path
    return m


def resolve_order(stack_modules):
    """base first, then stack modules with their `needs` (deps before dependents)."""
    order, seen = [], set()

    def visit(path):
        if path in seen:
            return
        seen.add(path)
        m = load_module(path)
        for dep in (m.get("needs") or []):
            visit(dep)
        order.append(m)

    visit("base")
    for p in stack_modules:
        visit(p)
    return order


def arch_merged(deps, arch, gpu_arch, kind):
    """deps.<kind> + deps.<arch>.<kind> + deps.<arch>_<gpu_arch>.<kind> (only when
    gpu_arch is set), order-preserving, de-duped. The gpu_arch combo layer is purely
    additive on top of the existing (arch)-only layer, and is skipped entirely when
    gpu_arch is None/empty — so callers that never pass a gpu_arch (i.e. every stack
    that doesn't declare `gpu_arch:`) get exactly the pre-existing merge behavior.
    """
    base = list((deps or {}).get(kind) or [])
    extra = list(((deps or {}).get(arch) or {}).get(kind) or [])
    combo = []
    if gpu_arch:
        combo = list(((deps or {}).get("%s_%s" % (arch, gpu_arch)) or {}).get(kind) or [])
    out = []
    for x in base + extra + combo:
        if x not in out:
            out.append(x)
    return out


def gen_dockerfile(mods, arch, gpu_arch, env):
    base_mod = mods[0]
    base_images = base_mod.get("base_image") or {}
    # combo key first (e.g. `amd64_sm89` for a driver-535-capped 4090 host that needs
    # an older CUDA line than the default amd64 image), fall back to the plain
    # cpu-arch key — identical lookup/behavior to before when gpu_arch is unset.
    bimg = base_images.get("%s_%s" % (arch, gpu_arch)) if gpu_arch else None
    if not bimg:
        bimg = base_images.get(arch)
    if not bimg:
        sys.exit("ERROR: base module declares no base_image for arch '%s'%s" % (
            arch, (" (or combo '%s_%s')" % (arch, gpu_arch)) if gpu_arch else ""))

    L = []
    # parser directive MUST be the very first line (enables BuildKit RUN --mount)
    L.append("# syntax=docker/dockerfile:1")
    L.append("# GENERATED by tools/gen_dockerfile_compose.py — do not edit. Stack arch=%s%s"
              % (arch, (" gpu_arch=%s" % gpu_arch) if gpu_arch else ""))
    L.append("ARG TARGETARCH=%s" % arch)
    L.append("FROM %s" % bimg)
    L.append("ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 TARGETARCH=%s" % arch)
    L.append('SHELL ["/bin/bash", "-c"]')
    # NOTE: modules/ is NOT copied in. Each install.sh runs via a transient BuildKit
    # bind-mount (see the RUN --mount lines below), so nothing from modules/ — not the
    # scripts, not the jax wheel — is baked into the image. No /modules at runtime;
    # the only modules tree in a running container is the /work bind-mount (the live repo).
    #
    # CACHE: BuildKit hashes the CONTENT of every bind-mounted path into the layer's
    # cache key (verified: a whole-`modules/` mount re-runs on ANY file edit under
    # modules/, even an unrelated run.sh). So each RUN mounts ONLY its own install
    # script + the module's declared `assets:` — editing run scripts / launch files /
    # other modules never invalidates an install layer.
    L.append("")

    for m in mods:
        path = m["_path"]
        deps = m.get("deps") or {}
        apt = arch_merged(deps, arch, gpu_arch, "apt")
        # pip is split in two: `pip_strict` (base + cpu-arch layer, same as before this
        # change) always hard-fails the build like every other RUN; `pip_combo_only`
        # (the NEW gpu_arch combo-key layer, e.g. deps.amd64_sm120.pip) is emitted as
        # its own separate RUN line, soft-failing only when the module opts in via
        # `allow_pip_fail: true` — so a "try this wheel, else install.sh builds from
        # source" combo entry can't accidentally mask a real failure of the base/
        # cpu-arch pip deps (those still fail the build normally).
        pip_strict = arch_merged(deps, arch, None, "pip")
        pip_all = arch_merged(deps, arch, gpu_arch, "pip")
        pip_combo_only = [p for p in pip_all if p not in pip_strict]
        src = arch_merged(deps, arch, gpu_arch, "source")
        for k, v in (m.get("env") or {}).items():
            L.append("ENV %s=%s" % (k, v))
        if not (apt or pip_strict or pip_combo_only or src):
            continue
        L.append("# ---- module: %s (%s) ----" % (m.get("name", path), path))
        if apt:
            L.append("RUN apt-get update && apt-get install -y --no-install-recommends \\")
            L.append("      " + " ".join(apt) + " \\")
            L.append(" && rm -rf /var/lib/apt/lists/*")
        if pip_strict:
            L.append("RUN python3 -m pip install --no-cache-dir \\")
            L.append("      " + " ".join('"%s"' % p for p in pip_strict))
        if pip_combo_only:
            L.append("RUN python3 -m pip install --no-cache-dir \\")
            pip_line = "      " + " ".join('"%s"' % p for p in pip_combo_only)
            # allow_pip_fail is read from the SPECIFIC active combo key (not a
            # module-wide flag) -- a module can declare several (cpu_arch, gpu_arch)
            # combos with different tolerance, e.g. spconv's amd64_sm120 (soft-fail,
            # wheel coverage of a brand-new arch is genuinely uncertain) vs its
            # amd64_sm89 (hard-fail, sm_89 is well-covered by published wheels so a
            # failure here means something is actually broken, not "expected gap").
            combo_deps = (deps.get("%s_%s" % (arch, gpu_arch)) or {}) if gpu_arch else {}
            if combo_deps.get("allow_pip_fail"):
                pip_line += " || true  # allow_pip_fail: true (gpu_arch=%s combo) — fallback is this module's install.sh" % gpu_arch
            L.append(pip_line)
        for s in src:
            # transient bind-mounts, scoped to this module's script (+ declared assets)
            # so the layer's cache key ignores everything else under modules/
            binds = [(s, True)] + [(a, False) for a in (m.get("assets") or [])]
            mnt = " \\\n    ".join(
                "--mount=type=bind,source=modules/%s/%s,target=/modules/%s/%s"
                % (path, f, path, f) for f, _ in binds)
            # GPU_ARCH=... only added to the invocation when the stack sets gpu_arch,
            # so a stack without it (all existing arm64 stacks today) emits the exact
            # same `TARGETARCH=... bash ...` line as before this change.
            envs = "TARGETARCH=%s" % arch
            if gpu_arch:
                envs += " GPU_ARCH=%s" % gpu_arch
            L.append("RUN %s \\\n    %s bash /modules/%s/%s" % (mnt, envs, path, s))
        L.append("")
    return "\n".join(L) + "\n"


def gen_compose(mods, arch, stack, env, gpu_uuids_key="GPU_UUIDS", gpu=True):
    image = "drone-stack:%s" % stack
    mounts, runs = [], []
    for m in mods:
        for mt in (m.get("mounts") or []):
            mt = expand(mt, env)
            if mt not in mounts:
                mounts.append(mt)
        for r in (m.get("run") or []):
            runs.append("modules/%s/%s" % (m["_path"], r))
    doc = {
        "services": {
            "dev": {
                "image": image,
                "build": {
                    "context": "../..",
                    "dockerfile": ".build/%s/Dockerfile" % stack,
                },
                "container_name": "drone-stack-%s" % stack,
                "network_mode": "host",
                "privileged": True,
                # gpu=False (stacks/*.yml `gpu: false`) drops ALL GPU wiring — the
                # arm64 `runtime: nvidia` here and the amd64 device-reservation /
                # NVIDIA_VISIBLE_DEVICES paths further down. Needed for a stack whose
                # host has no NVIDIA GPU (epic-x86): compose refuses to START a
                # container whose device reservation cannot be satisfied, so the
                # amd64 default below would make an otherwise-fine CPU stack undeployable.
                # Defaults to True -> byte-identical output for every existing stack.
                "runtime": "nvidia" if (gpu and arch == "arm64") else None,
                "working_dir": "/work",
                "volumes": ["../..:/work"] + mounts,
                # ROS_MASTER_URI / ROS_IP are NOT set here on purpose — they live in
                # config/ros_env.sh (mounted), so the IP is edit-and-go with no recreate.
                "environment": [
                    "DISPLAY=${DISPLAY:-:0}",
                ],
                "command": ["sleep", "infinity"],
            }
        }
    }
    # drop None runtime (compose dislikes null)
    if doc["services"]["dev"]["runtime"] is None:
        del doc["services"]["dev"]["runtime"]
    # CONTAINER_USER (optional, config/stack.env, "uid:gid" e.g. "1000:1000"):
    # confirmed on a real ete-train-2080ti run 2026-07-14 -- without this, the "dev"
    # container's default process (and anything `docker exec`'d into it without an
    # explicit `-u`) runs as root, so training writes checkpoints/tensorboard logs
    # into the bind-mounted outputs/ dir as root-owned files the host user can't
    # later delete/overwrite without sudo. Scoped to amd64 only (not applied to the
    # arm64/Jetson stacks -- those rely on root for udev/device access the same way
    # `privileged: true` does; changing that is out of scope here and untested).
    # Unset by default -- byte-identical to before this option existed.
    container_user = (env.get("CONTAINER_USER") or "").strip()
    if arch == "amd64" and container_user:
        doc["services"]["dev"]["user"] = container_user
        # the numeric uid/gid has no /etc/passwd entry in the image -> no default
        # HOME, which breaks things that need to write a cache dir (matplotlib font
        # cache, etc). /tmp is always writable regardless of uid.
        doc["services"]["dev"]["environment"].append("HOME=/tmp")
        # ...and no passwd entry also means `pwd.getpwuid(os.getuid())` raises KeyError.
        # Python's getpass.getuser() checks USER/LOGNAME/LNAME/USERNAME env FIRST and only
        # falls back to the pwd db, so setting USER/LOGNAME is enough — no image change.
        # 2026-07-26, found on a real ml ete-train-2080ti run: torch.compile's inductor
        # calls getpass.getuser() to name its cache dir, so ANY config with compile_* on
        # (e.g. every v23_P1_* ablation) crashes with `KeyError: getpwuid(): uid not found`
        # the moment CONTAINER_USER is set. Latent since CONTAINER_USER was introduced
        # (2026-07-14) -- unrelated to that day's torch swap, inductor behaved the same on
        # the previous torch. The value is cosmetic (only names a cache dir); "user" keeps
        # it uid-agnostic so this stays correct whatever CONTAINER_USER is set to.
        doc["services"]["dev"]["environment"].append("USER=user")
        doc["services"]["dev"]["environment"].append("LOGNAME=user")
    # amd64 has no nvidia-docker "runtime" shorthand wired the way arm64/L4T does;
    # use the standard compose device-reservation form instead so amd64 GPU stacks
    # (e.g. ete-train-5090) actually get the GPU. Purely additive — arm64 output is
    # unaffected (still just `runtime: nvidia` as before).
    #
    # GPU_UUIDS (optional, config/stack.env): comma-separated GPU UUIDs (`nvidia-smi
    # -L`). Needed on hosts with a dead/NVML-broken card mixed in with usable ones
    # (see docs/ETE_TRAIN_GPU_HOSTS.md's 2080ti section) -- CONFIRMED on real
    # hardware 2026-07-14 (ml desktop, dead card at PCI 1a:00.0): the modern
    # `deploy.resources.reservations.devices` mechanism (what compose translates
    # `--gpus`/device reservations into) fails ENTIRELY on such a host -- even
    # restricted to one healthy UUID, `nvidia-container-cli` still does a full
    # NVML device-count enumeration first and errors out on the dead card before it
    # ever gets to honoring the restriction. The only thing that actually works
    # there is the OLDER `runtime: nvidia` + `NVIDIA_VISIBLE_DEVICES` env-var path
    # (confirmed via plain `docker run --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=...`).
    # So: when GPU_UUIDS is set, use that legacy path instead of a device
    # reservation; when unset (5090/4090 hosts, no known dead-GPU issue), keep the
    # device-reservation form exactly as before.
    #
    # gpu_uuids_key (optional, stacks/*.yml `gpu_uuids_env:`, added 2026-07-25 for
    # sim-x86): lets a stack read its GPU UUID list from a DIFFERENT config/stack.env
    # key than the shared `GPU_UUIDS` (which the ete-train-* amd64 stacks use, set
    # per-invocation via shell export -- see that key's own comment in stack.env).
    # sim-x86 needs its own fixed value (SIM_GPU_UUIDS) baked into stack.env without
    # touching GPU_UUIDS's blank-by-default, export-at-invocation convention. Defaults
    # to "GPU_UUIDS" (see this function's signature) -- a stack that never sets
    # `gpu_uuids_env:` gets byte-identical behavior to before this parameter existed.
    gpu_uuids = [u.strip() for u in (env.get(gpu_uuids_key) or "").split(",") if u.strip()]
    if not gpu:
        pass                        # `gpu: false` — no runtime, no reservation (see above)
    elif arch == "amd64" and gpu_uuids:
        doc["services"]["dev"]["runtime"] = "nvidia"
        doc["services"]["dev"]["environment"].append("NVIDIA_VISIBLE_DEVICES=%s" % ",".join(gpu_uuids))
        doc["services"]["dev"]["environment"].append("NVIDIA_DRIVER_CAPABILITIES=all")
    elif arch == "amd64":
        doc["services"]["dev"]["deploy"] = {
            "resources": {"reservations": {"devices": [
                {"driver": "nvidia", "capabilities": ["gpu"], "count": 1}
            ]}}
        }
    header = "# GENERATED by tools/gen_dockerfile_compose.py. run scripts for this stack:\n#   " + \
             "\n#   ".join(runs) + "\n"
    return header + yaml.safe_dump(doc, sort_keys=False, default_flow_style=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stack")
    ap.add_argument("--arch", default=None)
    ap.add_argument("--gpu-arch", default=None,
                     help="GPU micro-arch tag (e.g. sm120/sm87/sm75). Defaults to the "
                          "stack file's `gpu_arch:`. Optional — omit entirely for "
                          "existing stacks that don't need GPU-variant deps.")
    a = ap.parse_args()

    env = load_env(os.path.join(ROOT, "config", "stack.env"))
    sf = os.path.join(ROOT, "stacks", a.stack + ".yml")
    if not os.path.isfile(sf):
        sys.exit("ERROR: stack not found: %s" % sf)
    stack = yaml.safe_load(open(sf)) or {}

    arches = stack.get("arch")
    arches = [arches] if isinstance(arches, str) else (arches or ["arm64"])
    arch = a.arch or arches[0]
    if arch not in arches:
        sys.exit("ERROR: arch %s not in stack's supported %s" % (arch, arches))
    # gpu_arch: CLI override > stack.yml `gpu_arch:` > None (no GPU-variant deps used).
    gpu_arch = a.gpu_arch or stack.get("gpu_arch")

    mods = resolve_order(stack.get("modules") or [])
    outdir = os.path.join(ROOT, ".build", a.stack)
    os.makedirs(outdir, exist_ok=True)
    open(os.path.join(outdir, "Dockerfile"), "w").write(gen_dockerfile(mods, arch, gpu_arch, env))
    gpu_uuids_key = stack.get("gpu_uuids_env") or "GPU_UUIDS"
    # `gpu:` (optional, stacks/*.yml) — set false for a stack that must run on a host
    # with no NVIDIA GPU. Defaults true, so every pre-existing stack is unaffected.
    gpu = stack.get("gpu", True)
    open(os.path.join(outdir, "compose.yml"), "w").write(
        gen_compose(mods, arch, a.stack, env, gpu_uuids_key, gpu))
    open(os.path.join(outdir, "modules.txt"), "w").write("\n".join(m["_path"] for m in mods) + "\n")

    print("stack '%s' arch=%s%s  modules: %s" % (
        a.stack, arch, (" gpu_arch=%s" % gpu_arch) if gpu_arch else "",
        ", ".join(m["_path"] for m in mods)))
    print("  -> %s/Dockerfile" % outdir)
    print("  -> %s/compose.yml" % outdir)


if __name__ == "__main__":
    main()
