# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission of
# Rockwell Automation Technologies, Inc.
"""Isaac-Sim-side worker for ``urdf2usd_export``.

Must run under Isaac Sim's bundled Python (``${ISAAC_SIM_PATH}/python.sh``).
Not meant to be called directly: ``urdf2usd_export`` ``exec``s it after writing
the expanded URDF and the parsed ``<isaac_inputs>`` JSON to disk.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import sys
import threading
from pathlib import Path


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="urdf2usd._isaac_worker",
        description="Isaac Sim worker: import URDF, post-process, export USD.",
    )
    p.add_argument("--urdf-file", required=True, help="Expanded URDF on disk.")
    p.add_argument("--inputs-json", required=True, help="Parsed <isaac_inputs> JSON.")
    p.add_argument("--output", required=True, help="Destination USD path.")
    p.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    p.add_argument("--keep-open", action="store_true")
    p.add_argument(
        "--cleanup-intermediates",
        action="store_true",
        help="Delete --urdf-file and --inputs-json after a successful export.",
    )
    return p


def _close_sim_app_with_timeout(sim_app, timeout_sec: float = 8.0) -> None:
    """Close SimulationApp on a daemon thread so a hung shutdown can't block."""
    close_error: list[BaseException] = []

    def _target() -> None:
        try:
            sim_app.close()
        except BaseException as exc:  # pragma: no cover - defensive path
            close_error.append(exc)

    closer = threading.Thread(target=_target, name="urdf2usd-sim-close", daemon=True)
    closer.start()
    closer.join(timeout=timeout_sec)

    if closer.is_alive():
        print(
            f"[urdf2usd] WARNING: SimulationApp.close() timed out after {timeout_sec:.1f}s; "
            "forcing process exit.",
            file=sys.stderr,
        )
        sys.stderr.flush()
        return

    if close_error:
        print(f"[urdf2usd] WARNING: SimulationApp.close() raised: {close_error[0]!r}", file=sys.stderr)
        sys.stderr.flush()


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)

    urdf_file = Path(args.urdf_file).expanduser().resolve()
    inputs_file = Path(args.inputs_json).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()

    urdf_xml = urdf_file.read_text(encoding="utf-8")
    cfg = json.loads(inputs_file.read_text(encoding="utf-8"))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    # The URDF importer writes its converted USD into a temp working dir; chdir
    # into the output directory anyway so any relative artefacts land next to
    # the final USD.
    os.chdir(output_path.parent)

    print(f"[urdf2usd] Booting SimulationApp (headless={args.headless})")
    from isaacsim import SimulationApp  # type: ignore[import-not-found]
    sim_app = SimulationApp({"headless": args.headless})

    from isaacsim.core.utils.extensions import enable_extension  # type: ignore[import-not-found]
    enable_extension("isaacsim.ros2.bridge")
    sim_app.update()

    export_ok = False
    try:
        from urdf2usd import apply_isaac_inputs, import_urdf

        prim_path = import_urdf(urdf_xml, cfg.get("import_config"))
        print(f"[urdf2usd] Imported robot at prim: {prim_path}")

        apply_isaac_inputs(prim_path, cfg, urdf_xml=urdf_xml)

        import omni.usd  # type: ignore[import-not-found]

        stage = omni.usd.get_context().get_stage()

        # The URDF importer references visual meshes from anonymous in-memory
        # sublayers, which vanish on sim_app.close(). Flatten the layer stack
        # into one self-contained layer before exporting.
        flattened = stage.Flatten()
        if not flattened.Export(str(output_path)):
            raise RuntimeError(f"USD Export() returned False for {output_path}")
        print(f"[urdf2usd] Saved USD (flattened): {output_path}")
        export_ok = True

        if args.keep_open:
            print("[urdf2usd] --keep-open set; running simulation loop. Ctrl-C to exit.")
            try:
                while sim_app.is_running():
                    sim_app.update()
            except KeyboardInterrupt:
                print("\n[urdf2usd] Interrupted.")
    except Exception:
        # Print the traceback before sim_app.close() runs; Kit's shutdown can
        # drop pending Python stderr.
        import traceback
        print("[urdf2usd] ERROR during conversion:", file=sys.stderr)
        traceback.print_exc()
        sys.stderr.flush()
    finally:
        _close_sim_app_with_timeout(sim_app)

    if export_ok and args.cleanup_intermediates:
        for f in (urdf_file, inputs_file):
            with contextlib.suppress(OSError):
                f.unlink()

    return 0 if export_ok else 1


if __name__ == "__main__":
    # Kit can leave non-daemon threads alive after close(); os._exit guarantees
    # the process terminates so shell chains like "cmd1 && cmd2" can continue.
    exit_code = main()
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        os._exit(exit_code)
