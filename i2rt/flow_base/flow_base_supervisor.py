"""Supervisor daemon to remotely start/stop the FlowBase controller.

This long-lived process runs on the base's Raspberry Pi (alongside, but
independent of, ``flow_base_controller.py``). It exposes a tiny ``portal`` RPC
surface (``start`` / ``stop`` / ``status``) that lets a remote follower spawn or
kill ``flow_base_controller.py`` on demand instead of an operator SSHing in by
hand.

Design notes:

- Imports only stdlib + ``portal`` (already an i2rt dependency). It deliberately
  pulls in no numpy/CAN/ruckig, so the supervisor keeps running even when the
  hardware deps are flaky or absent.
- The controller script itself is never modified. We spawn it exactly as the
  operator would (see ``docs/09_LinearbotIntegration.md``) in its own process
  group, and stop it with SIGTERM so the controller's own ``atexit`` cleanup
  (neutral motors + linear-rail brake) runs.
- ``start`` cross-checks ``/tmp/base-controller.pid`` (written by the
  controller's ``create_pid_file``) so a controller launched manually (desktop
  icon / SSH) is detected and reported as ``already_running`` rather than
  double-spawned.
- The corresponding :class:`FlowBaseSupervisorClient` lives in this module so all
  supervisor RPC detail is in one place (symmetric with ``FlowBaseClient``).
"""

import argparse
import atexit
import logging
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import portal

# Port for the supervisor RPC server. Distinct from the controller's
# BASE_DEFAULT_PORT (11323, flow_base_controller.py) so both run on one host.
# Defined here (not imported from flow_base_controller) so this daemon pulls in
# only stdlib + portal and keeps running even when numpy/CAN/ruckig are flaky.
SUPERVISOR_DEFAULT_PORT = 11324
CONTROLLER_DEFAULT_PORT = 11323

# PID file written by flow_base_controller.create_pid_file("base-controller").
_CONTROLLER_PID_FILE = Path("/tmp/base-controller.pid")

# Defaults for the controller launch command. Overridable via CLI/env so no
# absolute path is hardcoded into logic (per CLAUDE.md). The fallbacks mirror the
# documented manual invocation in docs/09_LinearbotIntegration.md.
_DEFAULT_VENV = os.getenv("FLOW_BASE_VENV", "/home/i2rt/JH_i2rt/JH_i2rt")
_DEFAULT_ROOT = os.getenv("FLOW_BASE_ROOT", "/home/i2rt/JH_i2rt")
_DEFAULT_CHANNEL = os.getenv("FLOW_BASE_CHANNEL", "can0")

# Seconds to wait for a SIGTERM'd controller to exit before escalating to SIGKILL.
_DEFAULT_STOP_GRACE_S = 10.0

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("FlowBaseSupervisor")


def _read_controller_pid() -> Optional[int]:
    """Return the live controller PID from the PID file, or None.

    Reads /tmp/base-controller.pid and verifies the process is still alive via
    os.kill(pid, 0) (same liveness idiom as flow_base_controller.create_pid_file).
    Returns None when the file is missing, malformed, or the process is dead.
    """
    if not _CONTROLLER_PID_FILE.exists():
        return None
    try:
        pid = int(_CONTROLLER_PID_FILE.read_text(encoding="utf-8").strip())
    except (ValueError, OSError):
        return None
    try:
        os.kill(pid, 0)
    except OSError:
        return None
    return pid


def build_controller_command(
    venv: str,
    root: str,
    channel: str,
    no_linear_rail: bool = False,
) -> List[str]:
    """Build the shell command that launches flow_base_controller.py.

    Mirrors the documented manual invocation:
        source <venv>/bin/activate && PYTHONPATH=<root> \\
            python <root>/i2rt/flow_base/flow_base_controller.py --channel <chan>

    Args:
        venv: Path to the virtualenv whose bin/activate is sourced.
        root: Repository root placed on PYTHONPATH and used to locate the script.
        channel: CAN channel passed to the controller (e.g. "can0").
        no_linear_rail: When True, append --no-linear-rail (base-only).

    Returns:
        A command list suitable for subprocess.Popen (a single bash -lc invocation).
    """
    script = f"{root}/i2rt/flow_base/flow_base_controller.py"
    inner = (
        f"source {venv}/bin/activate && "
        f"PYTHONPATH={root} python {script} --channel {channel}"
    )
    if no_linear_rail:
        inner += " --no-linear-rail"
    return ["bash", "-lc", inner]


class ControllerProcess:
    """Owns at most one flow_base_controller.py child process.

    Thread-safe: portal may dispatch handlers from multiple threads, so every
    public method takes an internal lock.
    """

    def __init__(self, command: List[str], stop_grace_s: float = _DEFAULT_STOP_GRACE_S):
        self._command = command
        self._stop_grace_s = stop_grace_s
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    def _is_running_locked(self) -> Tuple[bool, Optional[int]]:
        """Return (running, pid). Prefers our own child, falls back to PID file."""
        if self._proc is not None and self._proc.poll() is None:
            return True, self._proc.pid
        pid = _read_controller_pid()
        if pid is not None:
            return True, pid
        return False, None

    def start(self) -> Dict[str, Any]:
        """Spawn the controller if not already running.

        Returns:
            {"started": bool, "already_running": bool, "pid": int|None}.
        """
        with self._lock:
            running, pid = self._is_running_locked()
            if running:
                logger.info("start: controller already running (pid %s)", pid)
                return {"started": False, "already_running": True, "pid": pid}
            logger.info("start: launching controller: %s", " ".join(self._command))
            # start_new_session=True puts the child in its own process group so a
            # later SIGTERM to the group reaches the controller (and any shell
            # subprocesses it spawned).
            self._proc = subprocess.Popen(self._command, start_new_session=True)
            return {"started": True, "already_running": False, "pid": self._proc.pid}

    def stop(self) -> Dict[str, Any]:
        """SIGTERM the controller (escalating to SIGKILL after a grace period).

        SIGTERM lets the controller's atexit handler run (neutral motors + rail
        brake). Handles both a child we started and an externally-started
        controller known only through the PID file.

        Returns:
            {"stopped": bool} — False only when nothing was running.
        """
        with self._lock:
            running, pid = self._is_running_locked()
            if not running:
                self._proc = None
                logger.info("stop: no controller running")
                return {"stopped": False}

            if self._proc is not None and self._proc.poll() is None:
                self._terminate_group(self._proc.pid)
                try:
                    self._proc.wait(timeout=self._stop_grace_s)
                except subprocess.TimeoutExpired:
                    logger.warning("stop: SIGTERM timed out, sending SIGKILL")
                    self._kill_group(self._proc.pid)
                    try:
                        self._proc.wait(timeout=self._stop_grace_s)
                    except subprocess.TimeoutExpired:
                        logger.error("stop: controller did not exit after SIGKILL")
                self._proc = None
                logger.info("stop: controller stopped (was our child)")
                return {"stopped": True}

            # Externally-started controller: only the PID file identifies it.
            self._terminate_pid(pid)
            logger.info("stop: SIGTERM sent to external controller (pid %s)", pid)
            return {"stopped": True}

    def status(self) -> Dict[str, Any]:
        """Return {"running": bool, "pid": int|None}."""
        with self._lock:
            running, pid = self._is_running_locked()
            return {"running": running, "pid": pid}

    def stop_if_owned(self) -> None:
        """atexit hook: stop a controller this supervisor started, to avoid orphans."""
        if self._proc is not None and self._proc.poll() is None:
            logger.info("supervisor exiting: stopping owned controller")
            self.stop()

    def _terminate_group(self, pid: int) -> None:
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

    def _kill_group(self, pid: int) -> None:
        try:
            os.killpg(os.getpgid(pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

    def _terminate_pid(self, pid: int) -> None:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


class FlowBaseSupervisorClient:
    """Thin portal client for the supervisor RPC surface.

    Symmetric with FlowBaseClient: all supervisor wire detail (host, port,
    method names) lives here so callers (e.g. the follower node) stay agnostic.
    """

    def __init__(self, host: str = "localhost", port: int = SUPERVISOR_DEFAULT_PORT):
        self.client = portal.Client(f"{host}:{port}")

    def start(self) -> Any:
        """Ask the supervisor to spawn the controller (idempotent)."""
        return self.client.start({}).result()

    def stop(self) -> Any:
        """Ask the supervisor to SIGTERM the controller (engages the brake)."""
        return self.client.stop({}).result()

    def status(self) -> Any:
        """Query controller liveness: {"running": bool, "pid": int|None}."""
        return self.client.status({}).result()

    def close(self) -> None:
        """Tear down the local portal connection (does not touch the controller)."""
        self.client.close()


def _run_self_test(host: str, port: int, command: str) -> None:
    """Act as our own client for bring-up: print one RPC's result and exit."""
    client = FlowBaseSupervisorClient(host=host, port=port)
    try:
        if command == "start":
            print(client.start())
        elif command == "stop":
            print(client.stop())
        else:
            print(client.status())
    finally:
        client.close()


def main() -> None:
    """Run the supervisor RPC server, or a one-shot self-test client."""
    parser = argparse.ArgumentParser(description="FlowBase controller supervisor daemon")
    parser.add_argument("--venv", type=str, default=_DEFAULT_VENV,
                        help="Virtualenv whose bin/activate is sourced for the controller")
    parser.add_argument("--root", type=str, default=_DEFAULT_ROOT,
                        help="Repo root on PYTHONPATH; locates flow_base_controller.py")
    parser.add_argument("--channel", type=str, default=_DEFAULT_CHANNEL,
                        help="CAN channel passed to the controller")
    parser.add_argument("--no-linear-rail", action="store_true",
                        help="Launch the controller with --no-linear-rail (base only)")
    parser.add_argument("--port", type=int, default=SUPERVISOR_DEFAULT_PORT,
                        help="Supervisor RPC port")
    parser.add_argument("--host", type=str, default="localhost",
                        help="Host to reach the supervisor (self-test mode only)")
    parser.add_argument("--self-test", choices=["start", "stop", "status"], default=None,
                        help="Act as a client against a running supervisor and exit")
    args = parser.parse_args()

    if args.self_test is not None:
        _run_self_test(args.host, args.port, args.self_test)
        return

    command = build_controller_command(
        args.venv, args.root, args.channel, no_linear_rail=args.no_linear_rail
    )
    controller = ControllerProcess(command)
    atexit.register(controller.stop_if_owned)

    server = portal.Server(args.port)
    server.bind("start", lambda input_dict: controller.start())
    server.bind("stop", lambda input_dict: controller.stop())
    server.bind("status", lambda input_dict: controller.status())

    logger.info(
        "FlowBase supervisor listening on port %d (controller port %d)",
        args.port,
        CONTROLLER_DEFAULT_PORT,
    )
    server.start(block=True)


if __name__ == "__main__":
    main()
