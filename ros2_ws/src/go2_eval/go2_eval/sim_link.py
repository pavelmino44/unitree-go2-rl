"""Control connection to the MuJoCo evaluation simulator (unitree_mujoco/simulate_python/eval_sim.py).

The simulator runs in its own process (and its own Python, if needed: it uses mujoco and unitree_sdk2py, whose
CycloneDDS must not be loaded into the same process as rclpy). It connects back to a TCP port of this process
and exchanges JSON lines (see eval_sim.py).
"""

from __future__ import annotations

import json
import os
import queue
import socket
import subprocess
import threading
import time


class SimError(RuntimeError):
    pass


class SimProcess:
    def __init__(
        self,
        sim_dir: str,
        python: str,
        catalog: str,
        scenario: str,
        seed: int,
        domain: int,
        interface: str,
        sample_dt: float,
        markers: list,
        viewer: bool,
        log_path: str,
        extra_args: list[str] | None = None,
    ):
        self.sim_dir = sim_dir
        self.cmd_base = [
            python, "-u", os.path.join(sim_dir, "eval_sim.py"),
            "--catalog", catalog, "--scenario", scenario, "--seed", str(seed),
            "--domain", str(domain), "--interface", interface,
            "--sample_dt", repr(sample_dt), "--markers", json.dumps(markers),
        ]  # fmt: skip
        if viewer:
            self.cmd_base.append("--viewer")
        self.cmd_base += list(extra_args or [])
        self.log_path = log_path
        self.proc: subprocess.Popen | None = None
        self.conn: socket.socket | None = None
        self.samples: queue.Queue = queue.Queue()
        self.acks: queue.Queue = queue.Queue()
        self.hello: dict | None = None
        self.closed = False

    # ---- life cycle ------------------------------------------------------------------------------------

    def start(self, timeout: float = 90.0) -> dict:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        server.settimeout(0.5)
        port = server.getsockname()[1]
        cmd = self.cmd_base + ["--host", "127.0.0.1", "--port", str(port)]
        self._log = open(self.log_path, "w")
        self._log.write("$ " + " ".join(cmd) + "\n\n")
        self._log.flush()
        self.proc = subprocess.Popen(cmd, cwd=self.sim_dir, stdout=self._log, stderr=subprocess.STDOUT)
        deadline = time.time() + timeout
        try:
            while True:
                if self.proc.poll() is not None:
                    raise SimError(f"the simulator exited (code {self.proc.returncode}) before connecting")
                if time.time() > deadline:
                    raise SimError(f"the simulator did not connect within {timeout:.0f} s")
                try:
                    self.conn, _ = server.accept()
                    break
                except socket.timeout:
                    continue
        finally:
            server.close()
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=self._reader, daemon=True).start()
        try:
            msg = self.samples.get(timeout=30.0)
        except queue.Empty:
            raise SimError("no hello from the simulator") from None
        if msg.get("type") != "hello":
            raise SimError(f"unexpected first message: {msg}")
        self.hello = msg
        return msg

    def _reader(self):
        buf = b""
        while True:
            try:
                chunk = self.conn.recv(1 << 16)
            except OSError:
                chunk = b""
            if not chunk:
                self.closed = True
                self.samples.put({"type": "closed"})
                self.acks.put({"type": "closed"})
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                msg = json.loads(line)
                (self.acks if msg.get("type") == "ack" else self.samples).put(msg)

    def close(self, timeout: float = 5.0):
        try:
            if self.conn is not None and not self.closed:
                self.conn.sendall(b'{"cmd": "quit"}\n')
        except OSError:
            pass
        if self.proc is not None:
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        if self.conn is not None:
            self.conn.close()
        if getattr(self, "_log", None):
            self._log.close()

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None and not self.closed

    def log_tail(self, lines: int = 20) -> str:
        try:
            with open(self.log_path, errors="replace") as f:
                return "".join(f.readlines()[-lines:])
        except OSError:
            return ""

    # ---- messages ----------------------------------------------------------------------------------------

    def command(self, msg: dict, timeout: float = 5.0) -> dict:
        """Send a command and wait for its acknowledgement."""
        if not self.alive():
            raise SimError("the simulator is not running")
        while not self.acks.empty():
            self.acks.get_nowait()
        self.conn.sendall((json.dumps(msg) + "\n").encode())
        try:
            ack = self.acks.get(timeout=timeout)
        except queue.Empty:
            raise SimError(f"no answer to '{msg.get('cmd')}'") from None
        if ack.get("type") == "closed":
            raise SimError("the simulator closed the connection")
        return ack

    def drain(self):
        while not self.samples.empty():
            self.samples.get_nowait()

    def sample(self, timeout: float = 2.0) -> dict:
        try:
            msg = self.samples.get(timeout=timeout)
        except queue.Empty:
            raise SimError(f"no data from the simulator for {timeout:.1f} s") from None
        if msg.get("type") == "closed":
            raise SimError("the simulator closed the connection (window closed?)")
        return msg
