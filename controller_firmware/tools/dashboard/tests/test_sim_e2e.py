"""End to end: start foc_sim, talk to it exactly like the dashboard does."""
import os
import socket
import struct
import subprocess
import time

import pytest

import plot_motor as pm

SIM = os.environ.get("FOC_SIM") or pm.find_sim()
pytestmark = pytest.mark.skipif(not SIM, reason="foc_sim not built")


@pytest.fixture
def sim():
    """Start foc_sim on a free port and connect to it."""
    port = 5700 + os.getpid() % 200
    proc = subprocess.Popen([SIM, "--port", str(port), "--fast"], stdout=subprocess.PIPE)
    sock = None
    for _ in range(50):
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=1)
            break
        except OSError:
            time.sleep(0.1)
    assert sock, "simulator did not start"
    sock.settimeout(0.05)
    yield sock
    sock.close()
    proc.terminate()
    proc.wait(timeout=5)


class Link:
    """Minimal dashboard-like client: sends commands, decodes telemetry."""

    def __init__(self, sock):
        self.sock, self.buf, self.state, self.logs = sock, b"", {}, []

    def send(self, line):
        """Send one command line."""
        self.sock.sendall(line.encode() + b"\n")

    def pump(self, seconds):
        """Read and decode telemetry for `seconds`."""
        end = time.time() + seconds
        while time.time() < end:
            try:
                self.buf += self.sock.recv(65536)
            except socket.timeout:
                continue
            frames, self.buf = pm._extract_frames(self.buf)
            for t, p in frames:
                d = pm._dispatch(t, p) or {}
                if "__log__" in d:
                    self.logs.append(d.pop("__log__"))
                self.state.update(d)

    def wait_for(self, pred, timeout):
        """Pump until pred(state) is true or the timeout passes."""
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.1)
            if pred(self.state):
                return True
        return False


def test_calibrate_and_spin(sim):
    """Calibrate, then spin to 3000 rpm."""
    link = Link(sim)
    assert link.wait_for(lambda s: s.get("state") == 1, 5), "never reached IDLE"
    link.send("calibrate")
    assert link.wait_for(lambda s: s.get("state") == 2, 5)
    assert link.wait_for(lambda s: s.get("state") == 1, 30), "calibration did not finish"
    link.send("motor 3000")
    assert link.wait_for(lambda s: abs(s.get("speed_rpm", 0) - 3000) < 50, 30)
    assert link.state["fault"] == 0


def test_sim_and_status_commands(sim):
    """Simulator and firmware commands both reply."""
    link = Link(sim)
    link.wait_for(lambda s: s.get("state") == 1, 5)
    link.send("sim load 0.02")
    link.send("status")
    link.pump(0.5)
    assert any(l.startswith("sim: load 0.020") for l in link.logs)
    assert any(l.startswith("state=IDLE") for l in link.logs)


def test_refused_until_calibrated(sim):
    """Running before calibration is refused, not faulted."""
    link = Link(sim)
    link.wait_for(lambda s: s.get("state") == 1, 5)
    link.send("motor 1000")                     # not calibrated yet
    link.pump(0.5)
    assert link.state["state"] == 1
    assert any("not calibrated" in l for l in link.logs)


def test_fault_reported(sim):
    """A fault shows up in the telemetry with its name."""
    link = Link(sim)
    link.wait_for(lambda s: s.get("state") == 1, 5)
    link.send("sim temp 120")
    assert link.wait_for(lambda s: s.get("state") == 4, 5)
    assert "overtemp" in pm.fault_names(link.state["__faults__"])
