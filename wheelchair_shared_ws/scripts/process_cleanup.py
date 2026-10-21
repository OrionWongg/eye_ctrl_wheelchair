"""Terminate an owned start_new_session group, including orphaned children."""
import os
import signal
import subprocess
import time

def stop_group(process,initial_signal=True):
    if process is None:return
    for sig,timeout in ((signal.SIGINT,8.),(signal.SIGTERM,3.),(signal.SIGKILL,2.)):
        try:
            if sig==signal.SIGINT:
                if initial_signal:
                    if process.poll() is None:process.send_signal(sig)
                    else:os.killpg(process.pid,sig)
            else:os.killpg(process.pid,sig)
        except ProcessLookupError:
            process.poll();return
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            process.poll()
            try:os.killpg(process.pid,0)
            except ProcessLookupError:return
            time.sleep(.05)
    try:process.wait(timeout=1)
    except subprocess.TimeoutExpired:pass
