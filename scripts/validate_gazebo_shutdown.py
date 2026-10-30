"""Run inside devbox: regression checks for continuous mode and interrupt saving.

Uses an isolated ROS domain/master and /tmp outputs; preserves report results.
"""
import json, os, signal, subprocess, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
for sig in [signal.SIGINT, signal.SIGTERM]:
    out = Path('/tmp')/('ekf-shutdown-'+sig.name)
    out.mkdir(exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID='77', GAZEBO_MASTER_URI='http://127.0.0.1:11358', SLAM_OUTPUT_DIR=str(out))
    log = out/'console.log'
    with log.open('w') as f:
        proc = subprocess.Popen(['bash', str(ROOT/'gazebo/run_demo.sh'), 'headless', '0'],
                                cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+25
            while 't=2.1s' not in log.read_text():
                if proc.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Demo failed to start: '+log.read_text())
                time.sleep(.2)
            assert proc.poll() is None, 'Continuous mode exited on its own'
            children = Path(f'/proc/{proc.pid}/task/{proc.pid}/children').read_text().split()
            node_pid = next(int(pid) for pid in children if 'slam_node.py' in Path(f'/proc/{pid}/cmdline').read_text())
            os.kill(node_pid, sig)
            assert proc.wait(timeout=15)==0, log.read_text()
            text = log.read_text()
            assert 'Traceback' not in text and 'RCLError' not in text, text
            metrics = json.loads((out/'metrics.json').read_text())
            assert metrics['samples']>0 and metrics['landmarks']>0
            print(f'{sig.name}: clean exit, saved {metrics["samples"]} samples; continuous mode verified.', flush=True)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
