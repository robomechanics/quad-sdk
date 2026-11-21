"""Replay frozen experiment observations/targets and prove force invariance.

Run with the policy Python, passing the packaged Conga runtime root as argv[1].
"""
from pathlib import Path
import sys
import numpy as np
import torch

def main():
    root = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'robot_driver/conga'
    import json, hashlib
    manifest = json.loads((root/'manifest.json').read_text())
    for name, record in manifest['files'].items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == record['bundled_sha256'], name
    sys.path.insert(0, str(root))
    from deploy.cpg.runtime import SensorFrame, load_selection
    from conga_runtime import ContactFreeRuntime

    torch.set_num_threads(1)
    selection = load_selection(root/'assets/selection.json')
    trace = np.load(root/'tests/replay.npz')
    runtimes = [ContactFreeRuntime(selection['deploy_metadata'], selection['onnx_export']) for _ in range(2)]
    rng = np.random.default_rng(42)
    max_obs = max_target = 0.
    with torch.inference_mode():
        for i in range(600):
            results = []
            for runtime, force in zip(runtimes, (np.zeros(4), rng.uniform(0, 1000, 4))):
                frame = SensorFrame(float(trace['sensor_time'][i,0]), trace['sensor_q'][i,0],
                    trace['sensor_qd'][i,0], trace['sensor_quaternion'][i,0],
                    trace['angular_velocity'][i,0], force)
                obs = runtime.observe(frame, trace['operator_velocity'][i,0])
                target = runtime.act(obs)
                results.append((obs, target))
            np.testing.assert_array_equal(results[0][0], results[1][0])
            np.testing.assert_array_equal(results[0][1], results[1][1])
            max_obs = max(max_obs, float(np.max(np.abs(results[0][0]-trace['observation'][i]))))
            max_target = max(max_target, float(np.max(np.abs(results[0][1]-trace['target_sdk'][i,0]))))
            np.testing.assert_allclose(results[0][0], trace['observation'][i], atol=1e-6, rtol=1e-6)
            np.testing.assert_allclose(results[0][1], trace['target_sdk'][i,0], atol=1e-6, rtol=1e-6)
    print(f'600 frozen frames: max observation error={max_obs}, target error={max_target}; force invariant')


if __name__ == "__main__":
    main()
