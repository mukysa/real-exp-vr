"""Deterministic base-frame translation and tool-local rotation scenarios."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


Q0 = (0.0, -0.4, 0.0, -1.8, 0.0, 1.5, 0.7)
NORMAL = ('static', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'circle', 'combined')
FAULTS = ('unreachable', 'jump', 'invalid', 'interruption', 'reconnect', 'manual_hold')


@dataclass
class Sample:
    time: float
    pose: np.ndarray
    event: str = 'follow'
    connection: int = 1


def smooth(u):
    return 10*u**3 - 15*u**4 + 6*u**5


def scenarios(anchor, hz=50, duration=4):
    count = round(hz * duration)
    for name in NORMAL:
        samples = []
        for i in range(count + 1):
            u = i / count
            phase = smooth(2*u) if u <= .5 else smooth(2*(1-u))
            pose = anchor.copy()
            if name in ('x', 'y', 'z'):
                pose['xyz'.index(name), 3] += .02 * phase
            if name in ('roll', 'pitch', 'yaw', 'combined'):
                axis = ('roll', 'pitch', 'yaw').index(name) if name != 'combined' else 1
                rotvec = np.zeros(3)
                rotvec[axis] = np.deg2rad(5) * phase
                pose[:3, :3] = anchor[:3, :3] @ Rotation.from_rotvec(rotvec).as_matrix()
            if name in ('circle', 'combined'):
                theta = 2*np.pi*smooth(u)
                pose[:2, 3] += [.01*(np.cos(theta)-1), .01*np.sin(theta)]
            samples.append(Sample((i+1)/hz, pose))
        yield name, samples
    for name in FAULTS:
        samples = []
        for i in range(61):
            pose = anchor.copy()
            event = 'follow'
            connection = 1
            t = (i+1)/hz
            if i == 20:
                if name == 'unreachable':
                    pose[0, 3] = 2
                elif name == 'jump':
                    pose[0, 3] += .15
                elif name == 'invalid':
                    event = 'invalid'
            if name == 'interruption' and i >= 20:
                t += .4
            if name == 'reconnect' and i >= 20:
                connection = 2
            if name == 'manual_hold' and 20 <= i < 40:
                event = 'hold'
            if name in ('interruption', 'reconnect', 'manual_hold') and i == 40:
                event = 'reset'
            if name in ('interruption', 'reconnect', 'manual_hold') and i > 40:
                pose[2, 3] += .002*smooth((i-40)/20)
            samples.append(Sample(t, pose, event, connection))
        yield name, samples
