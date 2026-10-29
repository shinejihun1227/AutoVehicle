"""Scope and stamped speed-only contract for the existing main controller."""
import hashlib
import json
import math
from pathlib import Path
import yaml
from .path import load_mgeo_path


def text_sha(path):
    return hashlib.sha256(Path(path).read_bytes().replace(b'\r\n', b'\n')).hexdigest()


class RoundaboutScope:
    def __init__(self, config_file, path_file, circulating_file):
        self.config = yaml.safe_load(Path(config_file).read_text(encoding='utf-8'))
        self.path = load_mgeo_path(path_file)
        self.circulating = load_mgeo_path(circulating_file)
        geometry = self.config['geometry']
        self.path_sha = text_sha(path_file)
        if (self.path_sha != geometry['main_path_sha256']
                or text_sha(circulating_file) != geometry['circulating_path_sha256']):
            raise ValueError('Roundabout path changed: regenerate the MGeo mission configuration')
        s = [0.]
        for a, b in zip(self.path, self.path[1:]):
            s.append(s[-1]+math.hypot(b.x-a.x, b.y-a.y))
        ids = self.config['mission_indices']
        self.start, self.end = s[ids['approach']], s[ids['exit']]
        self.timeout = float(self.config['decision_timeout_s'])
        self.maximum = float(self.config['test_speed_limit'])
        if not 0 <= self.start < self.end or not 0 < self.timeout <= 1.:
            raise ValueError('Invalid roundabout scope or timeout')

    def contains(self, s):
        return math.isfinite(s) and self.start <= s < self.end


class MissionSpeedGate:
    def __init__(self, scope):
        self.scope, self.sample = scope, None

    def observe(self, text, ros_now, wall_now):
        try:
            state = json.loads(text, parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
            stamp, target = float(state['timestamp']), float(state['target_speed'])
            if (state['main_path_sha256'] != self.scope.path_sha
                    or type(state['active']) is not bool or type(state['stop_required']) is not bool
                    or type(state['emergency']) is not bool
                    or not isinstance(state.get('decision_reason'), str)
                    or not all(math.isfinite(v) for v in (stamp, target, ros_now, wall_now))
                    or not 0 <= target <= self.scope.maximum + 1e-6
                    or not -0.05 <= ros_now-stamp <= self.scope.timeout
                    or (self.sample and stamp <= self.sample[0])):
                raise ValueError('Invalid mission status')
            self.sample = stamp, wall_now, state
        except (ValueError, TypeError, KeyError, OverflowError):
            self.sample = None

    def limit(self, target, progress, ros_now, wall_now):
        if not self.scope.contains(progress):
            return target, False, 'outside_roundabout'
        sample = self.sample
        if (sample is None or not -0.05 <= ros_now-sample[0] <= self.scope.timeout
                or not 0 <= wall_now-sample[1] <= self.scope.timeout):
            return 0., True, 'roundabout_decision_stale'
        state = sample[2]
        if not state['active']:
            return 0., True, 'roundabout_scope_mismatch'
        if state['stop_required'] or state['emergency']:
            return 0., True, state['decision_reason']
        return min(target, state['target_speed']), False, state['decision_reason']
