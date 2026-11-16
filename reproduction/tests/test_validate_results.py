"""Consumer-visible same-round and map-freshness acceptance regressions."""
import copy
import importlib.util
from pathlib import Path
import unittest

VALIDATOR = Path(__file__).resolve().parents[1] / 'docker' / 'validate_results.py'
spec = importlib.util.spec_from_file_location('validate_results', VALIDATOR)
validator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validator)
BOUNDS = {r: (0.0, 100.0) for r in 'abc'}
SEPARATORS = [(ord('a') << 56) | 1, (ord('b') << 56) | 1,
              (ord('b') << 56) | 2, (ord('c') << 56) | 1]


def snapshot():
    statuses = {}
    for index, robot in enumerate('abc'):
        statuses[robot] = dict(round_id=100, coordinator=0, participants=[0, 1, 2],
                               order=[0, 1, 2], state=7, committed=True, aborted=False,
                               completed_state=6, completed_observed_at=110.0 + index,
                               separator_keys=list(SEPARATORS), proposal_keys=list(SEPARATORS),
                               rotation_done=True, pose_done=True,
                               completed_rotation_done=True, completed_pose_done=True,
                               observed_at=115.0)
    return dict(nonfinite_odometry={}, optimizer_states={r: [0, 6, 7] for r in 'abc'},
                accepted_inter_robot_loops=[[0, 1, 1, 1], [1, 1, 2, 1]],
                optimization_rounds={'100': dict(robots=statuses, aborted=False)},
                robots={r: dict(odometry_samples=100, map_points=20, first_stamp=0.0,
                                last_stamp=100.0, map_received_at=113.0,
                                map_header_stamp=90.0) for r in 'abc'})


class RoundAcceptanceTests(unittest.TestCase):
    def assert_rejected(self, data):
        with self.assertRaises(RuntimeError):
            validator.validate_snapshot(data, BOUNDS)

    def test_coherent_round_with_old_cloud_headers_succeeds(self):
        data = snapshot()
        self.assertIs(validator.validate_snapshot(data, BOUNDS), data)

    def test_same_round_idle_preserves_observed_end(self):
        data = snapshot()
        for status in data['optimization_rounds']['100']['robots'].values():
            status['state'] = 0
        validator.validate_snapshot(data, BOUNDS)

    def test_stale_ab_completion_and_new_c_cannot_combine(self):
        data = snapshot()
        statuses = data['optimization_rounds']['100']['robots']
        c = statuses.pop('c')
        for status in statuses.values():
            status['participants'] = [0, 1]
        c['round_id'] = 200
        data['optimization_rounds']['200'] = dict(robots={'c': c}, aborted=False)
        self.assert_rejected(data)

    def test_newer_pending_or_aborted_round_blocks_old_success(self):
        for aborted in (False, True):
            with self.subTest(aborted=aborted):
                data = snapshot()
                newer = copy.deepcopy(data['optimization_rounds']['100'])
                newer['aborted'] = aborted
                for status in newer['robots'].values():
                    status.update(round_id=200, completed_state=None,
                                  completed_observed_at=None, state=3, aborted=aborted)
                data['optimization_rounds']['200'] = newer
                self.assert_rejected(data)

    def test_abort_metadata_and_commit_rejected(self):
        changes = [dict(aborted=True), dict(committed=False),
                   dict(participants=[0, 1]), dict(participants=[0, 1, 1, 2]),
                   dict(order=[1, 0, 2]), dict(order=[0, 0, 2]),
                   dict(coordinator=1), dict(metadata_mismatch=True),
                   dict(completed_state=None), dict(round_id=99)]
        for change in changes:
            with self.subTest(change=change):
                data = snapshot()
                data['optimization_rounds']['100']['robots']['b'].update(change)
                self.assert_rejected(data)

    def test_postend_without_actual_end_rejected(self):
        data = snapshot()
        data['optimization_rounds']['100']['robots']['c'].update(
            completed_state=None, completed_observed_at=None)
        self.assert_rejected(data)
    def test_committed_pcm_graph_must_match_and_connect_all_robots(self):
        data = snapshot()
        data['optimization_rounds']['100']['robots']['b']['separator_keys'] = SEPARATORS[:2]
        self.assert_rejected(data)
        for separators in ([], SEPARATORS[:2], SEPARATORS[:3],
                           [SEPARATORS[0], (ord('d') << 56) | 1]):
            with self.subTest(separators=separators):
                data = snapshot()
                for status in data['optimization_rounds']['100']['robots'].values():
                    status['separator_keys'] = separators
                self.assert_rejected(data)

    def test_actual_end_requires_both_stage_completions(self):
        for field in ('completed_rotation_done', 'completed_pose_done'):
            with self.subTest(field=field):
                data = snapshot()
                data['optimization_rounds']['100']['robots']['c'][field] = False
                self.assert_rejected(data)


    def test_every_map_must_arrive_strictly_after_joint_completion(self):
        for robot in 'abc':
            for timestamp in (None, 111.0, 112.0, float('nan')):
                with self.subTest(robot=robot, timestamp=timestamp):
                    data = snapshot()
                    data['robots'][robot]['map_received_at'] = timestamp
                    data['robots'][robot]['map_header_stamp'] = 999.0
                    self.assert_rejected(data)

    def test_legacy_numeric_completion_alone_rejected(self):
        data = snapshot()
        del data['optimization_rounds']
        self.assert_rejected(data)

    def test_existing_quality_gates_remain(self):
        for kind in ('coverage', 'odometry', 'map', 'graph', 'finite'):
            with self.subTest(kind=kind):
                data = snapshot()
                if kind == 'coverage':
                    data['robots']['c']['last_stamp'] = 90.0
                elif kind == 'odometry':
                    data['robots']['c']['odometry_samples'] = 1
                elif kind == 'map':
                    data['robots']['c']['map_points'] = 0
                elif kind == 'graph':
                    data['accepted_inter_robot_loops'] = [[0, 1, 1, 1]]
                else:
                    data['nonfinite_odometry'] = {'a': 1}
                self.assert_rejected(data)


if __name__ == '__main__':
    unittest.main()
