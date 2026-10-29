#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Require real loop completion and a latched, non-advancing public pose stream."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path)
    args = parser.parse_args()
    case = args.case
    rows = [json.loads(line) for line in (case / 'vio.jsonl').read_text().splitlines()]
    runtime = case/('orb-runtime' if (case/'orb-runtime').is_dir() else 'runtime')
    atlas = json.loads((runtime/'atlas-final.json').read_text())
    result = json.loads((case / 'result.json').read_text())
    events = [r for r in rows if r['stream'] == 'atlas_event']
    # This audit targets actual new-map loop closing, not initial loaded-map
    # relocalization or fixture-generated status/position faults.
    # Initialization may discard an uninitialized empty map before any public
    # pose exists. The navigation session begins at the first admitted pose.
    corrections = [r for r in events if int(r['values']['correction_epoch']) > 0 and
                   int(r['values']['public_pose_count']) > 0]
    poses = [r for r in rows if r['stream'] == 'local_body_pose']
    statuses = [r for r in rows if r['stream'] == 'status']
    first = corrections[0] if corrections else None
    checks = dict(
        replay_and_native_pose_evaluation_passed=result['passed'] and
            json.loads((case / 'evaluation.json').read_text())['passed'],
        actual_completed_loop=atlas['loops'] > 0 and any(
            int(r['values']['loop_closures']) > 0 for r in events),
        no_relocalization_substituted_for_loop=atlas['relocalizations'] == 0,
        actual_epoch_event_observed=first is not None,
        normal_worker_join=atlas['workers_joined'] and not result['forced'],
    )
    if first:
        values = first['values']
        resets = [r for r in statuses if r['state'] == 4]
        checks.update(
            reset_latched_before_event=int(values['reset_latched']) == 1,
            # Distinct DDS topics can deliver an already-published old pose
            # after the epoch event. Use its original source stamp and the
            # publisher's frozen counter, not cross-topic receipt ordering.
            no_public_pose_after_correction=all(
                p['stamp_ns'] <= int(values['last_public_pose_stamp_ns']) for p in poses),
            public_counter_matches_observer=len(poses) == int(values['public_pose_count']),
            no_automatic_realignment=all(int(r['values']['public_pose_count']) ==
                                        int(values['public_pose_count']) for r in corrections),
            reset_status_maintained=bool(resets) and all(
                r['state'] == 4 for r in statuses if r['receipt'] >= resets[0]['receipt']),
            original_sensor_stream_continued_after_correction=any(
                json.loads(line)['receipt'] > first['receipt'] + 1.0 for line in
                (case / 'input-fingerprints.jsonl').read_text().splitlines()
                if not json.loads(line)['kind'].startswith('forwarded_')),
        )
    report = dict(passed=all(checks.values()), checks=checks, atlas=atlas,
                  first_correction_event=first,
                  method='actual upstream CorrectLoop completion + private epoch telemetry + public receipts',
                  hardware=False, auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (case / 'correction-audit.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
