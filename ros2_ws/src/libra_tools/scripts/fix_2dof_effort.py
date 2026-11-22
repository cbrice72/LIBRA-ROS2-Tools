#!/usr/bin/env python3

# Brief:
#   Doubles the Roll and Pitch effort in LIBRA-I rosbags recorded between 2025-08-21 and 2026-11-20.
# 
#   During this period, LIBRA App used `ActuatorToJointDifferential()`, a
#   translation function for joint positions, to also calculate joint torque.
#   This incorrectly averaged the 2-DoF joint's actuator torques instead of
#   adding them. Fixed in LIBRA App commit `da611a0``.
#
#   Only the `/joint_states` and `/joint_targets`` of each rosbag are rewritten,
#   so no expensive copies are necessary. Before any change, the original rows
#   are saved to an "undo" file next to the bag, which also marks the bag as
#   fixed so it is never doubled twice.
#
# Usage:
#   python3 fix_2dof_effort.py BAG_DIR [BAG_DIR ...]            # dry run
#   python3 fix_2dof_effort.py BAG_DIR [BAG_DIR ...] --apply    # fix bags
#   python3 fix_2dof_effort.py BAG_DIR [BAG_DIR ...] --restore  # revert
#
# Assumptions:
#   - ROS2 Humble is sourced.
#   - Bags use sqlite3 w/ CDR serialization.
#   - Every message on the above topics is a `sensor_msgs/msg/JointState``.

import argparse
import datetime
import os
import sqlite3
import sys

import yaml
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState

TOPICS = ('/joint_states', '/joint_targets')
JOINTS = ('Roll', 'Pitch')
SCALE = 2.0  # accidentally averaged joints must be added

# Recording dates affected by the averaging bug (local time, JST)
JST = datetime.timezone(datetime.timedelta(hours=9))
BUG_START = datetime.datetime(2025, 8, 21, tzinfo=JST)
BUG_END = datetime.datetime(2026, 11, 20, 10, 31, tzinfo=JST)

UNDO_SUFFIX = '_pre-effort-fix.sqlite'  # original rows, before the fix


def read_metadata(bag_dir):
    """
    Returns the bag's start time and the paths of its SQLite files.
    """
    with open(os.path.join(bag_dir, 'metadata.yaml')) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']

    if info['storage_identifier'] != 'sqlite3':
        raise ValueError('storage is ' + info['storage_identifier'] + ', not sqlite3')

    start_ns = info['starting_time']['nanoseconds_since_epoch']
    start = datetime.datetime.fromtimestamp(start_ns / 1e9, JST)
    db_paths = [os.path.join(bag_dir, p) for p in info['relative_file_paths']]

    return start, db_paths


def joint_rows(db):
    """
    Gives "(row id, topic name, serialized message)" for relevant topics.
    """
    topic_ids = {}
    for topic_id, name, msg_type in db.execute('SELECT id, name, type FROM topics'):
        if name in TOPICS:
            if msg_type != 'sensor_msgs/msg/JointState':
                raise ValueError(name + ' has type ' + msg_type)
            topic_ids[topic_id] = name

    if not topic_ids:
        return

    marks = ','.join('?' * len(topic_ids))
    query = 'SELECT id, topic_id, data FROM messages WHERE topic_id IN (' + marks + ')'
    for row_id, topic_id, data in db.execute(query, list(topic_ids)):
        yield row_id, topic_ids[topic_id], data


def scale_effort(data):
    """
    Returns the topic message with its 2-DoF efforts doubled (or None).
    """
    msg = deserialize_message(data, JointState)
    indices = [i for i, name in enumerate(msg.name)
               if name in JOINTS and i < len(msg.effort)]
    if not indices:
        return None

    effort = list(msg.effort)
    for i in indices:
        effort[i] *= SCALE

    msg.effort = effort

    return serialize_message(msg)


def max_effort(data):
    """
    Returns the largest 2-DoF effort magnitude in a message (Nm).
    """
    msg = deserialize_message(data, JointState)
    values = [abs(msg.effort[i]) for i, name in enumerate(msg.name)
              if name in JOINTS and i < len(msg.effort) and msg.effort[i] == msg.effort[i]]
    
    return max(values, default=0.0)


def fix_bag(bag_dir, undo_path, apply):
    """
    Doubles one bag's 2-DoF efforts, saving the original rows first.
    """
    start, db_paths = read_metadata(bag_dir)
    print('  Starts ' + start.strftime('%Y-%m-%d %H:%M') + ' JST')
    if not BUG_START <= start < BUG_END:
        print('  Skipped: recorded outside the affected dates')
        return

    if os.path.exists(undo_path):
        print('  Skipped: already fixed (undo file exists)')
        return

    # Compute fixed rows of every SQLite file
    updates = {}  # db path -> list of (row id, original data, fixed data)
    counts = {topic: 0 for topic in TOPICS}
    peak_before = 0.0  # largest 2-DoF torque before the fix (Nm)
    for db_path in db_paths:
        with sqlite3.connect(db_path) as db:
            rows = []
            for row_id, topic, data in joint_rows(db):
                fixed = scale_effort(data)
                if fixed is None:
                    continue
                rows.append((row_id, data, fixed))
                counts[topic] += 1
                peak_before = max(peak_before, max_effort(data))
            updates[db_path] = rows

    total = sum(counts.values())
    for topic in TOPICS:
        print('  ' + topic + ' messages w/ Roll or Pitch effort: ' + str(counts[topic]))

    if total == 0:
        print('  Skipped: no messages w/ Roll or Pitch effort (not a LIBRA-I bag)')
        return
    
    print('  Largest Roll or Pitch effort is ' + format(peak_before, '.2f')
          + ' Nm now, and will be ' + format(SCALE * peak_before, '.2f') + ' Nm')

    if not apply:
        print('  Dry run, so nothing was changed')
        return

    # Save original rows before touching rosbag
    # (NOTE: a partial undo file would mark the bag as fixed, so delete it if
    #  saving fails)
    try:
        with sqlite3.connect(undo_path) as undo:
            undo.execute('CREATE TABLE original_messages (db_file TEXT, id INTEGER, data BLOB)')
            for db_path, rows in updates.items():
                undo.executemany('INSERT INTO original_messages VALUES (?, ?, ?)',
                                 [(os.path.basename(db_path), row_id, data)
                                  for row_id, data, _ in rows])
    except sqlite3.Error:
        os.remove(undo_path)
        raise

    # Rewrite rows (each SQLite file in one go)
    for db_path, rows in updates.items():
        with sqlite3.connect(db_path) as db:
            db.executemany('UPDATE messages SET data = ? WHERE id = ?',
                           [(fixed, row_id) for row_id, _, fixed in rows])

    print('  Fixed ' + str(total) + ' messages. Original rows saved to ' + undo_path)


def restore_bag(bag_dir, undo_path):
    """
    Writes the original rows from the undo file back into the bag.
    """
    if not os.path.exists(undo_path):
        print('  Skipped: no undo file (bag was never fixed)')
        return

    with sqlite3.connect(undo_path) as undo:
        rows = undo.execute('SELECT db_file, id, data FROM original_messages').fetchall()

    by_file = {}
    for db_file, row_id, data in rows:
        by_file.setdefault(db_file, []).append((data, row_id))

    for db_file, file_rows in by_file.items():
        with sqlite3.connect(os.path.join(bag_dir, db_file)) as db:
            db.executemany('UPDATE messages SET data = ? WHERE id = ?', file_rows)

    os.remove(undo_path)
    
    print('  Restored ' + str(len(rows)) + ' messages and removed the undo file')


def main():
    parser = argparse.ArgumentParser(description='Double the 2-DoF joint effort in LIBRA-I rosbags.')
    parser.add_argument('bag_dirs', nargs='+', help='rosbag2 directories')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--apply', action='store_true', help='fix the bags (default is a dry run)')
    mode.add_argument('--restore', action='store_true', help='undo a previous fix')
    args = parser.parse_args()

    failed = False
    for bag_dir in args.bag_dirs:
        bag_dir = os.path.normpath(bag_dir)
        undo_path = bag_dir + UNDO_SUFFIX
        print(bag_dir)

        try:
            if args.restore:
                restore_bag(bag_dir, undo_path)
            else:
                fix_bag(bag_dir, undo_path, args.apply)
        except (OSError, ValueError, KeyError, sqlite3.Error) as e:
            print('  FAILED: ' + str(e))
            failed = True

    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
