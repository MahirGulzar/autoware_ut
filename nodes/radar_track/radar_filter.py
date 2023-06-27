#!/usr/bin/env python2

from math import sqrt

def is_consistent(prepared_object, id_count):
    if prepared_object['id'] not in id_count.keys():
        id_count[prepared_object['id']] = 1
        return False

    id_count[prepared_object['id']] += 1
    return id_count[prepared_object['id']] > 5


def is_within_range(prepared_object, tf_listener, map_frame, radar_frame, max_distance):
    position_on_map = tf_listener.lookupTransform(map_frame, radar_frame, prepared_object['timestamp'])
    distance = sqrt((position_on_map[0][0] - prepared_object['position'].x) ** 2 + (
                position_on_map[0][1] - prepared_object['position'].y) ** 2)
    return 1 < distance < max_distance


def remove_old_tracks(id_list, id_count):
    lost_tracks = id_count.viewkeys() - id_list
    remove_entries_from_dict(list(lost_tracks), id_count)


def remove_entries_from_dict(entries, the_dict):
    for key in entries:
        if key in the_dict:
            del the_dict[key]
