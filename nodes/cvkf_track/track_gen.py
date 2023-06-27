#!/usr/bin/env python2

import numpy as np
import copy
import scipy.stats.distributions as dists
from scipy.spatial import distance
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.optimize import linear_sum_assignment


class NNDATracker():

    def __init__(self, min_life_threshold, min_detection_thresh, max_misdetection_thresh, gating_treshold):

        self.min_life_threshold = min_life_threshold
        self.min_detection_thresh = min_detection_thresh
        self.max_misdetection_thresh = max_misdetection_thresh

        # Inverse cdf where gating_treshold tells the percentile of values, Here, we are assuming that the distance values are distributed in the form gamma distribution
        # where a is shape parameter that tells the skewness of the distribution and scale is scaling factor of distribution.
        # See https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.gamma.html#scipy.stats.gamma
        # TODO chi-squared could be a better option here
        self.gamma = dists.gamma.ppf(gating_treshold, a=1, scale=15)
        self.id_counter = 0

        self.active_tracks = np.array([])
        self.candidate_tracks = np.array([])

    def track_stage_one(self, object_positions, detected_objects, dt):

        # Check if any currently active track has exceeded its lifetime above misdetection threshold. Remove the ones which are not updating
        if len(self.active_tracks) > 0:
            idx = np.where(np.asarray([track.time_since_update + dt <= self.max_misdetection_thresh for track in self.active_tracks]))
            self.active_tracks = self.active_tracks[idx]

        # Check if any candidate track has exceeded its lifetime above misdetection threshold. Remove the ones which are not updating
        if len(self.candidate_tracks) > 0:
            idx = np.where(np.asarray([track.time_since_update + dt <= self.max_misdetection_thresh for track in self.candidate_tracks]))
            self.candidate_tracks = self.candidate_tracks[idx]

        # Apply track associations to active tracks
        self.active_tracks, unassigned_positions, unassigned_detected_objects = self.apply_track_associations_with_update(
            self.active_tracks, object_positions, detected_objects,  dt)

        # Apply track associations to currently available candidate tracks
        self.candidate_tracks, unassigned_positions, unassigned_detected_objects = self.apply_track_associations_with_update(
            self.candidate_tracks, unassigned_positions, unassigned_detected_objects, dt)

        # For object_positions which were not associated to any kind of tracks, generate new track
        for object_position, detected_object in zip(unassigned_positions, unassigned_detected_objects):
            self.candidate_tracks = np.append(self.candidate_tracks, CVKalmanFilterStateEstimator(np.array(object_position), detected_object))

        # Move candidate tracks to active tracks which have lived upto min_life_threshold and have seen min number of detections required
        idx = np.where(np.asarray([(track.lifetime >= self.min_life_threshold and track.num_of_updates >=
                       self.min_detection_thresh) for track in self.candidate_tracks]))

        # Assign id to the newly created tracks
        for track in self.candidate_tracks[idx]:
            track.track_id = self.id_counter
            self.id_counter += 1

        # Update candidate & active tracks list
        self.active_tracks = np.concatenate((self.active_tracks, self.candidate_tracks[idx]))
        self.candidate_tracks = np.delete(self.candidate_tracks, idx, axis=0)

        return self.active_tracks

    def track_stage_two(self, object_positions, dt):
        '''
            TODO: Add a refinement by taking two measurements for removing ghost tracks
        '''
        pass

    def apply_track_associations_with_update(self, tracks, object_positions, detected_objects, dt):
        predicts = [track.predict(dt) for track in tracks]

        if len(predicts) != 0 and len(object_positions) != 0:

            mahalanobis_cost_matrix = np.empty((0, object_positions.shape[0]))

            for state, system_cov in predicts:
                distances = np.squeeze(distance.cdist(object_positions[:, :3], [state[:3]], metric='mahalanobis', VI=np.linalg.inv(system_cov)))
                mahalanobis_cost_matrix = np.vstack([mahalanobis_cost_matrix, distances])

            rows_idx_pred, cols_idx_det = linear_sum_assignment(mahalanobis_cost_matrix)

            gated_idx_pred, gated_idx_det = [], []
            for i in range(rows_idx_pred.size):

                if mahalanobis_cost_matrix[rows_idx_pred[i], cols_idx_det[i]]**2 < self.gamma:
                    gated_idx_pred.append(rows_idx_pred[i])
                    gated_idx_det.append(cols_idx_det[i])

            gated_idx_pred, gated_idx_det = np.array(gated_idx_pred), np.array(gated_idx_det)

        else:
            gated_idx_pred, gated_idx_det = np.array([]), np.array([])

        for i in range(gated_idx_pred.size):
            tracks[gated_idx_pred][i].update(object_positions[gated_idx_det][i], detected_objects[gated_idx_det][i])
            tracks[gated_idx_pred][i].detected_object = detected_objects[gated_idx_det][i]

        return tracks, np.delete(object_positions, gated_idx_det, axis=0), np.delete(detected_objects, gated_idx_det, axis=0)


class CVKalmanFilterStateEstimator():
    def __init__(self, pos, detected_object):
        self.time_since_update = 0
        self.num_of_updates = 0
        self.lifetime = 0
        self.track_id = -1

        self.detected_object = detected_object

        self.filter = KalmanFilter(dim_x=6, dim_z=3)
        self.filter.x = np.array([pos, [0., 0., 0.]]).flatten()

        self.filter.P = np.array([[10., 0., 0., 0., 0., 0.],
                                  [0., 10., 0., 0., 0., 0.],
                                  [0., 0., 2., 0., 0., 0.],
                                  [0., 0., 0., 2., 0., 0.],
                                  [0., 0., 0., 0., 2., 0.],
                                  [0., 0., 0., 0., 0., 2.]])
        self.filter.H = np.array([[1., 0., 0., 0., 0., 0.],
                                  [0., 1., 0., 0., 0., 0.],
                                  [0., 0., 1., 0., 0., 0.]])
        self.filter.R = np.array([[.1, 0, 0],
                                  [0, .1, 0],
                                  [0, 0, 0.1]])

    def predict(self, dt):
        dt = float(dt)

        self.filter.F = np.array([[1., 0., 0., dt, 0., 0.],
                                  [0., 1., 0., 0., dt, 0.],
                                  [0., 0., 1., 0., 0., dt],
                                  [0., 0., 0., 1., 0., 0.],
                                  [0., 0., 0., 0., 1., 0.],
                                  [0., 0., 0., 0., 0., 1.]])
        # Process noise
        self.filter.Q = Q_discrete_white_noise(dim=2, dt=dt, var=5., block_size=3, order_by_dim=False)

        # Run process model step
        self.filter.predict()

        self.time_since_update += dt
        self.lifetime += dt

        # Uncertainty projected in measurement space
        # https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
        PHT = np.dot(self.filter.P, self.filter.H.T)
        S = np.dot(self.filter.H, PHT) + self.filter.R
        # Theoratically could use self.S that is read-only but contains the system uncertainty from last update

        return self.filter.x, S

    def update(self, object_position, detected_object):
        if len(object_position) == 0:
            object_position = None

        self.detected_object = detected_object

        self.filter.update(object_position)
        self.num_of_updates += 1
        self.time_since_update = 0
        
        return self.filter.x, self.filter.S

    def get_current_state(self):
        return self.filter.x

    def get_detected_object(self):

        latest_detected_object = copy.deepcopy(self.detected_object)
        latest_detected_object.id = self.track_id
        latest_detected_object.pose.position.x = self.get_current_state()[0]
        latest_detected_object.pose.position.y = self.get_current_state()[1]
        latest_detected_object.pose.position.z = self.get_current_state()[2]

        # Only update velocity when velocity is not reliable
        if not latest_detected_object.velocity_reliable:
            latest_detected_object.velocity.linear.x = self.get_current_state()[3:][0]
            latest_detected_object.velocity.linear.y = self.get_current_state()[3:][1]
            latest_detected_object.velocity.linear.z = self.get_current_state()[3:][2]

        latest_detected_object.velocity_reliable = True
        latest_detected_object.valid = True

        return latest_detected_object
