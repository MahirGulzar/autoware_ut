# Radar Relay Node
 The nodes process radar output and publishes `DetectedObjectArray` to be used by Openplanner.
 
### Subscriptions

|Topic|Message|
|---|---|
|/tracked_objects | [derived_object_msgs/ObjectWithCovarianceArray](http://docs.ros.org/melodic/api/derived_object_msgs/html/msg/ObjectWithCovarianceArray.html)|
|/radar_fc/as_tx/radar_tracks | [radar_msgs/RadarTrackArray](http://docs.ros.org/kinetic/api/radar_msgs/html/msg/RadarTrackArray.html)|


### Publications

|Topic|Message|
|---|---|
|/tracked_objects   | [autoware_msgs/DetectedObjectArray](https://gitlab.com/astuff/autoware.ai/messages/-/blob/oap5/autoware_msgs/msg/DetectedObjectArray.msg)|
|/detected_polygons | [visualization_msgs/MarkerArray](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html)|


### Parameters

|Parameter name|Type|Default|Description|
|---|---|---|---|
|target_frame                    |Private |`map`                        |Frame in which are the objects outputted|
|objects_input_topic             |Private |`radar_fc/as_tx/objects`     |Radar object input topic|
|radar_tracks_input_topic        |Private |`radar_fc/as_tx/radar_tracks`|Radar tracks input topic|
|detected_objects_output_topic   |Private |`tracked_objects`            |Tracked object output topic (For OP)|
|detected_objects_output_topic   |Private |`detected_polygons`          |Visualization topic|
