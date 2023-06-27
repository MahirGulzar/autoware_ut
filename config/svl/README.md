## Config files for SVL sim

#### TODO:
- yaml files need to be reviewed:
  - param duplication removed
  - see if can be merged with existing general yaml files
- currently not loaded from anywhere
- loading depends on `platform:=svl_sim` and might be necessary
  - to separate into: localization_svl.yaml, detection_svl.yaml .. etc.
  - Option to load after main param loading using `if="$(eval arg('platform')=='svl_sim')"`

```
<group if="$(eval arg('platform')=='svl_sim')">
    <rosparam command="load" subst_value="true" file="$(find autoware_ut)/config/localization_svl.yaml"/>
</group>
```