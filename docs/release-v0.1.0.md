# First release: v0.1.0

`ai4r_policy` 0.1.0 is the first source release of the standalone ROS 2 Jazzy
student policy package. It contains the single commented policy node, installed
launch file, and documented policy/component configuration. `package.xml` is
the authoritative version. The release remains pending until its reviewed
promotion, final tag, successful tag pipeline, and asset publication complete.

## Compatibility and evidence boundary

Build with the exact `dream_interfaces` v0.1.0 commit recorded in
`ci/dependencies.repos`. The source archive's `release.json` repeats that tag,
commit, and provider URL. The units-bearing `DriveAndSteer` definition is
required; older binaries are incompatible and must be rebuilt.

The synthetic fast gate covers installed policy behavior and launch without
hardware. The source-live check at policy commit
`b8591507b9bdcf8faa4f13da78aa673aa6e0a333` against the DREAM system using
`dream_interfaces` v0.1.1 observed startup publishing zero drive/steer actions
through Foxglove in policy state 2. The standalone behavior gate was not rerun
during that deployment, and no calculation request, vehicle enable, or firmware
flash occurred. Physical stop, watchdog, sensor/frame, pan, and powered-vehicle
acceptance remain not run as listed in [acceptance](acceptance.md).

## Promotion, tags, and publication

The reviewed `dev` to `main` MR is a non-squashed two-parent promotion titled
`release: v0.1.0`; the promotion tree must exactly equal its frozen `dev`
candidate. An optional annotated `v0.1.0-rc.N` tag may point to that exact
candidate on `dev`. The protected annotated final `v0.1.0` tag points to the
promotion on `main`. Candidate tags and final tags have separate identities.

Tagged CI builds the installed policy and exact interface dependency, checks
the installed launch arguments and message interface, and retains a
deterministic full Git source archive, `release.json`, and `SHA256SUMS`. It does
not repeat the full behavior suite already required on the release MR.
Candidate artifacts are never published. After final-tag CI succeeds and the
human review is recorded, a maintainer publishes those exact final-tag files as
durable GitLab Release assets. Existing tags and published assets are immutable.

Record candidate, promotion, equal-tree, tag-object, pipeline, checksum,
publication readback, and human-review evidence in the release record. Then
synchronize `main` back to `dev` through a non-squashed, tree-neutral MR before
ordinary development resumes.
