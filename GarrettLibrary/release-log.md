# release log for the Garrett Library

## Version 0.3.1 relased on 17 February 2022

+ Added appstate ID to the `OrbitCamera` constructor. (API change)
+ Added 4 methods to the `OrbitCamera` class:
  + `copyPreferredUpDirection()`
  + `getObstructionResponse()`
  + `orbitRate()`
  + `zoomMultiplier()`
+ Upgraded JMonkeyEngine to v3.5.0-stable.
+ Upgraded the Heart library to v7.3.0 and the Minie library to v4.6.1

## Version 0.2.0 relased on 28 August 2021

+ Replace `ChaseOption` with a continuum of time constants and setpoints.
  (API changes!)
+ Add options to `OrbitCamera` to ignore line-of-sight obstructions
  or respond by clipping instead of warping forward.

## Version 0.1.5 released on 23 August 2021

Upgraded the Heart library to v7.0.0 and the Minie library to v4.3.0

## Version 0.1.4 released on 3 July 2021

+ Bugfix: `Float.isFinite()` isn't compatible with Java v7
+ Added the `getObstructionFilter()` and `getTarget()` methods
  to the `OrbitCamera` class.
+ Upgraded the Minie library to v4.2.0

## Version 0.1.3 released on 2 June 2021

+ Upgraded JMonkeyEngine to v3.4.0-stable.
+ Upgraded the Heart library to v6.4.4 and the Minie library to v4.1.1

## Version 0.1.2 released on 11 February 2021

+ Published to MavenCentral instead of JCenter.
+ Upgraded the Heart library to v6.4.2 and the Minie library to v4.0.0

## Version 0.1.1 released on 30 January 2021

+ Don't "lock in" a particular Minie build, so developers can use the "big3"
  build if they wish.
+ Upgraded the Heart library to v6.4.0

## Version 0.1.0 released on 25 January 2021

+ Added `OrbitCamera` and its supporting classes, based on code
  from the BatsV2 project.
