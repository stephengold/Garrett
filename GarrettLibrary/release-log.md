# release log for the Garrett Library

## Version 0.5.4 released on 12 June 2025

+ Upgraded JMonkeyEngine to v3.8.1-stable.
+ Upgraded the Heart library to v9.2.0 and the Minie library to v9.0.1

## Version 0.5.3 released on 1 April 2023

+ Added a sweep-test option to `OrbitCamera`
+ Upgraded JMonkeyEngine to v3.6.0-stable.
+ Upgraded the Heart library to v8.3.2 and the Minie library to v7.4.0

## Version 0.5.2 released on 20 November 2022

+ Split the `maxAbsDot` parameter into 2 parameters: `maxDot` and `minDot`.
+ Split the `Warp` obstruction response into 2 options:
  `WarpBias` and `WarpNoBias`.
+ Added contact tests and the `maxFraction` parameter to `OrbitCamera`.
+ Upgraded the Heart library to v8.2.0 and the Minie library to v6.2.0

## Version 0.5.1 released on 28 July 2022

+ Merged all `DragToRotate` functionality into `PointToLook` and deleted the
  `DragToRotate` value from the `CameraSignal` enum. (API change)
+ Added an option to `DynamicCamera` to watch a `Target`.
+ Added the `defaultState()` and `setDefaultState()` methods
  to camera controllers.
+ Rebased all 3 camera controllers on a new superclass.
+ Upgraded the Heart library to v8.1.0

## Version 0.5.0 released on 18 July 2022

Added the `DynamicCamera` controller class.

## Version 0.4.1 released on 11 July 2022

+ Upgraded JMonkeyEngine to v3.5.2-stable.
+ Upgraded the Heart library to v8.0.0 and the Minie library to v5.0.0
+ Added the "checkstyle" plugin to the build.

## Version 0.4.0 released on 24 February 2022

+ Added the `AffixedCamera` controller class.
+ Stopped disabling `OrbitCamera` during updates.

## Version 0.3.1 released on 17 February 2022

+ Added appstate ID to the `OrbitCamera` constructor. (API change)
+ Added 4 methods to the `OrbitCamera` class:
  + `copyPreferredUpDirection()`
  + `getObstructionResponse()`
  + `orbitRate()`
  + `zoomMultiplier()`
+ Upgraded JMonkeyEngine to v3.5.0-stable.
+ Upgraded the Heart library to v7.3.0 and the Minie library to v4.6.1

## Version 0.2.0 released on 28 August 2021

+ Replaced `ChaseOption` with a continuum of time constants and setpoints.
  (API changes!)
+ Added options to `OrbitCamera` to ignore line-of-sight obstructions
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
