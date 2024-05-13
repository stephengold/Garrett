# Garrett Project

[The Garrett Project][garrett] provides a collection of camera controllers for
[the jMonkeyEngine (JME) game engine][jme].

It contains 2 subprojects:

1. GarrettLibrary: the Garrett runtime library
2. GarrettExamples: example applications using the library

Complete source code (in [Java]) is provided under
[a 3-clause BSD license][license].


<a name="toc"></a>

## Contents of this document

+ [Important features](#features)
+ [How to add Garrett to an existing project](#add)
+ [How to build Garrett from source](#build)
+ [Conventions](#conventions)
+ [An overview of the example applications](#examples)
+ [Acknowledgments](#acks)


<a name="features"></a>

## Important features

 + `AffixedCamera`: affixes a camera to a rigid body at a specific offset.
  The controlled camera moves with the rigid body as it translates and rotates.

 + `DynamicCamera`: a physics-based, 6 degree-of-freedom camera controller.
  The controlled camera is enclosed in a spherical rigid body
  that prevents it from penetrating other bodies.

 + `OrbitCamera`: a physics-based, 4 degree-of-freedom camera controller.
  The controlled camera orbits a specified target,
  optionally clipping or jumping forward
  to maintain a clear line of sight in the target's CollisionSpace.
  A continuum of chasing behaviors is implemented.

[Jump to the table of contents](#toc)


<a name="add"></a>

## How to add Garrett to an existing project

Garrett comes pre-built as a single JVM library
that depends on [Minie].
However, the Minie dependency is intentionally omitted from Garrett's POM
so developers can specify *which* Minie library should be used.

For projects built using [Maven] or [Gradle], it is
*not* sufficient to specify the
dependency on Garrett.
You must also explicitly specify the Minie dependency.
The following examples specify "+big4",
but "+debug" or the default Minie library should also work.

### Gradle-built projects

Add to the project’s "build.gradle" file:

    repositories {
        mavenCentral()
    }
    dependencies {
        implementation 'com.github.stephengold:Garrett:0.5.3'
        implementation 'com.github.stephengold:Minie:8.0.0+big4'
    }

For some older versions of Gradle,
it's necessary to replace `implementation` with `compile`.

### Maven-built projects

Add to the project’s "pom.xml" file:

    <repositories>
      <repository>
        <id>mvnrepository</id>
        <url>https://repo1.maven.org/maven2/</url>
      </repository>
    </repositories>

    <dependency>
      <groupId>com.github.stephengold</groupId>
      <artifactId>Garrett</artifactId>
      <version>0.5.3</version>
    </dependency>

    <dependency>
      <groupId>com.github.stephengold</groupId>
      <artifactId>Minie</artifactId>
      <version>8.0.0+big4</version>
    </dependency>

[Jump to the table of contents](#toc)


<a name="build"></a>

## How to build Garrett from source

1. Install a [Java Development Kit (JDK)][adoptium],
   if you don't already have one.
2. Point the `JAVA_HOME` environment variable to your JDK installation:
   (In other words, set it to the path of a directory/folder
   containing a "bin" that contains a Java executable.
   That path might look something like
   "C:\Program Files\Eclipse Adoptium\jdk-17.0.3.7-hotspot"
   or "/usr/lib/jvm/java-17-openjdk-amd64/" or
   "/Library/Java/JavaVirtualMachines/zulu-17.jdk/Contents/Home" .)
  + using Bash or Zsh: `export JAVA_HOME="` *path to installation* `"`
  + using [Fish]: `set -g JAVA_HOME "` *path to installation* `"`
  + using Windows Command Prompt: `set JAVA_HOME="` *path to installation* `"`
  + using PowerShell: `$env:JAVA_HOME = '` *path to installation* `'`
3. Download and extract the Garrett source code from GitHub:
  + using [Git]:
    + `git clone https://github.com/stephengold/Garrett.git`
    + `cd Garrett`
    + `git checkout -b latest 0.5.3`
  + using a web browser:
    + browse to [the latest release][latest]
    + follow the "Source code (zip)" link
    + save the ZIP file
    + extract the contents of the saved ZIP file
    + `cd` to the extracted directory/folder
4. Run the [Gradle] wrapper:
  + using Bash or Fish or PowerShell or Zsh: `./gradlew build`
  + using Windows Command Prompt: `.\gradlew build`

After a successful build,
Maven artifacts will be found in "GarrettLibrary/build/libs".

You can install the artifacts to your local Maven repository:
+ using Bash or Fish or PowerShell or Zsh: `./gradlew install`
+ using Windows Command Prompt: `.\gradlew install`

You can restore the project to a pristine state:
+ using Bash or Fish or PowerShell or Zsh: `./gradlew clean`
+ using Windows Command Prompt: `.\gradlew clean`

[Jump to the table of contents](#toc)


<a name="conventions"></a>

## Conventions

Library classes are in the `com.github.stephengold.garrett` package.
Example classes are in the `com.github.stephengold.garrett.examples` package.

The source code and pre-built libraries are compatible with JDK 8.

[Jump to the table of contents](#toc)


<a name="examples"></a>

## An overview of the example applications

Applications have been created to test and demonstrate
certain features of Garrett.
The following apps are found in the GarrettExamples subproject:

### HelloGarrett

A very simple example of how Garrett maps keys to input signals.
Using `AffixedCamera`, the camera stays at a fixed offset from the red sphere.

+ "W" key, up-arrow key, equals key, plus key, or mouse wheel to zoom in
+ "S" key, down-arrow key, hyphen key, minus key, or mouse wheel to zoom out

### HelloDynaCam

An example of a camera controlled by `DynamicCamera`.

+ move the mouse to rotate the camera in "point-to-look" mode
+ equals key, plus key, or mouse wheel to zoom in
+ hyphen key, minus key, or mouse wheel to zoom out
+ "W" key to move the camera forward
+ "S" key to move the camera backward
+ left-arrow key or "A" key to strafe left
+ right-arrow key or "D" key to strafe right
+ "Q" key to raise the camera along the world's Y axis
+ "Z" key to lower the camera along the world's Y axis
+ up-arrow key to move the camera upward in view coordinates
+ down-arrow key to move the camera downward in view coordinates
+ hold down the "G" key for "ghost mode", which temporarily disables physics
+ hold down the "R" key for "ram mode", which temporarily increase the mass
+ hold down the left shift key to tempararily disable point-to-look mode

### HelloOrbitCam

An example of a camera controlled by `OrbitCamera`.
The camera orbits the red ball, which is its target.

+ drag with the left mouse button (LMB) to orbit the ball on 2 axes
+ equals key, plus key, or mouse wheel to zoom in
+ hyphen key, minus key, or mouse wheel to zoom out
+ "W" key to move the camera forward (toward the ball)
+ "S" key to move the camera backward (away from the ball)
+ left-arrow key or "A" key to orbit left (counter-clockwise, seen from above)
+ right-arrow key or "D" key to orbit right (clockwise, seen from above)
+ "Q" key to orbit upward
+ "Z" key to orbit downward
+ hold down the "X" key for "X-ray mode", which temporarily ignores obstructions

[Jump to the table of contents](#toc)


<a name="acks"></a>

## Acknowledgments

Like most projects, the Garrett Project builds on the work of many who
have gone before.  I therefore acknowledge the creators of (and contributors to)
the following software:

+ the [Checkstyle] tool
+ the [Git] revision-control system and GitK commit viewer
+ the [GitKraken] client
+ the [Firefox] web browser
+ the [Gradle] build tool
+ the [Java] compiler, standard doclet, and runtime environment
+ [jMonkeyEngine][jme] and the jME3 Software Development Kit
+ the [Linux Mint][mint] operating system
+ [LWJGL], the Lightweight Java Game Library
+ the [Markdown] document-conversion tool
+ the [Meld] visual merge tool
+ Microsoft Windows
+ the [NetBeans] integrated development environment

I am grateful to [GitHub] and [Sonatype]
for providing free hosting for this project
and many other open-source projects.

I'm also grateful to my dear Holly, for keeping me sane.

If I've misattributed anything or left anyone out, please let me know, so I can
correct the situation: sgold@sonic.net

[Jump to the table of contents](#toc)


[adoptium]: https://adoptium.net/releases.html "Adoptium Project"
[ant]: https://ant.apache.org "Apache Ant Project"
[bsd3]: https://opensource.org/licenses/BSD-3-Clause "3-Clause BSD License"
[checkstyle]: https://checkstyle.org "Checkstyle"
[firefox]: https://www.mozilla.org/en-US/firefox "Firefox"
[fish]: https://fishshell.com/ "Fish command-line shell"
[garrett]: https://github.com/stephengold/Garrett "Garrett Project"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gitkraken]: https://www.gitkraken.com "GitKraken client"
[gradle]: https://gradle.org "Gradle Project"
[java]: https://en.wikipedia.org/wiki/Java_(programming_language) "Java programming language"
[jme]: https://jmonkeyengine.org "jMonkeyEngine Project"
[latest]: https://github.com/stephengold/Garrett/releases/latest "latest release"
[license]: https://github.com/stephengold/Garrett/blob/master/LICENSE "Garrett license"
[lwjgl]: https://www.lwjgl.org "Lightweight Java Game Library"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[maven]: https://maven.apache.org "Maven Project"
[meld]: https://meldmerge.org "Meld merge tool"
[minie]: https://stephengold.github.io/Minie/minie/overview.html "Minie Project"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[sonatype]: https://www.sonatype.com "Sonatype"
[utilities]: https://github.com/stephengold/jme3-utilities "Jme3-utilities Project"
