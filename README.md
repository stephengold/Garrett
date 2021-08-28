The [Garrett Project][garrett] provides a collection of camera controllers for
[the jMonkeyEngine (JME) game engine][jme].

Complete source code (in Java) is provided under
[a 3-clause BSD license][license].

<a name="toc"/>

## Contents of this document

+ [Important features](#features)
+ [How to add Garrett to an existing project](#add)
+ [Conventions](#conventions)
+ [How to build Garrett from source](#build)

<a name="features"/>

## Important features

 + `OrbitCamera`: a physics-based, 4 degree-of-freedom camera controller.
  The controlled camera orbits a specified target,
  optionally clipping or jumping forward
  to maintain a clear line of sight in the target's CollisionSpace.
  A continuum of chasing behaviors is implemented.

[Jump to table of contents](#toc)

<a name="add"/>

## How to add Garrett to an existing project

The Garrett Library depends on [Minie].
However, the Minie dependency is intentionally omitted from Garrett's POM
so developers can specify *which* Minie library should be used.

For projects built using Maven or [Gradle], it is *not* sufficient to specify the
dependency on the Garrett Library.
You must also explicitly specify the Minie dependency.
The following examples specify "+big3",
but "+debug" or the default Minie library should also work.

### Gradle-built projects

Add to the project’s "build.gradle" file:

    repositories {
        mavenCentral()
    }
    dependencies {
        compile 'com.github.stephengold:Garrett:0.2.0'
        compile 'com.github.stephengold:Minie:4.3.0+big3'
    }

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
      <version>0.2.0</version>
    </dependency>

    <dependency>
      <groupId>com.github.stephengold</groupId>
      <artifactId>Minie</artifactId>
      <version>4.3.0+big3</version>
    </dependency>

[Jump to table of contents](#toc)

<a name="conventions"/>

## Conventions

Classes are located in the `com.github.stephengold.garrett` package.

Both the source code and the pre-built libraries are compatible with JDK 7.

[Jump to table of contents](#toc)

<a name="build"/>

## How to build Garrett from source

 1. Install a [Java Development Kit (JDK)][openJDK],
    if you don't already have one.
 2. Download and extract the Garrett source code from GitHub:
   + using Git:
     + `git clone https://github.com/stephengold/Garrett.git`
     + `cd Garrett`
     + `git checkout -b latest 0.2.0`
   + using a web browser:
     + browse to [the latest release][latest]
     + follow the "Source code (zip)" link
     + save the ZIP file
     + extract the contents of the saved ZIP file
     + `cd` to the extracted directory/folder
 3. Set the `JAVA_HOME` environment variable:
   + using Bash:  `export JAVA_HOME="` *path to your JDK* `"`
   + using Windows Command Prompt:  `set JAVA_HOME="` *path to your JDK* `"`
   + using PowerShell: `$env:JAVA_HOME = '` *path to your JDK* `'`
 4. Run the [Gradle] wrapper:
   + using Bash or PowerShell:  `./gradlew build`
   + using Windows Command Prompt:  `.\gradlew build`

After a successful build,
Maven artifacts will be found in `GarrettLibrary/build/libs`.

You can install the Maven artifacts to your local Maven repository:
 + using Bash or PowerShell:  `./gradlew install`
 + using Windows Command Prompt:  `.\gradlew install`

You can restore the project to a pristine state:
 + using Bash or PowerShell: `./gradlew clean`
 + using Windows Command Prompt: `.\gradlew clean`

[Jump to table of contents](#toc)

<a name="acks"/>

## Acknowledgments

Like most projects, the Garrett Project builds on the work of many who
have gone before.  I therefore acknowledge the following
software developers:

 + plus the creators of (and contributors to) the following software:
    + the [Git] revision-control system and GitK commit viewer
    + the [Firefox] web browser
    + the [Gradle] build tool
    + the Java compiler, standard doclet, and virtual machine
    + [jMonkeyEngine][jme] and the jME3 Software Development Kit
    + the [Linux Mint][mint] operating system
    + LWJGL, the Lightweight Java Game Library
    + the [Markdown] document-conversion tool
    + Microsoft Windows
    + the [NetBeans] integrated development environment

I am grateful to [GitHub] and [Sonatype]
for providing free hosting for this project
and many other open-source projects.

I'm also grateful to my dear Holly, for keeping me sane.

If I've misattributed anything or left anyone out, please let me know, so I can
correct the situation: sgold@sonic.net

[Jump to table of contents](#toc)


[ant]: https://ant.apache.org "Apache Ant Project"
[bsd3]: https://opensource.org/licenses/BSD-3-Clause "3-Clause BSD License"
[firefox]: https://www.mozilla.org/en-US/firefox "Firefox"
[garrett]: https://github.com/stephengold/Garrett "Garrett Project"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gradle]: https://gradle.org "Gradle Project"
[jme]: https://jmonkeyengine.org  "jMonkeyEngine Project"
[latest]: https://github.com/stephengold/Garrett/releases/latest "latest release"
[license]: https://github.com/stephengold/Garrett/blob/master/LICENSE "Garrett license"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[minie]: https://github.com/stephengold/Minie "Minie Project"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[openJDK]: https://openjdk.java.net "OpenJDK Project"
[sonatype]: https://www.sonatype.com "Sonatype"
[utilities]: https://github.com/stephengold/jme3-utilities "Jme3-utilities Project"
