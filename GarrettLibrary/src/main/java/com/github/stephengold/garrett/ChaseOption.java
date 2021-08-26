/*
 Copyright (c) 2020-2021, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.github.stephengold.garrett;

import com.jme3.math.FastMath;

/**
 * Enumerate chase-behavior options for OrbitCamera.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public enum ChaseOption {
    // *************************************************************************
    // values

    /**
     * Ignore the target's forward direction.
     *
     * Freely orbit the target, including left/right, based on user input.
     *
     * If the target moves, maintain a constant offset from it.
     */
    FreeOrbit(false, 0f, "orbit camera"),
    /**
     * Stay directly behind the target.
     *
     * Orbit the target based on user input, but only up/down.
     *
     * If the target moves, maintain a constant distance and elevation from it.
     */
    StrictFollow(true, 0f, "chase camera"),
    /**
     * Stay directly ahead of the target.
     *
     * Orbit the target based on user input, but only up/down.
     *
     * If the target moves, maintain a constant distance and elevation from it.
     */
    StrictLead(true, FastMath.PI, "lead camera"),
    /**
     * Stay precisely on the target's left flank.
     *
     * Orbit the target based on user input, but only up/down.
     *
     * If the target moves, maintain a constant distance and elevation from it.
     */
    StrictLeft(true, -FastMath.HALF_PI, "left camera"),
    /**
     * Stay precisely on the target's right flank.
     *
     * Orbit the target based on user input, but only up/down.
     *
     * If the target moves, maintain a constant distance and elevation from it.
     */
    StrictRight(true, FastMath.HALF_PI, "right camera");
    // *************************************************************************
    // fields

    /**
     * true if the camera's forward azimuth is strictly controlled, otherwise
     * false
     */
    final private boolean isStrictAzimuth;
    /**
     * if strict, the angle between the forward azimuths of the Camera and the
     * Target (in radians)
     */
    final private float deltaTheta;
    /**
     * name to apply to the Camera
     */
    final private String cameraName;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a ChaseOption.
     *
     * @param isStrict true &rarr; strictly controlled, false &rarr; orbiting
     * @param deltaTheta the desired azimuth angle
     * @param cameraName the desired camera name
     */
    private ChaseOption(boolean isStrict, float deltaTheta, String cameraName) {
        this.isStrictAzimuth = isStrict;
        this.deltaTheta = deltaTheta;
        this.cameraName = cameraName;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Determine the name to apply to the Camera.
     *
     * @return the name
     */
    public String cameraName() {
        return cameraName;
    }

    /**
     * Determine the angle between the forward azimuths of the Camera and the
     * Target.
     *
     * @return the angle (in radians)
     */
    public float deltaTheta() {
        return deltaTheta;
    }

    /**
     * Test whether the camera's forward azimuth is strictly controlled,
     * otherwise false
     *
     * @return true if strictly controlled, otherwise false
     */
    public boolean isStrictAzimuth() {
        return isStrictAzimuth;
    }
}
