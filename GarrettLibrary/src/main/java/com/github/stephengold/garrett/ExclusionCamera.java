/*
 Copyright (c) 2020-2022, Stephen Gold
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
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import java.util.logging.Logger;
import jme3utilities.SignalTracker;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * A camera controller with a preferred "up" direction and pole-exclusion cones.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract class ExclusionCamera extends CameraController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger1
            = Logger.getLogger(ExclusionCamera.class.getName());
    // *************************************************************************
    // fields

    /**
     * maximum dot product between the camera's look direction and its preferred
     * "up" direction (constrains looking up)
     */
    private double maxDot = Math.cos(0.3);
    /**
     * minimum dot product between the camera's look direction and its preferred
     * "up" direction (constrains looking down)
     */
    private double minDot = -Math.cos(0.3);
    /**
     * camera's preferred up direction (unit vector in world coordinates)
     */
    final private Vector3f preferredUpDirection = new Vector3f(0f, 1f, 0f);
    /**
     * reusable vectors
     */
    final private static Vector3f tmpProj = new Vector3f();
    final private static Vector3f tmpRej = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled controller for the specified Camera.
     *
     * @param id the desired unique ID for this AppState
     * @param camera the Camera to control (not null, alias created)
     * @param tracker the status tracker for named signals (not null, alias
     * created)
     */
    protected ExclusionCamera(
            String id, Camera camera, SignalTracker tracker) {
        super(id, camera, tracker);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the preferred "up" direction.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new vector)
     */
    public Vector3f copyPreferredUpDirection(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = preferredUpDirection.clone();
        } else {
            result = storeResult.set(preferredUpDirection);
        }

        return result;
    }

    /**
     * Alter the apertures of the pole-exclusion cones, which prevent the Camera
     * from looking too near its preferred "up" direction or its opposite.
     *
     * @param minAngle the minimum angle between the camera axis and the
     * preferred "up" direction, half the aperture of the exclusion cone (in
     * radians, &ge;0, &le;pi/2, default=0.3)
     */
    public void setPoleExclusionAngle(float minAngle) {
        Validate.inRange(minAngle, "minimum angle", 0f, FastMath.HALF_PI);

        double cos = Math.cos(minAngle);
        if (cos < 0.0) {
            cos = 0.0;
        }
        this.maxDot = cos;
        this.minDot = -cos;
        assert maxDot > minDot : cos;
    }

    /**
     * Alter the apertures of the pole-exclusion cones, which prevent the Camera
     * from looking too near its preferred "up" direction or its opposite.
     *
     * @param upperAngle the minimum angle between the camera axis and the
     * preferred "up" direction, half the aperture of the upper exclusion cone
     * (in radians, &ge;0, &lt;{@code PI - minDownAngle}, default=0.3)
     * @param lowerAngle the minimum angle between the camera axis and the
     * preferred "down" direction, half the aperture of the lower exclusion cone
     * (in radians, &ge;0, &lt;{@code PI - minUpAngle}, default=0.3)
     */
    public void setPoleExclusionAngles(float upperAngle, float lowerAngle) {
        Validate.inRange(upperAngle, "upper angle", 0f, FastMath.PI);
        Validate.inRange(lowerAngle, "lower angle", 0f, FastMath.PI);
        float sum = upperAngle + lowerAngle;
        Validate.require(sum < FastMath.PI, "sum of angles less than pi");

        this.maxDot = Math.cos(upperAngle);
        this.minDot = -Math.cos(lowerAngle);
        assert maxDot > minDot;
    }

    /**
     * Alter the preferred "up" direction.
     *
     * @param direction the desired direction (not null, not zero,
     * default=(0,1,0))
     */
    public void setPreferredUpDirection(Vector3f direction) {
        Validate.nonZero(direction, "direction");

        preferredUpDirection.set(direction);
        preferredUpDirection.normalizeLocal();
    }
    // *************************************************************************
    // new protected methods

    /**
     * Avoid looking too near the preferred "up" direction or its opposite.
     *
     * @param lookDirection the camera's look direction (not null, not zero,
     * modified)
     */
    protected void avoidExclusionCones(Vector3f lookDirection) {
        Validate.nonZero(lookDirection, "look direction");

        MyVector3f.normalizeLocal(lookDirection);
        double dot = MyVector3f.dot(lookDirection, preferredUpDirection);
        if (!MyMath.isBetween(minDot, dot, maxDot)) {
            // looking in an excluded direction

            // tmpRej <- the horizontal direction closest to the look direction
            if (dot >= 1.0 || dot <= -1.0) {
                // looking directly up or down: pick a new direction
                MyVector3f.generateBasis(lookDirection, tmpProj, tmpRej);
                lookDirection.set(tmpRej);
                dot = MyVector3f.dot(lookDirection, preferredUpDirection);
            } else {
                preferredUpDirection.mult((float) dot, tmpProj);
                lookDirection.subtract(tmpProj, tmpRej);
                MyVector3f.normalizeLocal(tmpRej);
            }

            double newDot = MyMath.clamp(dot, minDot, maxDot);
            preferredUpDirection.mult((float) newDot, lookDirection);
            float rejCoefficient = (float) MyMath.circle(newDot);
            MyVector3f.accumulateScaled(lookDirection, tmpRej, rejCoefficient);
        }
    }

    /**
     * Apply the specified look direction to the Camera.
     *
     * @param lookDirection the desired look direction (not null, length=1,
     * modified)
     */
    protected void reorientCamera(Vector3f lookDirection) {
        assert lookDirection.isUnitVector() : lookDirection;

        Camera cam = getCamera();
        cam.lookAtDirection(lookDirection, preferredUpDirection);
    }

    /**
     * Access the preferred "up" direction.
     *
     * @return the pre-existing vector (not null)
     */
    protected Vector3f getPreferredUpDirection() {
        return preferredUpDirection;
    }
}
