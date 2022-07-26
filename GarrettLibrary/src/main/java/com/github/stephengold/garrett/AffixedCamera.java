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

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.InputManager;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyCamera;
import jme3utilities.SignalTracker;
import jme3utilities.Validate;

/**
 * An AppState to control a Camera affixed to (mounted on) a rigid body.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class AffixedCamera extends CameraController {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(AffixedCamera.class.getName());
    // *************************************************************************
    // fields

    /**
     * what the camera is affixed to
     */
    private PhysicsRigidBody rigidBody;
    /**
     * reusable Quaternion
     */
    final private static Quaternion tmpRotation = new Quaternion();
    /**
     * camera's "look" direction relative to the rigid body (unit vector in
     * local coordinates)
     */
    final private Vector3f lookDirection = new Vector3f(0f, 0f, 1f);
    /**
     * camera's offset from to the rigid body's center of mass (in scaled local
     * coordinates)
     */
    final private Vector3f offset = new Vector3f();
    /**
     * camera's "up" direction relative to the rigid body (unit vector in local
     * coordinates)
     */
    final private Vector3f upDirection = new Vector3f(0f, 1f, 0f);
    /**
     * reusable vectors
     */
    final private static Vector3f tmpCameraLocation = new Vector3f();
    final private static Vector3f tmpLook = new Vector3f();
    final private static Vector3f tmpOffset = new Vector3f();
    final private static Vector3f tmpUp = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled AppState that will affix the specified Camera to a
     * rigid body.
     *
     * @param id the desired unique ID for this AppState
     * @param camera the Camera to control (not null, alias created)
     * @param tracker the status tracker for named signals (not null, alias
     * created)
     */
    public AffixedCamera(String id, Camera camera, SignalTracker tracker) {
        super(id, camera, tracker);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the "look" direction (in local coordinates).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new vector)
     */
    public Vector3f copyLookDirection(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = lookDirection.clone();
        } else {
            result = storeResult.set(lookDirection);
        }

        return result;
    }

    /**
     * Copy the camera offset from the rigid body's center of mass.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an offset vector in scaled local coordinates (either storeResult
     * or a new vector)
     */
    public Vector3f copyOffset(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = offset.clone();
        } else {
            result = storeResult.set(offset);
        }

        return result;
    }

    /**
     * Copy the "up" direction (in local coordinates).
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (either storeResult or a new vector)
     */
    public Vector3f copyUpDirection(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = upDirection.clone();
        } else {
            result = storeResult.set(upDirection);
        }

        return result;
    }

    /**
     * Access the rigid body to which the Camera is affixed.
     *
     * @return the pre-existing instance, or null if none
     */
    public PhysicsRigidBody getRigidBody() {
        return rigidBody;
    }

    /**
     * Alter the "look" direction.
     *
     * @param direction the desired direction (in local coordinates, not null,
     * not zero, default=(0,0,1))
     */
    public void setLookDirection(Vector3f direction) {
        Validate.nonZero(direction, "direction");

        lookDirection.set(direction);
        lookDirection.normalizeLocal();
    }

    /**
     * Alter the offset of the Camera from the rigid body's center of mass.
     *
     * @param desiredOffset the desired offset (in scaled local coordinates)
     */
    public void setOffset(Vector3f desiredOffset) {
        Validate.finite(desiredOffset, "desired offset");
        offset.set(desiredOffset);
    }

    /**
     * Alter which rigid body the Camera is affixed to.
     *
     * @param rigidBody the desired rigid body (not null, alias created)
     */
    public void setRigidBody(PhysicsRigidBody rigidBody) {
        Validate.nonNull(rigidBody, "rigid body");

        if (rigidBody != this.rigidBody) {
            this.rigidBody = rigidBody;
            logger.log(Level.INFO, "{0} is the new rigid body.", rigidBody);
        }
    }

    /**
     * Alter the "up" direction.
     *
     * @param direction the desired direction (in local coordinates, not null,
     * not zero, default=(0,1,0))
     */
    public void setUpDirection(Vector3f direction) {
        Validate.nonZero(direction, "direction");

        upDirection.set(direction);
        upDirection.normalizeLocal();
    }

    /**
     * Callback invoked whenever this AppState ceases to be both attached and
     * enabled.
     */
    @Override
    protected void onDisable() {
        disable();
    }

    /**
     * Callback invoked whenever this AppState becomes both attached and
     * enabled.
     */
    @Override
    protected void onEnable() {
        super.onEnable();
        enable();
    }

    /**
     * Callback to update this state prior to rendering. (Invoked once per frame
     * while the state is attached and enabled.)
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        super.update(tpf);

        if (rigidBody == null) {
            logger.warning("No rigid body has been set!");
            return;
        }
        /*
         * Sum the discrete inputs (signals).
         */
        int zoomSignalDirection = 0;
        for (CameraSignal function : CameraSignal.values()) {
            if (isActive(function)) {
                switch (function) {
                    case ZoomIn:
                        ++zoomSignalDirection;
                        break;

                    case ZoomOut:
                        --zoomSignalDirection;
                        break;

                    default:
                }
            }
        }
        /*
         * Update the camera's orientation.
         */
        rigidBody.getPhysicsRotation(tmpRotation);
        tmpRotation.mult(lookDirection, tmpLook);
        tmpRotation.mult(upDirection, tmpUp);
        Camera camera = getCamera();
        camera.lookAtDirection(tmpLook, tmpUp);
        /*
         * Update the camera's location.
         */
        tmpRotation.mult(offset, tmpOffset);
        rigidBody.getMotionState().getLocation(tmpCameraLocation);
        tmpCameraLocation.addLocal(tmpOffset);
        camera.setLocation(tmpCameraLocation);

        applyFocalZoom(zoomSignalDirection, tpf);
    }
    // *************************************************************************
    // private methods

    /**
     * Disable this camera controller. Assumes it is initialized and enabled.
     */
    private void disable() {
        assert isInitialized();
        /*
         * Configure the analog inputs.
         */
        InputManager inputManager = getApplication().getInputManager();
        inputManager.deleteMapping(analogZoomIn);
        inputManager.deleteMapping(analogZoomOut);
        inputManager.removeListener(this);

        inputManager.setCursorVisible(true);
    }

    /**
     * Enable this camera controller. Assumes it is initialized and disabled.
     */
    private void enable() {
        if (rigidBody == null) {
            throw new IllegalStateException("No rigid body has been set!");
        }

        float yDegrees;
        Camera camera = getCamera();
        if (camera.isParallelProjection()) {
            /*
             * Configure perspective.
             */
            yDegrees = 30f;
            float aspectRatio = MyCamera.viewAspectRatio(camera);
            float near = camera.getFrustumNear();
            float far = camera.getFrustumFar();
            camera.setFrustumPerspective(yDegrees, aspectRatio, near, far);
        }
        /*
         * Configure the analog inputs.
         */
        InputManager inputManager = getApplication().getInputManager();
        inputManager.addMapping(analogZoomIn,
                new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false));
        inputManager.addMapping(analogZoomOut,
                new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true));

        inputManager.addListener(this, analogZoomIn, analogZoomOut);
    }
}
