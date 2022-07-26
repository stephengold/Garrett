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

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.InputManager;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyCamera;
import jme3utilities.SignalTracker;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * An AppState to control a 6 degree-of-freedom, perspective-projection Camera
 * enclosed in a dynamic spherical shell.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class DynamicCamera
        extends CameraController
        implements PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DynamicCamera.class.getName());
    /**
     * names of analog events
     */
    final private static String analogPitchDown = "pitch down";
    final private static String analogPitchUp = "pitch up";
    final private static String analogYawLeft = "yaw left";
    final private static String analogYawRight = "yaw right";
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * maximum magnitude of the dot product between the camera's look direction
     * and its preferred "up" direction
     */
    private double maxAbsDot = Math.cos(0.3);
    /**
     * signal-based translation speed (in psu per second, &ge;0)
     */
    private float moveSpeed = 1f;
    /**
     * accumulated analog pitch input since the last update (in 1024-pixel
     * units, measured downward from the look direction)
     */
    private float pitchAnalogSum = 0f;
    /**
     * relative turn rate for point-to-look
     */
    private float ptlTurnRate = 1f;
    /**
     * mass of shell when ramming (&gt;0)
     */
    final private float ramMass;
    /**
     * mass of shell when not ramming (&gt;0)
     */
    final private float usualMass;
    /**
     * accumulated analog yaw input since the last update (in 1024-pixel units,
     * measured leftward from the look direction)
     */
    private float yawAnalogSum = 0f;
    /**
     * direction for signal-based zoom (in=+1, out=-1)
     */
    private int zoomSignalDirection = 0;
    /**
     * rigid body being controlled (not null)
     */
    final private PhysicsRigidBody rigidBody;
    /**
     * PhysicsSpace for simulation (not null)
     */
    final private PhysicsSpace physicsSpace;
    /**
     * reusable Quaternion
     */
    final private static Quaternion tmpRotation = new Quaternion();
    /**
     * what's being watched, or null if none
     */
    private Target target = null;
    /**
     * camera's preferred up direction (unit vector in world coordinates)
     */
    final private Vector3f preferredUpDirection = new Vector3f(0f, 1f, 0f);
    /**
     * reusable vectors
     */
    final private static Vector3f tmpCorner = new Vector3f();
    final private static Vector3f tmpLeft = new Vector3f();
    final private static Vector3f tmpLocation = new Vector3f();
    final private static Vector3f tmpLook = new Vector3f();
    final private static Vector3f tmpProj = new Vector3f();
    final private static Vector3f tmpRej = new Vector3f();
    final private static Vector3f tmpScale = new Vector3f();
    final private static Vector3f tmpUp = new Vector3f();
    final private static Vector3f tmpVelocity = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled camera controller.
     *
     * @param id the desired unique ID for this AppState
     * @param camera the Camera to control (not null, alias created)
     * @param physicsSpace the PhysicsSpace for simulation (not null, alias
     * created)
     * @param tracker the status tracker for named signals (not null, alias
     * created)
     * @param usualMass the shell's mass when not ramming (&gt;0)
     * @param ramMass the shell's mass when ramming (&ge;massUsual)
     */
    public DynamicCamera(String id, Camera camera, PhysicsSpace physicsSpace,
            SignalTracker tracker, float usualMass, float ramMass) {
        super(id, camera, tracker);
        Validate.nonNull(physicsSpace, "physics space");
        Validate.positive(usualMass, "usual mass");
        Validate.inRange(ramMass, "ram mass", usualMass, Float.MAX_VALUE);

        this.physicsSpace = physicsSpace;
        this.usualMass = usualMass;
        this.ramMass = ramMass;
        /*
         * Create the shell.
         */
        CollisionShape shape = new MultiSphere(1f); // scalable shape
        rigidBody = new PhysicsRigidBody(shape, usualMass);
        rigidBody.setApplicationData(this);
        rigidBody.setFriction(0f);

        // Initialize some signal names use to imitate FlyByCamera.
        setSignalName(CameraSignal.Back, "FLYCAM_Backward");
        setSignalName(CameraSignal.Forward, "FLYCAM_Forward");
        setSignalName(CameraSignal.Left, "FLYCAM_StrafeLeft");
        setSignalName(CameraSignal.Right, "FLYCAM_StrafeRight");
        setSignalName(CameraSignal.WorldDown, "FLYCAM_Lower");
        setSignalName(CameraSignal.WorldUp, "FLYCAM_Rise");
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
     * Access the rigid body that simulates the shell.
     *
     * @return the pre-existing instance (not null)
     */
    public PhysicsRigidBody getRigidBody() {
        assert rigidBody != null;
        return rigidBody;
    }

    /**
     * Access the Target being watched.
     *
     * @return the pre-existing instance, or null if none
     */
    public Target getTarget() {
        return target;
    }

    /**
     * Return the translation speed.
     *
     * @return the speed (in psu per second, &ge;0)
     */
    public float moveSpeed() {
        assert moveSpeed >= 0f : moveSpeed;
        return moveSpeed;
    }

    /**
     * Return the turn rate for point-to-look.
     *
     * @return the relative turn rate (&gt;0, default=1)
     */
    public float ptlTurnRate() {
        assert ptlTurnRate > 0f : ptlTurnRate;
        return ptlTurnRate;
    }

    /**
     * Determine the radius of the rigid body.
     *
     * @return the radius (in psu, &ge;0)
     */
    public float radius() {
        CollisionShape shape = rigidBody.getCollisionShape();
        float result = shape.getScale(null).x;

        assert result >= 0f : result;
        return result;
    }

    /**
     * Alter the translation speed.
     *
     * @param speed the desired speed (in psu per second, &ge;0, default=1)
     */
    public void setMoveSpeed(float speed) {
        Validate.nonNegative(speed, "speed");
        this.moveSpeed = speed;
    }

    /**
     * Alter the size of the pole-exclusion cone, which prevents the Camera from
     * looking too near the preferred "up" direction or its opposite.
     *
     * @param minAngle the minimum angle between the camera axis and the
     * preferred "up" direction, half the aperture of the exclusion cone (in
     * radians, &ge;0, &le;pi/2, default=0.3)
     */
    public void setPoleExclusionAngle(float minAngle) {
        Validate.inRange(minAngle, "minimum angle", 0f, FastMath.HALF_PI);
        this.maxAbsDot = Math.cos(minAngle);
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

    /**
     * Alter the turn rate for point-to-look.
     *
     * @param turnRate the desired turn rate (in radians/sec, &gt;0)
     */
    public void setPtlTurnRate(float turnRate) {
        Validate.positive(turnRate, "turn rate");
        this.ptlTurnRate = turnRate;
    }

    /**
     * Alter which Target is being watched.
     *
     * @param target the desired Target (alias created) or null for none
     */
    public void setTarget(Target target) {
        if (target != this.target) {
            this.target = target;
            logger.log(Level.INFO, "{0} is the new target.", target);
        }
    }
    // *************************************************************************
    // CameraController methods

    /**
     * Callback to receive an analog input event.
     *
     * @param eventName the name of the input event (not null, not empty)
     * @param reading the input reading (&ge;0)
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAnalog(String eventName, float reading, float tpf) {
        Validate.nonEmpty(eventName, "event name");
        Validate.nonNegative(reading, "reading");
        Validate.nonNegative(tpf, "time per frame");
        assert isEnabled();

        boolean isPointToLook = isActive(CameraSignal.PointToLook);
        switch (eventName) {
            case analogPitchDown:
                if (isPointToLook) {
                    this.pitchAnalogSum += reading;
                }
                break;

            case analogPitchUp:
                if (isPointToLook) {
                    this.pitchAnalogSum -= reading;
                }
                break;

            case analogYawLeft:
                if (isPointToLook) {
                    this.yawAnalogSum += reading;
                }
                break;

            case analogYawRight:
                if (isPointToLook) {
                    this.yawAnalogSum -= reading;
                }
                break;

            case analogZoomIn:
            case analogZoomOut:
                super.onAnalog(eventName, reading, tpf);
                break;

            default:
                throw new IllegalArgumentException(eventName);
        }
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
        /*
         * Hide the cursor if dragging.
         */
        InputManager inputManager = getApplication().getInputManager();
        boolean cursorVisible = !isActive(CameraSignal.PointToLook);
        inputManager.setCursorVisible(cursorVisible);
        /*
         * Update the camera's location to match the rigid body.
         */
        rigidBody.getPhysicsLocation(tmpLocation);
        Camera camera = getCamera();
        camera.setLocation(tmpLocation);

        // Update the camera's direction.
        if (target == null) {
            camera.getDirection(tmpLook);
            /*
             * Rotate the look direction based on accumulated
             * pitch and yaw inputs.
             */
            float frustumYTangent = MyCamera.yTangent(camera);
            float multiplier = camera.getHeight() * frustumYTangent / 1024f;
            multiplier *= ptlTurnRate;
            float pitchAngle = multiplier * pitchAnalogSum;
            float yawAngle = multiplier * yawAnalogSum;
            tmpRotation.fromAngles(pitchAngle, yawAngle, 0f);
            tmpRotation.mult(tmpLook, tmpLook);

        } else { // Look at the target.
            target.locateTarget(tmpLook);
            tmpLook.subtractLocal(tmpLocation);
            MyVector3f.normalizeLocal(tmpLook);
        }

        this.pitchAnalogSum = 0f;
        this.yawAnalogSum = 0f;
        /*
         * Avoid looking too near the preferred "up" direction or its opposite.
         */
        tmpLook.normalizeLocal();
        double dot = MyVector3f.dot(tmpLook, preferredUpDirection);
        if (Math.abs(dot) > maxAbsDot) {
            preferredUpDirection.mult((float) dot, tmpProj);
            tmpLook.subtract(tmpProj, tmpRej);
            double rejL2 = MyVector3f.lengthSquared(tmpRej);
            if (rejL2 > 0.0) { // not directly above or below
                double newDot = MyMath.clamp(dot, maxAbsDot);
                double projCoefficient = newDot / dot;
                double rejCoefficient
                        = Math.sqrt((1.0 - newDot * newDot) / rejL2);
                tmpProj.mult((float) projCoefficient, tmpLook);
                MyVector3f.accumulateScaled(
                        tmpLook, tmpRej, (float) rejCoefficient);
            } else {
                MyVector3f.generateBasis(tmpLook, tmpProj, tmpRej);
                tmpLook.set(tmpProj);
            }
        }
        assert tmpLook.isUnitVector() : tmpLook;
        camera.lookAtDirection(tmpLook, preferredUpDirection);

        applyFocalZoom(zoomSignalDirection, tpf);
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped.
     *
     * @param space the space that is about to be stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        if (!isEnabled()) {
            return;
        }
        updateRigidBodySize();
        updateVelocity();
        rigidBody.setLinearVelocity(tmpVelocity);
        rigidBody.setAngularVelocity(translateIdentity);

        boolean isGhost = isActive(CameraSignal.Ghost);
        rigidBody.setContactResponse(!isGhost);

        float newMass;
        if (isActive(CameraSignal.Ram)) {
            newMass = ramMass;
        } else {
            newMass = usualMass;
        }
        float oldMass = rigidBody.getMass();
        if (newMass != oldMass) {
            rigidBody.setMass(newMass);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Disable this camera controller. Assumes it is initialized and enabled.
     */
    private void disable() {
        assert isInitialized();

        physicsSpace.removeCollisionObject(rigidBody);
        physicsSpace.removeTickListener(this);
        physicsSpace.activateAll(true);
        /*
         * Configure the analog inputs.
         */
        InputManager inputManager = getApplication().getInputManager();
        inputManager.deleteMapping(analogPitchDown);
        inputManager.deleteMapping(analogPitchUp);
        inputManager.deleteMapping(analogYawLeft);
        inputManager.deleteMapping(analogYawRight);
        inputManager.deleteMapping(analogZoomIn);
        inputManager.deleteMapping(analogZoomOut);
        inputManager.removeListener(this);

        inputManager.setCursorVisible(true);
    }

    /**
     * Enable this camera controller. Assumes it is initialized and disabled.
     */
    private void enable() {
        Camera camera = getCamera();
        Vector3f location = camera.getLocation();
        rigidBody.setPhysicsLocation(location);

        float frustumYTangent = MyCamera.yTangent(camera);
        float yDegrees;
        if (camera.isParallelProjection()) {
            yDegrees = MyMath.toDegrees(FastMath.atan(frustumYTangent));
        } else {
            yDegrees = MyCamera.yDegrees(camera);
        }
        float aspectRatio = MyCamera.viewAspectRatio(camera);
        float near = camera.getFrustumNear();
        float far = camera.getFrustumFar();
        camera.setFrustumPerspective(yDegrees, aspectRatio, near, far);
        frustumYTangent = MyCamera.yTangent(camera);

        physicsSpace.addCollisionObject(rigidBody);
        physicsSpace.addTickListener(this);
        rigidBody.setGravity(translateIdentity);
        /*
         * Configure the analog inputs.
         */
        InputManager inputManager = getApplication().getInputManager();
        inputManager.addMapping(analogPitchDown,
                new MouseAxisTrigger(MouseInput.AXIS_Y, false));
        inputManager.addMapping(analogPitchUp,
                new MouseAxisTrigger(MouseInput.AXIS_Y, true));
        inputManager.addMapping(analogYawLeft,
                new MouseAxisTrigger(MouseInput.AXIS_X, true));
        inputManager.addMapping(analogYawRight,
                new MouseAxisTrigger(MouseInput.AXIS_X, false));
        inputManager.addMapping(analogZoomIn,
                new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false));
        inputManager.addMapping(analogZoomOut,
                new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true));

        inputManager.addListener(this, analogPitchDown, analogPitchUp,
                analogYawLeft, analogYawRight, analogZoomIn, analogZoomOut);
    }

    /**
     * Update the size of the rigid body.
     */
    private void updateRigidBodySize() {
        Camera camera = getCamera();
        assert !camera.isParallelProjection();
        float left = camera.getFrustumLeft();
        float top = camera.getFrustumTop();
        float near = camera.getFrustumNear();
        tmpCorner.set(left, top, near);
        float radius = tmpCorner.length();

        CollisionShape shape = rigidBody.getCollisionShape();
        shape.getScale(tmpScale);
        if (tmpScale.x != radius
                || tmpScale.y != radius
                || tmpScale.z != radius) {
            tmpScale.set(radius, radius, radius);
            shape.setScale(tmpScale);
        }
    }

    /**
     * Calculate a new (linear) velocity for the camera and store it in
     * tmpVelocity. The magnitude of the result will either be 0 or moveSpeed.
     * Also update the zoomSignalDirection field.
     */
    private void updateVelocity() {
        Camera camera = getCamera();
        camera.getDirection(tmpLook);
        camera.getLeft(tmpLeft);
        camera.getUp(tmpUp);
        assert tmpLook.isUnitVector() : tmpLook;
        assert tmpLeft.isUnitVector() : tmpLeft;
        assert tmpUp.isUnitVector() : tmpUp;
        /*
         * Sum the active input signals.
         */
        tmpVelocity.zero();
        zoomSignalDirection = 0;
        for (CameraSignal function : CameraSignal.values()) {
            if (isActive(function)) {
                switch (function) {
                    case Back:
                        tmpVelocity.subtractLocal(tmpLook);
                        break;

                    case East:
                        tmpVelocity.z += 1f;
                        break;

                    case Forward:
                        tmpVelocity.addLocal(tmpLook);
                        break;

                    case Left:
                        tmpVelocity.addLocal(tmpLeft);
                        break;

                    case North:
                        tmpVelocity.x += 1f;
                        break;

                    case PreferredDown:
                        tmpVelocity.subtractLocal(preferredUpDirection);
                        break;

                    case PreferredUp:
                        tmpVelocity.addLocal(preferredUpDirection);
                        break;

                    case Right:
                        tmpVelocity.subtractLocal(tmpLeft);
                        break;

                    case South:
                        tmpVelocity.x -= 1f;
                        break;

                    case ViewDown:
                        tmpVelocity.subtractLocal(tmpUp);
                        break;

                    case ViewUp:
                        tmpVelocity.addLocal(tmpUp);
                        break;

                    case West:
                        tmpVelocity.z -= 1f;
                        break;

                    case WorldDown:
                        tmpVelocity.y -= 1f;
                        break;

                    case WorldUp:
                        tmpVelocity.y += 1f;
                        break;

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
         * Scale the velocity so that its magnitude is either 0 or moveSpeed.
         */
        if (!MyVector3f.isZero(tmpVelocity)) {
            tmpVelocity.normalizeLocal();
            tmpVelocity.multLocal(moveSpeed);
        }
    }
}
