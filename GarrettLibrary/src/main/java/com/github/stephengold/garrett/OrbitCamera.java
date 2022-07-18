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

import com.jme3.app.Application;
import com.jme3.app.state.BaseAppState;
import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.input.InputManager;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import java.util.EnumMap;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyCamera;
import jme3utilities.SignalTracker;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * An AppState to control a physics-based, 4 degree-of-freedom Camera. The
 * controlled Camera orbits a specified Target, optionally clipping or jumping
 * forward to maintain a clear line of sight in the target's CollisionSpace. A
 * continuum of chasing behaviors is implemented.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class OrbitCamera
        extends BaseAppState
        implements AnalogListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(OrbitCamera.class.getName());
    /**
     * names of analog events
     */
    final private static String analogOrbitCcw = "orbit ccw";
    final private static String analogOrbitCw = "orbit cw";
    final private static String analogOrbitDown = "orbit down";
    final private static String analogOrbitUp = "orbit up";
    final private static String analogZoomIn = "zoom in";
    final private static String analogZoomOut = "zoom out";
    // *************************************************************************
    // fields

    /**
     * Camera being controlled (not null)
     */
    final private Camera camera;
    /**
     * test whether a collision object can obstruct the line of sight, or null
     * to treat all non-target PCOs as obstructions
     */
    private BulletDebugAppState.DebugAppStateFilter obstructionFilter;
    /**
     * maximum magnitude of the dot product between the camera's look direction
     * and its preferred "up" direction
     */
    private double maxAbsDot = Math.cos(0.3);
    /**
     * map functions to signal names
     */
    final private EnumMap<CameraSignal, String> signalNames
            = new EnumMap<>(CameraSignal.class);
    /**
     * time constant for horizontal rotation (in seconds, 0 &rarr; locked on
     * deltaAzimuthSetpoint, +Infinity &rarr; free horizontal rotation)
     */
    private float azimuthTau = Float.POSITIVE_INFINITY;
    /**
     * setpoint for the azimuth difference between the Camera and the Target (in
     * radians, 0 &rarr;camera following target, Pi/2 &rarr; camera on target's
     * right flank)
     */
    private float deltaAzimuthSetpoint = 0f;
    /**
     * frustum's Y tangent ratio at lowest magnification (&gt;minYTangent)
     */
    private float maxYTangent = 2f;
    /**
     * frustum's Y tangent ratio at highest magnification (&gt;0)
     */
    private float minYTangent = 0.01f;
    /**
     * orbiting rate (in radians per second, &ge;0)
     */
    private float orbitRate = 0.5f;
    /**
     * accumulated analog pitch input since the last update (in 1024-pixel
     * units, measured downward from the look direction)
     */
    private float pitchAnalogSum = 0f;
    /**
     * distance to the near clipping plane if the Camera had X-ray vision
     */
    private float preferredClip = 0.1f;
    /**
     * distance to the Target if the Camera had X-ray vision
     */
    private float preferredRange = 10f;
    /**
     * accumulated analog yaw input since the last update (in 1024-pixel units,
     * measured leftward from the look direction)
     */
    private float yawAnalogSum = 0f;
    /**
     * accumulated analog zoom amount since the last update (in clicks)
     */
    private float zoomAnalogSum = 0f;
    /**
     * analog zoom input multiplier (in log units per click)
     */
    private float zoomMultiplier = 0.3f;
    /**
     * configured response to obstructed line-of-sight
     */
    private ObstructionResponse obstructionResponse = ObstructionResponse.Clip;
    /**
     * reusable Quaternion
     */
    final private static Quaternion tmpRotation = new Quaternion();
    /**
     * status of named signals
     */
    final private SignalTracker signalTracker;
    /**
     * name applied to the Camera when this controller becomes attached and
     * enabled
     */
    private String cameraName;
    /**
     * what's being orbited, or null if none
     */
    private Target target = null;
    /**
     * camera's offset relative to the Target (in world coordinates)
     */
    final private Vector3f offset = new Vector3f();
    /**
     * camera's preferred "up" direction (unit vector in world coordinates)
     */
    final private Vector3f preferredUpDirection = new Vector3f(0f, 1f, 0f);
    /**
     * reusable vectors
     */
    final private static Vector3f tmpCameraLocation = new Vector3f();
    final private static Vector3f tmpLeft = new Vector3f();
    final private static Vector3f tmpLook = new Vector3f();
    final private static Vector3f tmpProj = new Vector3f();
    final private static Vector3f tmpRej = new Vector3f();
    final private static Vector3f tmpTargetLocation = new Vector3f();
    final private static Vector3f tmpUp = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled AppState that will cause the specified Camera to
     * orbit (and optionally chase) a Target.
     *
     * @param id the desired unique ID for this AppState
     * @param camera the Camera to control (not null, alias created)
     * @param tracker the status tracker for named signals (not null, alias
     * created)
     */
    public OrbitCamera(String id, Camera camera, SignalTracker tracker) {
        super(id);
        Validate.nonNull(camera, "camera");
        Validate.nonNull(tracker, "tracker");

        this.camera = camera;
        this.signalTracker = tracker;
        super.setEnabled(false);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Determine the time constant for horizontal rotation.
     *
     * @return the time constant (in seconds, &ge;0, 0 &rarr; locked on
     * deltaAzimuth, +Infinity &rarr; free horizontal rotation)
     */
    public float azimuthTau() {
        assert azimuthTau >= 0f : azimuthTau;
        return azimuthTau;
    }

    /**
     * Determine the name applied to the Camera when this controller becomes
     * attached and enabled.
     *
     * @return the name
     */
    public String cameraName() {
        return cameraName;
    }

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
     * Determine the steady-state azimuth difference between the Camera and the
     * Target.
     *
     * @return the setpoint angle (in radians, &gt;-2*Pi, &lt;2*Pi, 0
     * &rarr;camera following target, Pi/2 &rarr; camera on target's right
     * flank)
     */
    public float deltaAzimuth() {
        assert deltaAzimuthSetpoint >= -FastMath.TWO_PI : deltaAzimuthSetpoint;
        assert deltaAzimuthSetpoint <= FastMath.TWO_PI : deltaAzimuthSetpoint;
        return deltaAzimuthSetpoint;
    }

    /**
     * Access the obstruction filter.
     *
     * @return the pre-existing instance, or null if none
     */
    public BulletDebugAppState.DebugAppStateFilter getObstructionFilter() {
        return obstructionFilter;
    }

    /**
     * Determine the controller's response to an obstructed line-of-sight.
     *
     * @return the enum value (not null)
     */
    public ObstructionResponse getObstructionResponse() {
        assert obstructionResponse != null;
        return obstructionResponse;
    }

    /**
     * Access the Target being orbited.
     *
     * @return the pre-existing instance, or null if none
     */
    public Target getTarget() {
        return target;
    }

    /**
     * Test whether the camera's horizontal rotation is influenced by the
     * Target.
     *
     * @return true if free of influence, otherwise false
     */
    public boolean isAzimuthFree() {
        boolean result = (azimuthTau == Float.POSITIVE_INFINITY);
        return result;
    }

    /**
     * Test whether the camera's horizontal rotation is locked to the Target.
     *
     * @return true if rotation is locked, otherwise false
     */
    public boolean isAzimuthLocked() {
        boolean result = (azimuthTau == 0f);
        return result;
    }

    /**
     * Magnify the view by the specified factor.
     *
     * @param factor the factor to increase magnification (&gt;0)
     */
    public void magnify(float factor) {
        Validate.positive(factor, "factor");

        float frustumYTangent = MyCamera.yTangent(camera);
        frustumYTangent /= factor;
        frustumYTangent
                = FastMath.clamp(frustumYTangent, minYTangent, maxYTangent);
        if (isInitialized() && isEnabled()) {
            MyCamera.setYTangent(camera, frustumYTangent);
        }
    }

    /**
     * Determine the orbital rate.
     *
     * @return the rate (in radians per second, &ge;0)
     */
    public float orbitRate() {
        assert orbitRate >= 0f : orbitRate;
        return orbitRate;
    }

    /**
     * Alter the time constant for horizontal rotation.
     * <p>
     * Allowed only when the controller is NOT attached and enabled.
     *
     * @param timeConstant the desired time constant (in seconds, &ge;0, 0
     * &rarr; locked on the setpoint, +Infinity &rarr; free horizontal rotation,
     * default=+Infinity)
     */
    public void setAzimuthTau(float timeConstant) {
        Validate.nonNegative(timeConstant, "time constant");

        if (isInitialized() && isEnabled()) {
            throw new IllegalStateException("Cannot alter the time constant "
                    + "while the controller is attached and enabled.");
        }
        this.azimuthTau = timeConstant;
    }

    /**
     * Alter the name applied to the Camera when this controller becomes
     * attached and enabled.
     * <p>
     * Allowed only when the controller is NOT attached and enabled.
     *
     * @param name the desired name (default=null)
     */
    public void setCameraName(String name) {
        if (isInitialized() && isEnabled()) {
            throw new IllegalStateException("Cannot alter the camera name "
                    + "while the controller is attached and enabled.");
        }
        this.cameraName = name;
    }

    /**
     * Alter the azimuth difference between the Camera and the Target.
     *
     * @param angle the desired angle (in radians, &gt;-2*Pi, &lt;2*Pi, 0
     * &rarr;camera following target, Pi/2 &rarr; camera on target's right
     * flank, default=0)
     */
    public void setDeltaAzimuth(float angle) {
        Validate.inRange(angle, "angle", -FastMath.TWO_PI, FastMath.TWO_PI);
        this.deltaAzimuthSetpoint = angle;
    }

    /**
     * Alter the range of the camera's focal zoom.
     *
     * @param max the desired maximum magnification (&gt;min, 1&rarr;45deg
     * Y-angle)
     * @param min the desired minimum magnification (&gt;0, 1&rarr;45deg
     * Y-angle)
     */
    public void setMaxMinMagnification(float min, float max) {
        Validate.positive(min, "min magnification");
        Validate.inRange(max, "max magnification", min, Float.MAX_VALUE);

        float frustumYTangent = MyCamera.yTangent(camera);
        this.minYTangent = 1f / max;
        this.maxYTangent = 1f / min;
        frustumYTangent
                = FastMath.clamp(frustumYTangent, minYTangent, maxYTangent);
        if (isInitialized() && isEnabled()) {
            MyCamera.setYTangent(camera, frustumYTangent);
        }
    }

    /**
     * Alter the obstruction filter.
     *
     * @param filter the desired filter to determine which collision objects
     * obstruct the camera's view (alias created) or null to treat all
     * non-target PCOs as obstructions (default=null)
     */
    public void setObstructionFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        this.obstructionFilter = filter;
    }

    /**
     * Alter how the controller responds to an obstructed line of sight.
     *
     * @param response the desired response (not null, default=Clip)
     */
    public void setObstructionResponse(ObstructionResponse response) {
        Validate.nonNull(response, "response");
        this.obstructionResponse = response;
    }

    /**
     * Alter the offset of the Camera from the Target.
     *
     * @param desiredOffset the desired offset from the Target (in world
     * coordinates)
     */
    public void setOffset(Vector3f desiredOffset) {
        Validate.finite(desiredOffset, "offset");
        offset.set(desiredOffset);
    }

    /**
     * Alter the orbital rate.
     *
     * @param rate the desired rate (in radians per second, &ge;0, default=0.5)
     */
    public void setOrbitRate(float rate) {
        Validate.nonNegative(rate, "rate");
        this.orbitRate = rate;
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
     * Alter the preferred distance to the near clipping plane.
     *
     * @param distance the desired distance (in world units, &gt;0)
     */
    public void setPreferredClip(float distance) {
        Validate.positive(distance, "distance");
        this.preferredClip = distance;
    }

    /**
     * Alter the preferred range.
     *
     * @param range the desired distance (in world units, &gt;0)
     */
    public void setPreferredRange(float range) {
        Validate.positive(range, "range");
        this.preferredRange = range;
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
     * Alter the offset and preferred range to reflect the current locations of
     * the Camera and the Target.
     */
    public void setRangeAndOffset() {
        tmpCameraLocation.set(camera.getLocation());
        target.locateTarget(tmpTargetLocation);
        tmpCameraLocation.subtract(tmpTargetLocation, offset);

        this.preferredRange = offset.length();
    }

    /**
     * Alter which signal is assigned to the specified function.
     *
     * @param function which function to alter (not null)
     * @param signalName the desired signal name (may be null)
     */
    public void setSignalName(CameraSignal function, String signalName) {
        Validate.nonNull(function, "function");
        signalNames.put(function, signalName);
    }

    /**
     * Alter which Target is being orbited.
     *
     * @param target the desired Target (not null, alias created)
     */
    public void setTarget(Target target) {
        Validate.nonNull(target, "target");

        if (target != this.target) {
            this.target = target;
            logger.log(Level.INFO, "{0} is the new target.", target);
            setRangeAndOffset();
        }
    }

    /**
     * Alter the analog input multiplier for focal zoom.
     *
     * @param multiplier the desired multiplier (in log units per click, &gt;0,
     * default=0.3)
     */
    public void setZoomMultiplier(float multiplier) {
        Validate.positive(multiplier, "multiplier");
        this.zoomMultiplier = multiplier;
    }

    /**
     * Determine the analog input multiplier for focal zoom.
     *
     * @return the multiplier (in log units per click)
     */
    public float zoomMultiplier() {
        assert zoomMultiplier > 0f : zoomMultiplier;
        return zoomMultiplier;
    }
    // *************************************************************************
    // AnalogListener methods

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

        boolean isDragToOrbit = isActive(CameraSignal.DragToOrbit);
        switch (eventName) {
            case analogOrbitCcw:
                if (isDragToOrbit) {
                    this.yawAnalogSum += reading;
                }
                break;

            case analogOrbitCw:
                if (isDragToOrbit) {
                    this.yawAnalogSum -= reading;
                }
                break;

            case analogOrbitDown:
                if (isDragToOrbit) {
                    this.pitchAnalogSum += reading;
                }
                break;

            case analogOrbitUp:
                if (isDragToOrbit) {
                    this.pitchAnalogSum -= reading;
                }
                break;

            case analogZoomIn:
                this.zoomAnalogSum += reading;
                break;

            case analogZoomOut:
                this.zoomAnalogSum -= reading;
                break;

            default:
                throw new IllegalArgumentException(eventName);
        }
    }
    // *************************************************************************
    // BaseAppState methods

    /**
     * Callback invoked after this AppState is detached or during application
     * shutdown if the state is still attached. onDisable() is called before
     * this cleanup() method if the state is enabled at the time of cleanup.
     *
     * @param application the application instance (not null)
     */
    @Override
    protected void cleanup(Application application) {
        // do nothing
    }

    /**
     * Callback invoked after this AppState is attached but before onEnable().
     *
     * @param application the application instance (not null)
     */
    @Override
    protected void initialize(Application application) {
        // do nothing
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

        if (target == null) {
            logger.warning("No target has been set!");
            return;
        }
        PhysicsCollisionObject targetPco = target.getTargetPco();
        if (targetPco == null) {
            logger.warning("The target has no collision object!");
            return;
        }
        CollisionSpace collisionSpace = targetPco.getCollisionSpace();
        if (collisionSpace == null) {
            logger.warning("The object isn't added to a CollisionSpace!");
            return;
        }
        /*
         * Hide the cursor if dragging.
         */
        InputManager inputManager = getApplication().getInputManager();
        boolean cursorVisible = !isActive(CameraSignal.DragToOrbit);
        inputManager.setCursorVisible(cursorVisible);
        /*
         * Sum the discrete inputs (signals).
         */
        int forwardSum = 0;
        int orbitUpSign = 0;
        int orbitCwSign = 0;
        int zoomSignalDirection = 0;
        for (CameraSignal function : CameraSignal.values()) {
            if (isActive(function)) {
                switch (function) {
                    case Back:
                        --forwardSum;
                        break;

                    case Forward:
                        ++forwardSum;
                        break;

                    case OrbitCcw:
                        --orbitCwSign;
                        break;

                    case OrbitCw:
                        ++orbitCwSign;
                        break;

                    case OrbitDown:
                        --orbitUpSign;
                        break;

                    case OrbitUp:
                        ++orbitUpSign;
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
         * Apply the orbital inputs to the camera offset:
         * first the discrete signals and then the analog values.
         */
        float range = offset.length();
        if (orbitCwSign != 0 || orbitUpSign != 0) {
            float rootSumSquares = MyMath.hypotenuse(orbitCwSign, orbitUpSign);
            float dist = range * orbitRate * tpf / rootSumSquares;

            camera.getLeft(tmpLeft);
            assert tmpLeft.isUnitVector();
            MyVector3f.accumulateScaled(offset, tmpLeft, orbitCwSign * dist);

            camera.getUp(tmpUp);
            assert tmpUp.isUnitVector();
            MyVector3f.accumulateScaled(offset, tmpUp, orbitUpSign * dist);

            float factor = range / offset.length();
            offset.multLocal(factor);
        }
        if (isAzimuthLocked()) {
            yawAnalogSum = 0f;
        }
        if (pitchAnalogSum != 0f || yawAnalogSum != 0f) {
            float multiplier = camera.getHeight() / 1024f;
            float pitchAngle = multiplier * pitchAnalogSum;
            float yawAngle = multiplier * yawAnalogSum;
            tmpRotation.fromAngles(pitchAngle, yawAngle, 0f);
            tmpRotation.mult(offset, offset);

            this.pitchAnalogSum = 0f;
            this.yawAnalogSum = 0f;
        }
        /*
         * Avoid looking too near the preferred "up" direction or its opposite.
         */
        offset.mult(-1f, tmpLook);
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
        if (!isAzimuthFree()) {
            /*
             * Rotate the camera horizontally
             * based on the target's forward direction.
             */
            target.forwardDirection(tmpRej);
            float targetAzimuth = FastMath.atan2(tmpRej.x, tmpRej.z);
            float cameraAzimuth = FastMath.atan2(tmpLook.x, tmpLook.z);
            float deltaAzimuth = cameraAzimuth - targetAzimuth;
            if (Float.isFinite(deltaAzimuth)) {
                float azimuthError = deltaAzimuthSetpoint - deltaAzimuth;
                azimuthError = MyMath.standardizeAngle(azimuthError);
                if (azimuthError != 0f) {
                    float yTurnAngle;
                    if (isAzimuthLocked()) {
                        yTurnAngle = azimuthError;
                    } else {
                        float gain = 1f - FastMath.exp(-tpf / azimuthTau);
                        yTurnAngle = gain * azimuthError;
                    }
                    tmpRotation.fromAngles(0f, yTurnAngle, 0f);
                    tmpRotation.mult(tmpLook, tmpLook);
                }
            }
        }
        /*
         * Apply the new "look" direction to the Camera.
         */
        assert tmpLook.isUnitVector() : tmpLook;
        camera.lookAtDirection(tmpLook, preferredUpDirection);

        boolean xrayVision
                = obstructionResponse == ObstructionResponse.XRay
                || isActive(CameraSignal.Xray);
        boolean jumpy = obstructionResponse == ObstructionResponse.Warp;
        if (forwardSum != 0) {
            range *= FastMath.exp(-tpf * forwardSum); // TODO move rate?
            if (forwardSum > 0 || xrayVision) {
                preferredRange = range;
            }
        } else if (jumpy && range < preferredRange) { // jump backward
            range = preferredRange;
        }
        /*
         * Limit the range to reduce the risk of far-plane clipping.
         */
        float far = camera.getFrustumFar();
        float maxRange = 0.5f * far;
        if (range > maxRange) {
            range = maxRange;
        }

        float near = preferredClip;
        target.locateTarget(tmpTargetLocation);
        if (!xrayVision) {
            /*
             * Test the sightline for obstructions.
             */
            if (jumpy) {
                float rayRange = Math.max(range, preferredRange);
                range = testSightline(rayRange, targetPco);
            } else {
                assert obstructionResponse == ObstructionResponse.Clip :
                        obstructionResponse;
                near = range - testSightline(range, targetPco);
                near = FastMath.clamp(near, preferredClip, far);
            }
        }
        if (obstructionResponse == ObstructionResponse.Clip) {
            MyCamera.setNearFar(camera, near, far);
        }
        /*
         * Calculate the new camera offset and apply it to the Camera.
         */
        tmpLook.mult(-range, offset);
        tmpTargetLocation.add(offset, tmpCameraLocation);
        camera.setLocation(tmpCameraLocation);
        /*
         * Apply focal zoom, if any:
         * first the discrete signals and then the analog values.
         */
        if (zoomSignalDirection != 0) {
            float zoomFactor = FastMath.exp(zoomSignalDirection * tpf);
            magnify(zoomFactor);
        }
        if (zoomAnalogSum != 0f) {
            float zoomFactor = FastMath.exp(zoomMultiplier * zoomAnalogSum);
            magnify(zoomFactor);
            zoomAnalogSum = 0f;
        }
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
        if (isAzimuthFree()) {
            inputManager.deleteMapping(analogOrbitCcw);
            inputManager.deleteMapping(analogOrbitCw);
        }
        inputManager.deleteMapping(analogOrbitDown);
        inputManager.deleteMapping(analogOrbitUp);
        inputManager.deleteMapping(analogZoomIn);
        inputManager.deleteMapping(analogZoomOut);
        inputManager.removeListener(this);

        inputManager.setCursorVisible(true);
    }

    /**
     * Enable this camera controller. Assumes it is initialized and disabled.
     */
    private void enable() {
        assert isInitialized();
        if (target == null) {
            throw new IllegalStateException("No target has been set!");
        }

        camera.setName(cameraName);
        /*
         * Initialize the camera offset and preferred range.
         */
        setRangeAndOffset();

        float yDegrees;
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
        if (isAzimuthFree()) {
            inputManager.addMapping(analogOrbitCcw,
                    new MouseAxisTrigger(MouseInput.AXIS_X, false));
            inputManager.addMapping(analogOrbitCw,
                    new MouseAxisTrigger(MouseInput.AXIS_X, true));
            inputManager.addListener(this, analogOrbitCcw, analogOrbitCw);
        }
        inputManager.addMapping(analogOrbitDown,
                new MouseAxisTrigger(MouseInput.AXIS_Y, true));
        inputManager.addMapping(analogOrbitUp,
                new MouseAxisTrigger(MouseInput.AXIS_Y, false));
        inputManager.addMapping(analogZoomIn,
                new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false));
        inputManager.addMapping(analogZoomOut,
                new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true));

        inputManager.addListener(this, analogOrbitDown, analogOrbitUp,
                analogZoomIn, analogZoomOut);
    }

    /**
     * Test whether the specified camera function (signal) is active.
     */
    private boolean isActive(CameraSignal function) {
        String signalName = signalNames.get(function);
        if (signalName != null && signalTracker.test(signalName)) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test the sightline for obstructions, from the Target to the Camera, using
     * the obstructionFilter (if any). Requires that "tmpLook" and
     * "tmpTargetLocation" be initialized on entry. May modify the "offset" and
     * "tmpCameraLocation" fields.
     *
     * @param range the distance between the Target and the Camera (in world
     * units, &ge;0)
     * @param targetPco the collision object of the Target (not null)
     * @return a modified distance (in world units, &ge;0)
     */
    private float testSightline(float range, PhysicsCollisionObject targetPco) {
        CollisionSpace collisionSpace = targetPco.getCollisionSpace();
        if (collisionSpace == null) {
            return range;
        }

        tmpLook.mult(-range, offset);
        tmpTargetLocation.add(offset, tmpCameraLocation);
        List<PhysicsRayTestResult> hits = collisionSpace.rayTestRaw(
                tmpTargetLocation, tmpCameraLocation);

        float minFraction = 1f;
        for (PhysicsRayTestResult hit : hits) {
            PhysicsCollisionObject pco = hit.getCollisionObject();
            boolean isObstruction = (pco != targetPco)
                    && (obstructionFilter == null
                    || obstructionFilter.displayObject(pco));
            if (isObstruction) {
                float hitFraction = hit.getHitFraction();
                if (hitFraction < minFraction) {
                    minFraction = hitFraction;
                }
            }
        }

        float obstructRange = range * minFraction;
        float result = Math.min(obstructRange, range);

        return result;
    }
}
