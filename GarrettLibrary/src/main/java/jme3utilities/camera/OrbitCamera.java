/*
 Copyright (c) 2020, Stephen Gold
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
package jme3utilities.camera;

import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.debug.BulletDebugAppState;
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
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import jme3utilities.ui.ActionAppState;

/**
 * A VehicleCamera to control a 4 degree-of-freedom Camera that chases and
 * orbits a Target, jumping forward as needed to maintain a clear line of sight
 * in the target's CollisionSpace.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class OrbitCamera
        extends ActionAppState
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
     * test whether a collision object can obstruct the line of sight, or null
     * to treat all non-target PCOs as obstructions
     */
    final private BulletDebugAppState.DebugAppStateFilter obstructionFilter;
    /**
     * Camera being controlled (not null)
     */
    final private Camera camera;
    /**
     * maximum magnitude of the dot product between the camera's look direction
     * and its preferred "up" direction (default=cos(0.3))
     */
    private double maxAbsDot = Math.cos(0.3);
    /**
     * map functions to signal names
     */
    final private EnumMap<OcFunction, String> signalNames
            = new EnumMap<>(OcFunction.class);
    /**
     * frustum's Y tangent ratio
     */
    private float frustumYTangent = 1f;
    /**
     * frustum's Y tangent ratio at lowest magnification (&gt;minYTangent)
     */
    private float maxYTangent = 2f;
    /**
     * frustum's Y tangent ratio at highest magnification (&gt;0)
     */
    private float minYTangent = 0.01f;
    /**
     * orbiting rate (in radians per second, &ge;0, default=0.5)
     */
    private float orbitRate = 0.5f;
    /**
     * accumulated analog pitch input since the last update (in 1024-pixel
     * units, measured downward from the look direction)
     */
    private float pitchAnalogSum = 0f;
    /**
     * distance from the center if the camera had X-ray vision
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
     * analog zoom input multiplier (in log units per click, default=0.5)
     */
    private float zoomMultiplier = 0.5f;
    /**
     * reusable Quaternion
     */
    final private static Quaternion tmpRotation = new Quaternion();
    /**
     * what's being orbited, or null if none
     */
    private Target target = null;
    /**
     * camera's preferred "up" direction (unit vector in world coordinates)
     */
    final private Vector3f preferredUpDirection = new Vector3f(0f, 1f, 0f);
    /**
     * reusable vectors
     */
    final private static Vector3f tmpCenter = new Vector3f();
    final private static Vector3f tmpLeft = new Vector3f();
    final private static Vector3f tmpLocation = new Vector3f();
    final private static Vector3f tmpLook = new Vector3f();
    final private static Vector3f tmpOffset = new Vector3f();
    final private static Vector3f tmpProj = new Vector3f();
    final private static Vector3f tmpRej = new Vector3f();
    final private static Vector3f tmpUp = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled camera controller that chases and orbits a target.
     *
     * @param camera the Camera to control (not null, alias created)
     * @param obstructionFilter to determine which collision objects obstruct
     * the camera's view (alias created) or null to treat all non-target PCOs as
     * obstructions
     */
    public OrbitCamera(Camera camera,
            BulletDebugAppState.DebugAppStateFilter obstructionFilter) {
        super(false);
        Validate.nonNull(camera, "camera");

        this.camera = camera;
        this.obstructionFilter = obstructionFilter;
        /*
         * Initialize some signal names.
         */
        signalNames.put(OcFunction.Back, "FLYCAM_Backward");
        signalNames.put(OcFunction.DragToOrbit, "cameraDrag");
        signalNames.put(OcFunction.Forward, "FLYCAM_Forward");
        signalNames.put(OcFunction.OrbitCcw, "FLYCAM_StrafeRight");
        signalNames.put(OcFunction.OrbitCw, "FLYCAM_StrafeLeft");
        signalNames.put(OcFunction.OrbitDown, "FLYCAM_Lower");
        signalNames.put(OcFunction.OrbitUp, "FLYCAM_Rise");

        assert !isEnabled();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Magnify the view by the specified factor.
     *
     * @param factor the factor to increase magnification (&gt;0)
     */
    public void magnify(float factor) {
        Validate.positive(factor, "factor");

        frustumYTangent /= factor;
        frustumYTangent
                = FastMath.clamp(frustumYTangent, minYTangent, maxYTangent);
        if (isInitialized() && isEnabled()) {
            MyCamera.setYTangent(camera, frustumYTangent);
        }
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

        minYTangent = 1f / max;
        maxYTangent = 1f / min;
        frustumYTangent
                = FastMath.clamp(frustumYTangent, minYTangent, maxYTangent);
        if (isInitialized() && isEnabled()) {
            MyCamera.setYTangent(camera, frustumYTangent);
        }
    }

    /**
     * Alter the orbital rate.
     *
     * @param rate the desired rate (in radians per second, &ge;0, default=0.5)
     */
    public void setOrbitRate(float rate) {
        Validate.nonNegative(rate, "rate");
        orbitRate = rate;
    }

    /**
     * Alter the size of the pole-exclusion cone, which keeps the camera from
     * looking too near the preferred "up" direction or its opposite.
     *
     * @param minAngle the minimum angle between the camera axis and the
     * preferred "up" direction, half the aperture of the exclusion cone (in
     * radians, &ge;0, &le;pi/2, default=0.3)
     */
    public void setPoleExclusionAngle(float minAngle) {
        Validate.inRange(minAngle, "minimum angle", 0f, FastMath.HALF_PI);
        maxAbsDot = Math.cos(minAngle);
    }

    /**
     * Alter the preferred "up" direction.
     *
     * @param direction the desired direction (not null, not zero)
     */
    public void setPreferredUpDirection(Vector3f direction) {
        Validate.nonZero(direction, "direction");

        preferredUpDirection.set(direction);
        preferredUpDirection.normalizeLocal();
    }

    /**
     * Alter which signal is assigned to the specified function.
     *
     * @param function which function to alter (not null)
     * @param signalName the desired signal name (may be null)
     */
    public void setSignalName(OcFunction function, String signalName) {
        Validate.nonNull(function, "function");
        signalNames.put(function, signalName);
    }

    /**
     * Alter which Target is being orbited.
     *
     * @param target the desired target (not null, alias created)
     */
    public void setTarget(Target target) {
        Validate.nonNull(target, "target");

        if (target != this.target) {
            this.target = target;
            logger.log(Level.INFO, "{0} is the new target.", target);

            tmpLocation.set(camera.getLocation());
            target.target(tmpCenter);
            preferredRange = tmpLocation.distance(tmpCenter);
        }
    }

    /**
     * Alter the analog input multiplier for focal zoom.
     *
     * @param multiplier the desired multipler (in log units per click,
     * default=0.2)
     */
    public void setZoomMultiplier(float multiplier) {
        zoomMultiplier = multiplier;
    }
    // *************************************************************************
    // ActionAppState methods

    /**
     * Clean up this state during the first update after it gets detached.
     * Should be invoked only by a subclass or by the AppStateManager.
     */
    @Override
    public void cleanup() {
        if (isEnabled()) {
            disable();
        }

        super.cleanup();
    }

    /**
     * Enable or disable the functionality of this state.
     *
     * @param newSetting true &rarr; enable, false &rarr; disable
     */
    @Override
    final public void setEnabled(boolean newSetting) {
        if (!isInitialized()) {
            throw new RuntimeException();
        }

        if (newSetting && !isEnabled()) {
            enable();
        } else if (!newSetting && isEnabled()) {
            disable();
        }
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
            disable();
            return;
        }
        PhysicsCollisionObject targetPco = target.getTargetPco();
        if (targetPco == null) {
            disable();
            return;
        }
        CollisionSpace collisionSpace = targetPco.getCollisionSpace();
        if (collisionSpace == null) {
            disable();
            return;
        }
        /*
         * Hide the cursor if dragging.
         */
        boolean cursorVisible = !isDragging();
        inputManager.setCursorVisible(cursorVisible);
        /*
         * Sum the active discrete inputs (signals).
         */
        int forwardSum = 0;
        int orbitUpSign = 0;
        int orbitCwSign = 0;
        int zoomSignalDirection = 0;

        for (OcFunction function : OcFunction.values()) {
            String signalName = signalNames.get(function);
            if (signalName != null && signals.test(signalName)) {
                switch (function) {
                    case Back:
                        --forwardSum;
                        break;

                    case DragToOrbit:
                    case Xray:
                        // do nothing
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
                        throw new RuntimeException(function.toString());
                }
            }
        }
        /*
         * Calculate the camera's offset from the center.
         */
        tmpLocation.set(camera.getLocation());
        target.target(tmpCenter);
        tmpLocation.subtract(tmpCenter, tmpOffset);

        float range = tmpOffset.length();
        assert range > 0f : range;

        if (orbitCwSign != 0 || orbitUpSign != 0) {
            float rootSumSquares = MyMath.hypotenuse(orbitCwSign, orbitUpSign);
            float dist = range * orbitRate * tpf / rootSumSquares;

            camera.getLeft(tmpLeft);
            assert tmpLeft.isUnitVector();
            MyVector3f.accumulateScaled(tmpOffset, tmpLeft, orbitCwSign * dist);

            camera.getUp(tmpUp);
            assert tmpUp.isUnitVector();
            MyVector3f.accumulateScaled(tmpOffset, tmpUp, orbitUpSign * dist);

            float factor = range / tmpOffset.length();
            tmpOffset.multLocal(factor);
            tmpCenter.add(tmpOffset, tmpLocation);
        }
        if (pitchAnalogSum != 0f || yawAnalogSum != 0f) {
            float multiplier = camera.getHeight() * frustumYTangent / 1024f;
            float pitchAngle = multiplier * pitchAnalogSum;
            float yawAngle = multiplier * yawAnalogSum;
            tmpRotation.fromAngles(pitchAngle, yawAngle, 0f);
            tmpRotation.mult(tmpOffset, tmpOffset);
            tmpCenter.add(tmpOffset, tmpLocation);

            pitchAnalogSum = 0f;
            yawAnalogSum = 0f;
        }
        /*
         * Avoid looking too near the preferred "up" direction or its opposite.
         */
        tmpOffset.mult(-1f, tmpLook);
        tmpLook.normalizeLocal();
        double dot = MyVector3f.dot(tmpLook, preferredUpDirection);
        if (Math.abs(dot) > maxAbsDot) {
            preferredUpDirection.mult((float) dot, tmpProj);
            tmpLook.subtract(tmpProj, tmpRej);
            double rejL2 = MyVector3f.lengthSquared(tmpRej);
            if (rejL2 > 0.0) { // not directly above or below
                double newDot = MyMath.clamp(dot, maxAbsDot);
                double projCoeff = newDot / dot;
                double rejCoeff = Math.sqrt((1.0 - newDot * newDot) / rejL2);
                tmpProj.mult((float) projCoeff, tmpLook);
                MyVector3f.accumulateScaled(tmpLook, tmpRej, (float) rejCoeff);
            } else {
                MyVector3f.generateBasis(tmpLook, tmpProj, tmpRej);
                tmpLook.set(tmpProj);
            }
        }
        assert tmpLook.isUnitVector() : tmpLook;
        camera.lookAtDirection(tmpLook, preferredUpDirection);

        String xraySignalName = signalNames.get(OcFunction.Xray);
        boolean xrayVision = xraySignalName != null
                && signals.test(xraySignalName);

        if (forwardSum != 0) {
            range *= FastMath.exp(-tpf * forwardSum); // TODO move rate?
            if (forwardSum > 0 || xrayVision) {
                preferredRange = range;
            }
        } else if (range < preferredRange) {
            range = preferredRange;
        }
        /*
         * Limit the range to reduce the risk of far-plane clipping.
         */
        float maxRange = 0.5f * camera.getFrustumFar();
        if (range > maxRange) {
            range = maxRange;
        }

        tmpLook.mult(-range, tmpOffset);
        tmpCenter.add(tmpOffset, tmpLocation);

        if (!xrayVision) {
            /*
             * Test the sightline for obstructions, from target to camera.
             */
            List<PhysicsRayTestResult> hits
                    = collisionSpace.rayTestRaw(tmpCenter, tmpLocation);

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
            tmpOffset.multLocal(minFraction);
            tmpCenter.add(tmpOffset, tmpLocation);
        }

        camera.setLocation(tmpLocation);
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

        switch (eventName) {
            case analogOrbitCcw:
                if (isDragging()) {
                    yawAnalogSum += reading;
                }
                break;

            case analogOrbitCw:
                if (isDragging()) {
                    yawAnalogSum -= reading;
                }
                break;

            case analogOrbitDown:
                if (isDragging()) {
                    pitchAnalogSum += reading;
                }
                break;

            case analogOrbitUp:
                if (isDragging()) {
                    pitchAnalogSum -= reading;
                }
                break;

            case analogZoomIn:
                zoomAnalogSum += reading;
                break;

            case analogZoomOut:
                zoomAnalogSum -= reading;
                break;

            default:
                throw new IllegalArgumentException(eventName);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Disable this camera controller. Assumes it is initialized and enabled.
     */
    private void disable() {
        assert isInitialized();
        assert isEnabled();
        /*
         * Configure the analog inputs.
         */
        inputManager.deleteMapping(analogOrbitCcw);
        inputManager.deleteMapping(analogOrbitCw);
        inputManager.deleteMapping(analogOrbitDown);
        inputManager.deleteMapping(analogOrbitUp);
        inputManager.deleteMapping(analogZoomIn);
        inputManager.deleteMapping(analogZoomOut);
        inputManager.removeListener(this);

        inputManager.setCursorVisible(true);

        super.setEnabled(false);
    }

    /**
     * Enable this camera controller. Assumes it is initialized and disabled.
     */
    private void enable() {
        assert isInitialized();
        assert !isEnabled();
        if (target == null) {
            throw new IllegalStateException("No target has been set.");
        }

        camera.setName("orbit camera");
        /*
         * Initialize the preferred range.
         */
        tmpLocation.set(camera.getLocation());
        target.target(tmpCenter);
        preferredRange = tmpLocation.distance(tmpCenter);

        float yDegrees;
        if (camera.isParallelProjection()) {
            /*
             * Configure perspective.
             */
            yDegrees = 30f;
        } else {
            yDegrees = MyCamera.yDegrees(camera);
        }
        float aspectRatio = MyCamera.viewAspectRatio(camera);
        float near = camera.getFrustumNear();
        float far = camera.getFrustumFar();
        camera.setFrustumPerspective(yDegrees, aspectRatio, near, far);
        /*
         * Configure the analog inputs.
         */
        inputManager.addMapping(analogOrbitCcw,
                new MouseAxisTrigger(MouseInput.AXIS_X, false));
        inputManager.addMapping(analogOrbitCw,
                new MouseAxisTrigger(MouseInput.AXIS_X, true));
        inputManager.addMapping(analogOrbitDown,
                new MouseAxisTrigger(MouseInput.AXIS_Y, true));
        inputManager.addMapping(analogOrbitUp,
                new MouseAxisTrigger(MouseInput.AXIS_Y, false));
        inputManager.addMapping(analogZoomIn,
                new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false));
        inputManager.addMapping(analogZoomOut,
                new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true));

        inputManager.addListener(this, analogOrbitCcw, analogOrbitCw,
                analogOrbitDown, analogOrbitUp, analogZoomIn, analogZoomOut);

        super.setEnabled(true);
    }

    /**
     * Test whether a dragging function is active.
     */
    private boolean isDragging() {
        String signalName = signalNames.get(OcFunction.DragToOrbit);
        if (signalName != null && signals.test(signalName)) {
            return true;
        } else {
            return false;
        }
    }
}
