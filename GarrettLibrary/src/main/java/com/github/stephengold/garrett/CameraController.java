/*
 Copyright (c) 2020-2022, Stephen Gold

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
import com.jme3.input.controls.AnalogListener;
import com.jme3.math.FastMath;
import com.jme3.renderer.Camera;
import java.util.EnumMap;
import java.util.logging.Logger;
import jme3utilities.MyCamera;
import jme3utilities.SignalTracker;
import jme3utilities.Validate;

/**
 * An AppState to control a Camera.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract class CameraController
        extends BaseAppState
        implements AnalogListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger0
            = Logger.getLogger(CameraController.class.getName());
    /**
     * name of analog event for zooming in
     */
    final protected static String analogZoomIn = "zoom in";
    /**
     * name of analog event for zooming out
     */
    final protected static String analogZoomOut = "zoom out";
    // *************************************************************************
    // fields

    /**
     * Camera being controlled (not null)
     */
    final private Camera camera;
    /**
     * map modal functions to their default states (true&rarr;active,
     * false&rarr;inactive)
     */
    final private EnumMap<CameraSignal, Boolean> defaultStates
            = new EnumMap<>(CameraSignal.class);
    /**
     * map functions to signal names
     */
    final private EnumMap<CameraSignal, String> signalNames
            = new EnumMap<>(CameraSignal.class);
    /**
     * frustum's Y tangent ratio at lowest magnification (&gt;minYTangent)
     */
    private float maxYTangent = 2f;
    /**
     * frustum's Y tangent ratio at highest magnification (&gt;0)
     */
    private float minYTangent = 0.01f;
    /**
     * accumulated analog zoom amount since the last update (in clicks)
     */
    private float zoomAnalogSum = 0f;
    /**
     * analog zoom input multiplier (in log units per click)
     */
    private float zoomMultiplier = 0.3f;
    /**
     * status of named signals
     */
    final private SignalTracker signalTracker;
    /**
     * name applied to the Camera when this controller becomes attached and
     * enabled
     */
    private String cameraName;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a disabled AppState that will control the specified Camera.
     *
     * @param id the desired unique ID for this AppState
     * @param camera the Camera to control (not null, alias created)
     * @param tracker the status tracker for named signals (not null, alias
     * created)
     */
    protected CameraController(
            String id, Camera camera, SignalTracker tracker) {
        super(id);
        Validate.nonNull(camera, "camera");
        Validate.nonNull(tracker, "tracker");

        this.camera = camera;
        this.signalTracker = tracker;
        setCameraName(id);
        super.setEnabled(false);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the name applied to the Camera when this controller becomes
     * attached and enabled.
     *
     * @return the name
     */
    public String cameraName() {
        return cameraName;
    }

    /**
     * Return the default state of the specified modal function.
     *
     * @param function which function to query (not null)
     * @return true for active, otherwise false
     */
    public boolean defaultState(CameraSignal function) {
        Validate.nonNull(function, "function");

        Boolean defaultState = defaultStates.get(function);
        if (defaultState == null) {
            return false;
        } else {
            return defaultState;
        }
    }

    /**
     * Access the camera that's being controlled.
     *
     * @return the pre-existing instance (not null)
     */
    public Camera getCamera() {
        assert camera != null;
        return camera;
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
     * Return the maximum magnification.
     *
     * @return the magnification (&gt;0)
     */
    public float maxMagnification() {
        assert minYTangent > 0f : minYTangent;

        float result = 1f / minYTangent;
        return result;
    }

    /**
     * Return the minimum magnification.
     *
     * @return the magnification (&gt;0)
     */
    public float minMagnification() {
        assert maxYTangent > 0f : maxYTangent;

        float result = 1f / maxYTangent;
        return result;
    }

    /**
     * Alter the name applied to the Camera when this controller becomes
     * attached and enabled.
     * <p>
     * Allowed only when the controller is NOT attached and enabled.
     *
     * @param name the desired name (default=appstate ID)
     */
    public void setCameraName(String name) {
        if (isInitialized() && isEnabled()) {
            throw new IllegalStateException("Cannot alter the camera name "
                    + "while the controller is attached and enabled.");
        }
        this.cameraName = name;
    }

    /**
     * Alter the default state of the specified modal function.
     *
     * @param function which function to alter (not null)
     * @param state the desired default state (true&rarr;active,
     * false&rarr;inactive, default=false)
     */
    public void setDefaultState(CameraSignal function, boolean state) {
        Validate.nonNull(function, "function");
        defaultStates.put(function, state);
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
     * Return the signal name assigned to the specified function.
     *
     * @param function which function to read (not null)
     * @return the signal name, or null if none set for that function
     */
    public String signalName(CameraSignal function) {
        Validate.nonNull(function, "function");
        String result = signalNames.get(function);
        return result;
    }

    /**
     * Return the analog input multiplier for focal zoom.
     *
     * @return the multiplier (in log units per click)
     */
    public float zoomMultiplier() {
        assert zoomMultiplier > 0f : zoomMultiplier;
        return zoomMultiplier;
    }
    // *************************************************************************
    // protected methods

    /**
     * Apply focal zoom, if any: first the discrete signals and then the analog
     * values.
     *
     * @param zoomSignalDirection the zoom direction requested by discrete
     * signals (+1 to zoom in, -1 to zoom out)
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    protected void applyFocalZoom(int zoomSignalDirection, float tpf) {
        if (zoomSignalDirection != 0) {
            float zoomFactor = FastMath.exp(zoomSignalDirection * tpf);
            magnify(zoomFactor);
        }

        if (zoomAnalogSum != 0f) {
            float logFactor = zoomMultiplier() * zoomAnalogSum;
            float zoomFactor = FastMath.exp(logFactor);
            magnify(zoomFactor);
            this.zoomAnalogSum = 0f;
        }
    }

    /**
     * Test whether the specified camera function is active.
     *
     * @param function which function to test (not null)
     * @return true if active, otherwise false
     */
    protected boolean isActive(CameraSignal function) {
        assert function != null;

        boolean result = defaultState(function);
        String signalName = signalNames.get(function);
        if (signalName != null && signalTracker.test(signalName)) {
            result = !result;
        }

        return result;
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
     * Callback invoked whenever this AppState becomes both attached and
     * enabled.
     */
    @Override
    protected void onEnable() {
        assert isInitialized();
        camera.setName(cameraName);
    }
}
