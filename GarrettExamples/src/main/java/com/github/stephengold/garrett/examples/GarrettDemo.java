/*
 Copyright (c) 2022-2023, Stephen Gold
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
package com.github.stephengold.garrett.examples;

import com.github.stephengold.garrett.AffixedCamera;
import com.github.stephengold.garrett.CameraSignal;
import com.github.stephengold.garrett.DynamicCamera;
import com.github.stephengold.garrett.ObstructionResponse;
import com.github.stephengold.garrett.OrbitCamera;
import com.github.stephengold.garrett.Target;
import com.jme3.app.Application;
import com.jme3.app.StatsAppState;
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.font.Rectangle;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.system.AppSettings;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Heart;
import jme3utilities.MyCamera;
import jme3utilities.MyString;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.ui.AcorusDemo;
import jme3utilities.ui.InputMode;
import jme3utilities.ui.Overlay;
import jme3utilities.ui.Signals;

/**
 * Test/demonstrate switching between 5 camera controllers, all sharing a common
 * InputMode.
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class GarrettDemo extends AcorusDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(GarrettDemo.class.getName());
    /**
     * application name (for the title bar of the app's window)
     */
    final private static String applicationName
            = GarrettDemo.class.getSimpleName();
    /**
     * action prefix to switch to the camera controller
     */
    final private static String apSelectCamera = "select camera ";
    /**
     * some action strings that onAction() recognizes
     */
    final private static String asDumpSpace = "dump space";
    final private static String asDumpViewPort = "dump viewPort";
    final private static String asNextResponse = "next response";
    /**
     * camera names
     */
    final private static String cnAffixed = "AffixedCam";
    final private static String cnChase = "ChaseCam";
    final private static String cnDynamic = "DynaCam";
    final private static String cnOrbit = "OrbitCam";
    final private static String cnWatch = "WatchCam";
    /**
     * some signal names
     */
    final private static String signalDrag = "drag";
    final private static String signalPtl = "point to look";
    final private static String signalRam = "ram";
    final private static String signalViewDown = "view down";
    final private static String signalViewUp = "view up";
    final private static String signalXray = "xray";
    final private static String signalZoomIn = "zoom in";
    final private static String signalZoomOut = "zoom out";
    // *************************************************************************
    // fields

    /**
     * continuously display information about the active camera controller
     */
    private static AppState activeCameraController;
    /**
     * map camera names to camera controllers
     */
    final private static Map<String, AppState> cameraControllers
            = new HashMap<>(6);
    /**
     * display information about the camera controller
     */
    private static Overlay overlay;
    /**
     * dump debugging information to {@code System.out}
     */
    final private static PhysicsDumper dumper = new PhysicsDumper();
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the GarrettDemo application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        String title = applicationName + " " + MyString.join(arguments);

        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        boolean loadDefaults = true;
        AppSettings settings = new AppSettings(loadDefaults);
        settings.setAudioRenderer(null);
        settings.setResizable(true);
        settings.setSamples(4); // anti-aliasing
        settings.setTitle(title); // Customize the window's title bar.

        Application application = new GarrettDemo();
        application.setSettings(settings);
        application.start();
    }
    // *************************************************************************
    // AcorusDemo methods

    /**
     * Initialize this application.
     */
    @Override
    public void acorusInit() {
        // Disable JMonkeyEngine's FlyByCamera, which would otherwise interfere.
        flyCam.setEnabled(false);

        configureDumper();
        configurePhysics();

        float ramMass = 100f;
        float usualMass = 0.1f;
        PhysicsSpace physicsSpace = DemoSpace.getPhysicsSpace(this);
        Signals signals = getSignals();

        // Instantiate and attach the "Affixed" camera controller.
        AffixedCamera affixedCam = new AffixedCamera(cnAffixed, cam, signals);
        affixedCam.setLookDirection(new Vector3f(0f, 0f, -1f));
        affixedCam.setOffset(new Vector3f(0f, 1f, 5f));
        affixedCam.setSignalName(CameraSignal.ZoomIn, signalZoomIn);
        affixedCam.setSignalName(CameraSignal.ZoomOut, signalZoomOut);
        boolean success = stateManager.attach(affixedCam);
        assert success;
        cameraControllers.put(cnAffixed, affixedCam);

        // Instantiate and attach the "Chase" camera controller.
        OrbitCamera chaseCam = new OrbitCamera(cnChase, cam, signals);
        chaseCam.setAzimuthTau(0.2f);
        setOrbitSignals(chaseCam);
        success = stateManager.attach(chaseCam);
        assert success;
        cameraControllers.put(cnChase, chaseCam);

        // Instantiate and attach the "Dynamic" camera controller.
        DynamicCamera dynaCam = new DynamicCamera(
                cnDynamic, cam, physicsSpace, signals, usualMass, ramMass);
        dynaCam.setDefaultState(CameraSignal.PointToLook, true);
        dynaCam.setMoveSpeed(6f); // default=1
        dynaCam.setPoleExclusionAngle(1.2f); // default=0.3
        dynaCam.setPtlTurnRate(2f); // default=1
        setDynamicSignals(dynaCam);
        success = stateManager.attach(dynaCam);
        assert success;
        cameraControllers.put(cnDynamic, dynaCam);

        // Instantiate and attach the "Orbit" camera controller.
        OrbitCamera orbitCam = new OrbitCamera(cnOrbit, cam, signals);
        setOrbitSignals(orbitCam);
        success = stateManager.attach(orbitCam);
        assert success;
        cameraControllers.put(cnOrbit, orbitCam);

        // Instantiate and attach the "Watch" camera controller.
        DynamicCamera watchCam = new DynamicCamera(
                cnWatch, cam, physicsSpace, signals, usualMass, ramMass);
        watchCam.setMoveSpeed(6f); // default=1
        setDynamicSignals(watchCam);
        success = stateManager.attach(watchCam);
        assert success;
        cameraControllers.put(cnWatch, watchCam);

        // Continuously display information about the active camera controller.
        float width = 214f; // pixels
        int numLines = 1;
        overlay = new Overlay("Overlay", width, numLines) {
            @Override
            public void update(float tpf) {
                super.update(tpf);
                String text = activeCameraController.getId();
                if (text == null) {
                    text = "";
                }
                if (activeCameraController instanceof OrbitCamera) {
                    OrbitCamera oc = (OrbitCamera) activeCameraController;
                    text += " with " + oc.getObstructionResponse();
                }
                int lineNumber = 0;
                setText(lineNumber, text);
            }
        };
        overlay.setBackgroundColor(ColorRGBA.Blue);
        overlay.setEnabled(true);
        success = stateManager.attach(overlay);
        assert success;

        super.acorusInit();

        // Hide the render-statistics overlay.
        stateManager.getState(StatsAppState.class).toggleStats();

        restartScenario();
    }

    /**
     * Calculate screen bounds for the detailed help node.
     *
     * @param viewPortWidth (in pixels, &gt;0)
     * @param viewPortHeight (in pixels, &gt;0)
     * @return a new instance
     */
    @Override
    public Rectangle detailedHelpBounds(int viewPortWidth, int viewPortHeight) {
        // Position help nodes below the status.
        float margin = 10f; // in pixels
        float leftX = margin;
        float topY = viewPortHeight - 40f - margin;
        float width = 260f;
        float height = topY - margin;
        Rectangle result = new Rectangle(leftX, topY, width, height);

        return result;
    }

    /**
     * Add application-specific hotkey bindings (and override existing ones, if
     * necessary).
     */
    @Override
    public void moreDefaultBindings() {
        InputMode dim = getDefaultInputMode();

        //dim.bind(asCollectGarbage, KeyInput.KEY_G);
        dim.bind(asDumpSpace, KeyInput.KEY_O);
        dim.bind(asDumpViewPort, KeyInput.KEY_P);
        dim.bind(asNextResponse, KeyInput.KEY_N, KeyInput.KEY_SPACE);
        dim.bind(asToggleHelp, KeyInput.KEY_H);

        // Bind the 1-through-5 keys to select the camera controller.
        dim.bind(apSelectCamera + cnAffixed,
                KeyInput.KEY_1, KeyInput.KEY_NUMPAD1);
        dim.bind(apSelectCamera + cnChase,
                KeyInput.KEY_2, KeyInput.KEY_NUMPAD2);
        dim.bind(apSelectCamera + cnDynamic,
                KeyInput.KEY_3, KeyInput.KEY_NUMPAD3);
        dim.bind(apSelectCamera + cnOrbit,
                KeyInput.KEY_4, KeyInput.KEY_NUMPAD4);
        dim.bind(apSelectCamera + cnWatch,
                KeyInput.KEY_5, KeyInput.KEY_NUMPAD5);

        dim.bindSignal(CameraInput.FLYCAM_BACKWARD, KeyInput.KEY_S);
        dim.bindSignal(CameraInput.FLYCAM_FORWARD, KeyInput.KEY_W);
        dim.bindSignal(CameraInput.FLYCAM_LOWER, KeyInput.KEY_Z);
        dim.bindSignal(CameraInput.FLYCAM_RISE, KeyInput.KEY_Q);
        dim.bindSignal(CameraInput.FLYCAM_STRAFELEFT,
                KeyInput.KEY_A, KeyInput.KEY_LEFT);
        dim.bindSignal(CameraInput.FLYCAM_STRAFERIGHT,
                KeyInput.KEY_D, KeyInput.KEY_RIGHT);
        dim.bindSignal(signalViewDown, KeyInput.KEY_DOWN);
        dim.bindSignal(signalViewUp, KeyInput.KEY_UP);

        dim.bind(InputMode.signalActionPrefix + signalDrag, "MMB");
        dim.bindSignal(signalPtl, KeyInput.KEY_LSHIFT);
        dim.bindSignal(signalRam, KeyInput.KEY_R);
        dim.bindSignal(signalXray, KeyInput.KEY_X);
        dim.bindSignal(
                signalZoomIn, KeyInput.KEY_ADD, KeyInput.KEY_EQUALS);
        dim.bindSignal(
                signalZoomOut, KeyInput.KEY_MINUS, KeyInput.KEY_SUBTRACT);
    }

    /**
     * Process an action that wasn't handled by the active InputMode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case asDumpSpace:
                    PhysicsSpace physicsSpace = DemoSpace.getPhysicsSpace(this);
                    dumper.dump(physicsSpace);
                    return;

                case asDumpViewPort:
                    dumper.dump(viewPort);
                    return;

                case asNextResponse:
                    nextObstructionResponse();
                    return;

                default:
                    if (actionString.startsWith(apSelectCamera)) {
                        String cameraName = MyString
                                .remainder(actionString, apSelectCamera);
                        setCameraController(cameraName);
                        return;
                    }
            }
        }

        // The action is not handled here: forward it to the superclass.
        super.onAction(actionString, ongoing, tpf);
    }

    /**
     * Update the GUI layout and proposed settings after a resize.
     *
     * @param newWidth the new width of the framebuffer (in pixels, &gt;0)
     * @param newHeight the new height of the framebuffer (in pixels, &gt;0)
     */
    @Override
    public void onViewPortResize(int newWidth, int newHeight) {
        overlay.onViewPortResize(newWidth, newHeight);
        super.onViewPortResize(newWidth, newHeight);
    }
    // *************************************************************************
    // private methods

    /**
     * Configure the PhysicsDumper. Invoked during startup.
     */
    private void configureDumper() {
        dumper.setEnabled(DumpFlags.JointsInBodies, true);
        dumper.setEnabled(DumpFlags.ShadowModes, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        // Set up Bullet physics and create a physics space.
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.getPhysicsSpace();

        // Visualize what occurs in physics space.
        bulletAppState.setDebugEnabled(true);

        // Add lighting and shadows to the debug scene.
        AcorusDemo app = this;
        bulletAppState.setDebugInitListener((Node physicsDebugRootNode)
                -> DemoSpace.addLighting(app, physicsDebugRootNode)
        );
    }

    private void nextObstructionResponse() {
        if (activeCameraController instanceof OrbitCamera) {
            OrbitCamera oc = (OrbitCamera) activeCameraController;
            ObstructionResponse current = oc.getObstructionResponse();
            ObstructionResponse next;
            switch (current) {
                case Clip:
                    next = ObstructionResponse.WarpBias;
                    break;
                case WarpBias:
                    next = ObstructionResponse.WarpNoBias;
                    break;
                case WarpNoBias:
                    next = ObstructionResponse.XRay;
                    break;
                case XRay:
                    next = ObstructionResponse.Clip;
                    break;
                default:
                    throw new IllegalStateException("current = " + current);
            }
            oc.setObstructionResponse(next);
        }
    }

    /**
     * Reset the default camera during a restart.
     */
    private void resetCamera() {
        float near = 0.1f;
        float far = 500f;
        MyCamera.setNearFar(cam, near, far);
        MyCamera.setYTangent(cam, 1f);

        cam.setLocation(new Vector3f(0f, 0f, 10f));
        cam.setRotation(new Quaternion(0f, 1f, 0f, 0f));

        setCameraController(cnWatch);
    }

    /**
     * Restart the scenario.
     */
    private void restartScenario() {
        resetCamera();

        PhysicsSpace physicsSpace = DemoSpace.getPhysicsSpace(this);
        physicsSpace.destroy();
        assert physicsSpace.isEmpty();
        int numTickListeners = physicsSpace.countTickListeners();
        assert numTickListeners == 1 : numTickListeners;

        // Populate the PhysicsSpace.
        PhysicsRigidBody doorBody = DemoSpace.addBlueDoor(this);
        PhysicsRigidBody doorFrameBody = DemoSpace.addDoorframe(this);
        DemoSpace.addGrayBox(this);
        PhysicsRigidBody redBall = DemoSpace.addRedBall(this);
        DemoSpace.addTiledPlane(this);
        DemoSpace.addYellowPyramid(this);

        // Disable collisions between the door and the door frame.
        doorBody.addToIgnoreList(doorFrameBody);

        AffixedCamera affixedCam
                = (AffixedCamera) cameraControllers.get(cnAffixed);
        affixedCam.setRigidBody(redBall);

        updateTargets(redBall);
    }

    /**
     * Switch to the named CameraController.
     *
     * @param controllerName the name of the desired controller (not null, not
     * empty)
     */
    private void setCameraController(String controllerName) {
        AppState cc = cameraControllers.get(controllerName);
        if (cc == null || cc == activeCameraController) {
            return;
        }

        if (activeCameraController != null) {
            assert activeCameraController.isEnabled();
            activeCameraController.setEnabled(false);
        }

        cc.setEnabled(true);
        activeCameraController = cc;
    }

    /**
     * Configure the signals assigned to various DynamicCamera functions.
     *
     * @param dc the controller to configure (not null)
     */
    private static void setDynamicSignals(DynamicCamera dc) {
        dc.setSignalName(CameraSignal.Ghost, signalXray);
        dc.setSignalName(CameraSignal.PointToLook, signalPtl);
        dc.setSignalName(CameraSignal.Ram, signalRam);
        dc.setSignalName(CameraSignal.ViewDown, signalViewDown);
        dc.setSignalName(CameraSignal.ViewUp, signalViewUp);
        dc.setSignalName(CameraSignal.ZoomIn, signalZoomIn);
        dc.setSignalName(CameraSignal.ZoomOut, signalZoomOut);
    }

    /**
     * Configure the signals assigned to various OrbitCamera functions.
     *
     * @param oc the controller to configure (not null)
     */
    private static void setOrbitSignals(OrbitCamera oc) {
        oc.setSignalName(CameraSignal.Back, CameraInput.FLYCAM_BACKWARD);
        oc.setSignalName(CameraSignal.DragToOrbit, signalDrag);
        oc.setSignalName(CameraSignal.Forward, CameraInput.FLYCAM_FORWARD);
        oc.setSignalName(CameraSignal.OrbitCcw, CameraInput.FLYCAM_STRAFERIGHT);
        oc.setSignalName(CameraSignal.OrbitCw, CameraInput.FLYCAM_STRAFELEFT);
        oc.setSignalName(CameraSignal.OrbitDown, CameraInput.FLYCAM_LOWER);
        oc.setSignalName(CameraSignal.OrbitUp, CameraInput.FLYCAM_RISE);
        oc.setSignalName(CameraSignal.Xray, signalXray);
        oc.setSignalName(CameraSignal.ZoomIn, signalZoomIn);
        oc.setSignalName(CameraSignal.ZoomOut, signalZoomOut);
    }

    private void updateTargets(PhysicsRigidBody redBall) {
        Target redBallTarget = new Target() {
            @Override
            public Vector3f forwardDirection(Vector3f storeResult) {
                Vector3f result = redBall.getPhysicsLocation(storeResult);
                result.y = 0f;
                MyVector3f.normalizeLocal(result);
                float saveX = result.x;
                result.x = result.z;
                result.z = -saveX;
                return result;
            }

            @Override
            public PhysicsCollisionObject getTargetPco() {
                return redBall;
            }

            @Override
            public Vector3f locateTarget(Vector3f storeResult) {
                Vector3f result = redBall.getPhysicsLocation(storeResult);
                return result;
            }
        };

        OrbitCamera chaseCam = (OrbitCamera) cameraControllers.get(cnChase);
        chaseCam.setTarget(redBallTarget);

        OrbitCamera orbitCam = (OrbitCamera) cameraControllers.get(cnOrbit);
        orbitCam.setTarget(redBallTarget);

        DynamicCamera watchCam = (DynamicCamera) cameraControllers.get(cnWatch);
        watchCam.setTarget(redBallTarget);
    }
}
