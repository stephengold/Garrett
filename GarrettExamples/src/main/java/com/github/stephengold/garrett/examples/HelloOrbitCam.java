/*
 Copyright (c) 2022-2023, Stephen Gold

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

import com.github.stephengold.garrett.CameraSignal;
import com.github.stephengold.garrett.OrbitCamera;
import com.github.stephengold.garrett.Target;
import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import java.util.logging.Level;
import jme3utilities.Heart;
import jme3utilities.SignalTracker;

/**
 * A simple application, its camera controlled by OrbitCamera.
 * <p>
 * Collision objects are rendered entirely by debug visualization.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloOrbitCam extends SimpleApplication {
    // *************************************************************************
    // fields

    /**
     * count how many triggers have been added to the InputManager
     */
    private static int numTriggers = 0;
    /**
     * track which of the named input signals are active
     */
    final private static SignalTracker signalTracker = new SignalTracker();
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloOrbitCam application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        // Mute the chatty loggers in certain packages.
        Heart.setLoggingLevels(Level.WARNING);

        HelloOrbitCam application = new HelloOrbitCam();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        // Set up Bullet physics and create a physics space.
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        // Visualize what occurs in physics space.
        bulletAppState.setDebugEnabled(true);

        // Add lighting and shadows to the debug scene.
        SimpleApplication app = this;
        bulletAppState.setDebugInitListener((Node physicsDebugRootNode)
                -> DemoSpace.addLighting(app, physicsDebugRootNode)
        );
        bulletAppState
                .setDebugShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        // Populate the PhysicsSpace.
        DemoSpace.addGrayBox(this);
        PhysicsRigidBody redBall = DemoSpace.addRedBall(this);
        DemoSpace.addTiledPlane(this);
        DemoSpace.addYellowPyramid(this);

        // Disable JMonkeyEngine's FlyByCamera, which would otherwise interfere.
        flyCam.setEnabled(false);

        // Instantiate a target for orbiting the red ball.
        Target target = new Target() {
            @Override
            public Vector3f forwardDirection(Vector3f storeResult) {
                throw new UnsupportedOperationException("not chasable");
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

        // Instantiate the camera controller.
        OrbitCamera orbitCam = new OrbitCamera("OrbitCam", cam, signalTracker);
        orbitCam.setPoleExclusionAngles(1.5f, 0.2f); // default=0.3, 0.3
        orbitCam.setTarget(target);

        // Name some camera-input signals.
        orbitCam.setSignalName(CameraSignal.Back, "cameraBackward");
        orbitCam.setSignalName(CameraSignal.DragToOrbit, "cameraDrag");
        orbitCam.setSignalName(CameraSignal.Forward, "cameraForward");
        orbitCam.setSignalName(CameraSignal.OrbitCcw, "cameraOrbitCcw");
        orbitCam.setSignalName(CameraSignal.OrbitCw, "cameraOrbitCw");
        orbitCam.setSignalName(CameraSignal.OrbitDown, "cameraOrbitDown");
        orbitCam.setSignalName(CameraSignal.OrbitUp, "cameraOrbitUp");
        orbitCam.setSignalName(CameraSignal.Xray, "cameraXray");
        orbitCam.setSignalName(CameraSignal.ZoomIn, "cameraZoomIn");
        orbitCam.setSignalName(CameraSignal.ZoomOut, "cameraZoomOut");
        /*
         * Map keyboard keys and mouse buttons
         * to the named camera-input signals.
         */
        mapKeyToSignal(KeyInput.KEY_S, "cameraBackward");
        mapButtonToSignal(MouseInput.BUTTON_LEFT, "cameraDrag");
        mapKeyToSignal(KeyInput.KEY_W, "cameraForward");
        mapKeyToSignal(KeyInput.KEY_A, "cameraOrbitCcw");
        mapKeyToSignal(KeyInput.KEY_LEFT, "cameraOrbitCcw");
        mapKeyToSignal(KeyInput.KEY_D, "cameraOrbitCw");
        mapKeyToSignal(KeyInput.KEY_RIGHT, "cameraOrbitCw");
        mapKeyToSignal(KeyInput.KEY_Z, "cameraOrbitDown");
        mapKeyToSignal(KeyInput.KEY_Q, "cameraOrbitUp");
        mapKeyToSignal(KeyInput.KEY_X, "cameraXray");
        mapKeyToSignal(KeyInput.KEY_UP, "cameraZoomIn");
        mapKeyToSignal(KeyInput.KEY_EQUALS, "cameraZoomIn");
        mapKeyToSignal(KeyInput.KEY_ADD, "cameraZoomIn");
        mapKeyToSignal(KeyInput.KEY_DOWN, "cameraZoomOut");
        mapKeyToSignal(KeyInput.KEY_MINUS, "cameraZoomOut");
        mapKeyToSignal(KeyInput.KEY_SUBTRACT, "cameraZoomOut");

        // Attach and enable the camera controller.
        boolean success = stateManager.attach(orbitCam);
        assert success;
        orbitCam.setEnabled(true);
    }
    // *************************************************************************
    // private methods

    /**
     * Add an input mapping that causes the SignalTracker to track the specified
     * mouse button.
     *
     * @param buttonId the mouse button to be tracked
     * @param signalName name for the input signal (not null) mappings to the
     * same signal
     */
    private void mapButtonToSignal(int buttonId, String signalName) {
        signalTracker.add(signalName);

        int sourceIndex = numTriggers;
        ActionListener actionListener = (action, keyPressed, tpf)
                -> signalTracker.setActive(signalName, sourceIndex, keyPressed);
        String action = "signal " + signalName;
        inputManager.addListener(actionListener, action);

        MouseButtonTrigger trigger = new MouseButtonTrigger(buttonId);
        inputManager.addMapping(action, trigger);
        ++numTriggers;
    }

    /**
     * Add an input mapping that causes the SignalTracker to track the specified
     * keyboard key.
     *
     * @param keyId the keyboard key to be tracked
     * @param signalName name for the input signal (not null)
     */
    private void mapKeyToSignal(int keyId, String signalName) {
        signalTracker.add(signalName);

        int sourceIndex = numTriggers;
        ActionListener actionListener = (action, keyPressed, tpf)
                -> signalTracker.setActive(signalName, sourceIndex, keyPressed);
        String action = "signal " + signalName;
        inputManager.addListener(actionListener, action);

        KeyTrigger trigger = new KeyTrigger(keyId);
        inputManager.addMapping(action, trigger);
        ++numTriggers;
    }
}
