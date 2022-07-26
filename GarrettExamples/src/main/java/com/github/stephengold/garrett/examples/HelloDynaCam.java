/*
 Copyright (c) 2022, Stephen Gold
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

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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
import com.github.stephengold.garrett.DynamicCamera;
import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.scene.Node;
import jme3utilities.SignalTracker;

/**
 * A simple application, its camera controlled by DynamicCamera.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloDynaCam extends SimpleApplication {
    // *************************************************************************
    // fields

    /**
     * count how many triggers have been added to the InputManager
     */
    private int numTriggers = 0;
    /**
     * track which of the named input signals are active
     */
    final private SignalTracker signalTracker = new SignalTracker();
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HelloDynaCam application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloDynaCam application = new HelloDynaCam();
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
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();

        // Visualize what occurs in physics space.
        bulletAppState.setDebugEnabled(true);

        // Add lighting and shadows to the debug scene.
        SimpleApplication app = this;
        bulletAppState.setDebugInitListener((Node physicsDebugRootNode) -> {
            DemoSpace.addLighting(app, physicsDebugRootNode);
        });
//        bulletAppState.setDebugShadowMode(
//                RenderQueue.ShadowMode.CastAndReceive);

        // Populate the PhysicsSpace.
        PhysicsRigidBody doorBody = DemoSpace.addBlueDoor(this);
        PhysicsRigidBody doorFrameBody = DemoSpace.addDoorframe(this);
        DemoSpace.addGrayBox(this);
        DemoSpace.addTiledPlane(this);
        DemoSpace.addYellowPyramid(this);

        // Disable collisions between the door and the door frame.
        doorBody.addToIgnoreList(doorFrameBody);

        // Disable JMonkeyEngine's FlyByCamera, which would otherwise interfere.
        flyCam.setEnabled(false);
        /*
         * Map keyboard keys and mouse buttons to the camera-input signals
         * that will be named automatically
         * when the camera controller is instantiated.
         */
        mapKeyToSignal(KeyInput.KEY_W, CameraInput.FLYCAM_FORWARD);
        mapKeyToSignal(KeyInput.KEY_A, CameraInput.FLYCAM_STRAFELEFT);
        mapKeyToSignal(KeyInput.KEY_D, CameraInput.FLYCAM_STRAFERIGHT);
        mapKeyToSignal(KeyInput.KEY_S, CameraInput.FLYCAM_BACKWARD);
        mapKeyToSignal(KeyInput.KEY_Q, CameraInput.FLYCAM_RISE);
        mapKeyToSignal(KeyInput.KEY_Z, CameraInput.FLYCAM_LOWER);
        mapKeyToSignal(KeyInput.KEY_LEFT, CameraInput.FLYCAM_STRAFELEFT);
        mapKeyToSignal(KeyInput.KEY_RIGHT, CameraInput.FLYCAM_STRAFERIGHT);

        // Instantiate the camera controller.
        float usualMass = 0.1f;
        float ramMass = 100f;
        DynamicCamera dyna = new DynamicCamera(
                "dyna", cam, physicsSpace, signalTracker, usualMass, ramMass);
        dyna.setDefaultState(CameraSignal.PointToLook, true);
        dyna.setMoveSpeed(6f);
        dyna.setPtlTurnRate(4f);

        // Add some camera-input signals that aren't automatically named.
        dyna.setSignalName(CameraSignal.Ghost, "ghost");
        dyna.setSignalName(CameraSignal.PointToLook, "pointToLook");
        dyna.setSignalName(CameraSignal.Ram, "ram");
        dyna.setSignalName(CameraSignal.ViewDown, "viewDown");
        dyna.setSignalName(CameraSignal.ViewUp, "viewUp");
        dyna.setSignalName(CameraSignal.ZoomIn, "cameraZoomIn");
        dyna.setSignalName(CameraSignal.ZoomOut, "cameraZoomOut");
        /*
         * Map keyboard keys and mouse buttons
         * to the added camera-input signals.
         */
        mapKeyToSignal(KeyInput.KEY_G, "ghost");
        mapKeyToSignal(KeyInput.KEY_LSHIFT, "pointToLook");
        mapKeyToSignal(KeyInput.KEY_R, "ram");
        mapKeyToSignal(KeyInput.KEY_DOWN, "viewDown");
        mapKeyToSignal(KeyInput.KEY_UP, "viewUp");
        mapKeyToSignal(KeyInput.KEY_EQUALS, "cameraZoomIn");
        mapKeyToSignal(KeyInput.KEY_ADD, "cameraZoomIn");
        mapKeyToSignal(KeyInput.KEY_MINUS, "cameraZoomOut");
        mapKeyToSignal(KeyInput.KEY_SUBTRACT, "cameraZoomOut");

        // Attach and enable the camera controller.
        boolean success = stateManager.attach(dyna);
        assert success;
        dyna.setEnabled(true);
    }
    // *************************************************************************
    // private methods

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
        ActionListener actionListener = (action, keyPressed, tpf) -> {
            signalTracker.setActive(signalName, sourceIndex, keyPressed);
        };
        String action = "signal " + signalName;
        inputManager.addListener(actionListener, action);

        KeyTrigger trigger = new KeyTrigger(keyId);
        inputManager.addMapping(action, trigger);
        ++numTriggers;
    }
}
