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

import com.github.stephengold.garrett.AffixedCamera;
import com.github.stephengold.garrett.CameraSignal;
import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.math.Vector3f;
import jme3utilities.SignalTracker;

/**
 * A simple application, its camera controlled by AffixedCamera.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HelloGarrett extends SimpleApplication {
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
     * Main entry point for the HelloGarrett application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        HelloGarrett application = new HelloGarrett();
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

        // Populate the PhysicsSpace.
        DemoSpace.addGrayBox(this);
        PhysicsRigidBody redBall = DemoSpace.addRedBall(this);
        DemoSpace.addTiledPlane(this);
        DemoSpace.addYellowPyramid(this);

        // Disable JMonkeyEngine's FlyByCamera, which would otherwise interfere.
        flyCam.setEnabled(false);

        // Instantiate the camera controller.
        AffixedCamera fix = new AffixedCamera("fix", cam, signalTracker);
        fix.setLookDirection(new Vector3f(0f, 0f, -1f));
        fix.setOffset(new Vector3f(0f, 1f, 5f));
        fix.setRigidBody(redBall);

        // Name some camera-input signals.
        fix.setSignalName(CameraSignal.ZoomIn, "cameraIn");
        fix.setSignalName(CameraSignal.ZoomOut, "cameraOut");

        // Map keyboard keys to the named camera-input signals.
        mapKeyToSignal(KeyInput.KEY_DOWN, "cameraIn");
        mapKeyToSignal(KeyInput.KEY_UP, "cameraOut");

        // Attach and enable the camera controller.
        boolean success = stateManager.attach(fix);
        assert success;
        fix.setEnabled(true);
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