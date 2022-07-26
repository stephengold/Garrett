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

import com.jme3.app.Application;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.asset.TextureKey;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.bullet.util.PlaneDmiListener;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Limits;
import com.jme3.renderer.Renderer;
import com.jme3.texture.Texture;
import java.util.EnumMap;
import java.util.logging.Logger;

/**
 * A utility class to populate a PhysicsSpace for demos.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class DemoSpace {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DemoSpace.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private DemoSpace() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Create and configure a gray, static box and add it at the origin of the
     * physics space.
     *
     * @param app the application (not null)
     * @return the new instance
     */
    public static PhysicsRigidBody addGrayBox(Application app) {
        CollisionShape boxShape = new BoxCollisionShape(1f, 2.25f, 0.25f);
        PhysicsRigidBody result
                = new PhysicsRigidBody(boxShape, PhysicsBody.massForStatic);

        // Configure the debug visualization.
        AssetManager assetManager = app.getAssetManager();
        Material boxMaterial = new Material(assetManager, Materials.UNSHADED);
        boxMaterial.setColor("Color", new ColorRGBA(0.1f, 0.1f, 0.1f, 1f));
        result.setDebugMaterial(boxMaterial);

        PhysicsSpace physicsSpace = getPhysicsSpace(app);
        physicsSpace.addCollisionObject(result);

        return result;
    }

    /**
     * Create and configure a red, kinematic ball, add it to the physics space,
     * and cause it to orbit the origin.
     *
     * @param app the application (not null)
     * @return the new instance
     */
    public static PhysicsRigidBody addRedBall(Application app) {
        float ballRadius = 0.5f;
        CollisionShape ballShape = new SphereCollisionShape(ballRadius);
        PhysicsRigidBody result = new PhysicsRigidBody(ballShape);
        result.setKinematic(true);
        result.setPhysicsLocation(new Vector3f(0f, 0f, 3f));

        // Configure the debug visualization.
        AssetManager assetManager = app.getAssetManager();
        Material ballMaterial = new Material(assetManager, Materials.UNSHADED);
        ballMaterial.setColor("Color", new ColorRGBA(0.5f, 0f, 0f, 1f));
        result.setDebugMaterial(ballMaterial);
        result.setDebugMeshResolution(DebugShapeFactory.highResolution);

        PhysicsSpace physicsSpace = getPhysicsSpace(app);
        physicsSpace.addCollisionObject(result);

        // Update the ball's location before each time step.
        PhysicsTickListener listener = new PhysicsTickListener() {
            private float elapsedTime = 0f; // in seconds

            @Override
            public void physicsTick(PhysicsSpace space, float timeStep) {
                // do nothing
            }

            @Override
            public void prePhysicsTick(PhysicsSpace space, float timeStep) {
                // Cause the kinematic ball to orbit the origin.
                float orbitalPeriod = 8f; // seconds
                float phaseAngle
                        = elapsedTime * FastMath.TWO_PI / orbitalPeriod;

                float orbitRadius = 3f; // physics-space units
                float x = orbitRadius * FastMath.sin(phaseAngle);
                float z = orbitRadius * FastMath.cos(phaseAngle);
                Vector3f location = new Vector3f(x, 0f, z);
                result.setPhysicsLocation(location);

                elapsedTime += timeStep;
            }
        };
        physicsSpace.addTickListener(listener);

        return result;
    }

    /**
     * Create and configure a tiled, static horizontal plane and add it to the
     * physics space.
     *
     * @param app the application (not null)
     * @return the new instance
     */
    public static PhysicsRigidBody addTiledPlane(Application app) {
        Plane plane = new Plane(Vector3f.UNIT_Y, -2.25f);
        PlaneCollisionShape shape = new PlaneCollisionShape(plane);
        PhysicsRigidBody result
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        // Load a repeating tile texture.
        String assetPath = "Textures/greenTile.png";
        boolean flipY = false;
        TextureKey key = new TextureKey(assetPath, flipY);
        boolean generateMips = true;
        key.setGenerateMips(generateMips);
        AssetManager assetManager = app.getAssetManager();
        Texture texture = assetManager.loadTexture(key);
        texture.setMinFilter(Texture.MinFilter.Trilinear);
        texture.setWrap(Texture.WrapMode.Repeat);

        // Enable anisotropic filtering, to reduce blurring.
        Renderer renderer = app.getRenderer();
        EnumMap<Limits, Integer> limits = renderer.getLimits();
        Integer maxDegree = limits.get(Limits.TextureAnisotropy);
        int degree = (maxDegree == null) ? 1 : Math.min(8, maxDegree);
        texture.setAnisotropicFilter(degree);

        // Apply a tiled, unshaded debug material to the body.
        Material material = new Material(assetManager, Materials.UNSHADED);
        material.setTexture("ColorMap", texture);
        result.setDebugMaterial(material);

        // Generate texture coordinates during debug-mesh initialization.
        float tileSize = 1f;
        PlaneDmiListener planeDmiListener = new PlaneDmiListener(tileSize);
        result.setDebugMeshInitListener(planeDmiListener);

        PhysicsSpace physicsSpace = getPhysicsSpace(app);
        physicsSpace.addCollisionObject(result);

        return result;
    }

    /**
     * Create and configure a yellow, static pyramid and add it to the physics
     * space.
     *
     * @param app the application (not null)
     * @return the new instance
     */
    public static PhysicsRigidBody addYellowPyramid(Application app) {
        float yBase = -0.5f;
        CollisionShape pyramidShape = new HullCollisionShape(
                +2f, yBase, +2f,
                -2f, yBase, +2f,
                +2f, yBase, +2f,
                -2f, yBase, +2f,
                0f, 1f, 0f
        );
        PhysicsRigidBody result
                = new PhysicsRigidBody(pyramidShape, PhysicsBody.massForStatic);
        result.setPhysicsLocation(new Vector3f(0f, -1.75f, -20f));

        // Configure the debug visualization.
        AssetManager assetManager = app.getAssetManager();
        Material yellow = new Material(assetManager, Materials.UNSHADED);
        yellow.setColor("Color", new ColorRGBA(1f, 0.9f, 0.2f, 1f));
        result.setDebugMaterial(yellow);

        PhysicsSpace physicsSpace = getPhysicsSpace(app);
        physicsSpace.addCollisionObject(result);

        return result;
    }

    /**
     * Access the physics space of the specified application.
     *
     * @param app the application (not null, unaffected)
     * @return the pre-existing instance
     */
    public static PhysicsSpace getPhysicsSpace(Application app) {
        AppStateManager stateManager = app.getStateManager();
        BulletAppState bas = stateManager.getState(BulletAppState.class);
        PhysicsSpace result = bas.getPhysicsSpace();

        return result;
    }
}
