/*
 Copyright (c) 2022-2025 Stephen Gold

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

import com.jme3.app.Application;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.asset.TextureKey;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.joints.JointEnd;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.bullet.util.PlaneDmiListener;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Limits;
import com.jme3.renderer.Renderer;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.texture.Texture;
import java.util.EnumMap;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;

/**
 * A utility class to populate a PhysicsSpace for demos and tutorial apps.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class DemoSpace {
    // *************************************************************************
    // constants and loggers

    /**
     * X coordinate of the door's center (in physics space)
     */
    final private static float doorCenterX = 8f;
    /**
     * half the height of the door (in physics-space units)
     */
    final private static float doorHalfHeight = 2f;
    /**
     * half the width of the door (in physics-space units)
     */
    final private static float doorHalfWidth = 2f;
    /**
     * Y coordinate of the floor (in physics space)
     */
    final private static float floorY = -2.25f;
    /**
     * half the thickness of boxes (in physics-space units)
     */
    final private static float halfThickness = 0.25f;
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
     * Create a blue, dynamic body with a box shape and a hinge joint and add it
     * to the space.
     *
     * @param app the application (not null)
     * @return a new dynamic rigid body
     */
    public static PhysicsRigidBody addBlueDoor(Application app) {
        BoxCollisionShape shape = new BoxCollisionShape(
                doorHalfWidth, doorHalfHeight, halfThickness);
        float mass = 1000f;
        PhysicsRigidBody result = new PhysicsRigidBody(shape, mass);
        result.setPhysicsLocation(
                new Vector3f(doorCenterX, floorY + doorHalfHeight, 0f));

        // Disable sleep (deactivation).
        result.setEnableSleep(false);

        // Configure the debug visualization.
        Material doorMaterial = createLitMaterial(app, 0.1f, 0.1f, 1f);
        result.setDebugMaterial(doorMaterial);
        result.setDebugMeshNormals(MeshNormals.Facet);

        // Add a single-ended physics joint to constrain the door's motion.
        Vector3f pivotInDoor = new Vector3f(-doorHalfWidth, 0f, 0f);
        float x = doorCenterX - doorHalfWidth;
        Vector3f pivotInWorld = new Vector3f(x, floorY + doorHalfHeight, 0f);
        HingeJoint joint = new HingeJoint(result, pivotInDoor, pivotInWorld,
                Vector3f.UNIT_Y, Vector3f.UNIT_Y, JointEnd.B);
        float lowLimitAngle = -FastMath.HALF_PI;
        float highLimitAngle = +FastMath.HALF_PI;
        joint.setLimit(lowLimitAngle, highLimitAngle);

        PhysicsSpace physicsSpace = getPhysicsSpace(app);
        physicsSpace.addCollisionObject(result);
        physicsSpace.addJoint(joint);

        return result;
    }

    /**
     * Create and configure a gray, static doorframe and add it at to the
     * physics space.
     *
     * @param app the application (not null)
     * @return a new static rigid body
     */
    public static PhysicsRigidBody addDoorframe(Application app) {
        float frameHalfWidth = 0.5f;
        BoxCollisionShape jambShape = new BoxCollisionShape(
                frameHalfWidth, doorHalfHeight, halfThickness);

        float lintelLength = doorHalfWidth + 2 * frameHalfWidth;
        BoxCollisionShape lintelShape = new BoxCollisionShape(
                lintelLength, frameHalfWidth, halfThickness);

        CompoundCollisionShape shape = new CompoundCollisionShape();
        shape.addChildShape(jambShape, doorHalfWidth + frameHalfWidth, 0f, 0f);
        shape.addChildShape(jambShape, -doorHalfWidth - frameHalfWidth, 0f, 0f);
        shape.addChildShape(
                lintelShape, 0f, doorHalfHeight + frameHalfWidth, 0f);

        PhysicsRigidBody result
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);
        result.setPhysicsLocation(
                new Vector3f(doorCenterX, floorY + doorHalfHeight, 0f));

        // Configure the debug visualization.
        Material grayMaterial = createLitMaterial(app, 0.1f, 0.1f, 0.1f);
        result.setDebugMaterial(grayMaterial);
        result.setDebugMeshNormals(MeshNormals.Facet);

        PhysicsSpace physicsSpace = getPhysicsSpace(app);
        physicsSpace.addCollisionObject(result);

        return result;
    }

    /**
     * Add background, lighting, and shadows to the specified scene.
     *
     * @param app the application (not null)
     * @param scene where to add the lights (not null)
     */
    public static void addLighting(Application app, Spatial scene) {
        // Set the viewport's background color to light blue.
        ViewPort viewPort = app.getViewPort();
        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        // Add an ambient light.
        ColorRGBA ambientColor = new ColorRGBA(0.1f, 0.1f, 0.1f, 1f);
        AmbientLight ambient = new AmbientLight(ambientColor);
        scene.addLight(ambient);
        ambient.setName("ambient");

        // Add a direct light.
        ColorRGBA directColor = ColorRGBA.White.clone();
        Vector3f direction = new Vector3f(-7f, -3f, -4f).normalizeLocal();
        DirectionalLight sun = new DirectionalLight(direction, directColor);
        scene.addLight(sun);
        sun.setName("sun");

        // Render shadows based on the directional light.
        viewPort.clearProcessors();
        AssetManager assetManager = app.getAssetManager();
        int shadowMapSize = 4_096; // in pixels
        int numSplits = 3;
        DirectionalLightShadowRenderer sr = new DirectionalLightShadowRenderer(
                assetManager, shadowMapSize, numSplits);
        sr.setEdgeFilteringMode(EdgeFilteringMode.PCFPOISSON);
        sr.setEdgesThickness(5);
        sr.setLight(sun);
        sr.setShadowIntensity(0.6f);
        viewPort.addProcessor(sr);
    }

    /**
     * Create and configure a gray, static box and add it at the origin of the
     * physics space.
     *
     * @param app the application (not null)
     * @return the new instance
     */
    public static PhysicsRigidBody addGrayBox(Application app) {
        CollisionShape boxShape
                = new BoxCollisionShape(1f, -floorY, halfThickness);
        PhysicsRigidBody result
                = new PhysicsRigidBody(boxShape, PhysicsBody.massForStatic);

        // Configure the debug visualization.
        Material boxMaterial = createLitMaterial(app, 0.1f, 0.1f, 0.1f);
        result.setDebugMaterial(boxMaterial);
        result.setDebugMeshNormals(MeshNormals.Facet);

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
        Material ballMaterial = createLitMaterial(app, 0.5f, 0f, 0f);
        result.setDebugMaterial(ballMaterial);
        result.setDebugMeshNormals(MeshNormals.Sphere);
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
        Plane plane = new Plane(Vector3f.UNIT_Y, floorY);
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
        float yBase = -1.5f;
        CollisionShape pyramidShape = new HullCollisionShape(
                +6f, yBase, +6f,
                -6f, yBase, +6f,
                +6f, yBase, -6f,
                -6f, yBase, -6f,
                0f, 3f, 0f
        );
        PhysicsRigidBody result
                = new PhysicsRigidBody(pyramidShape, PhysicsBody.massForStatic);
        float y = floorY - yBase;
        result.setPhysicsLocation(new Vector3f(0f, y, -20f));

        // Configure the debug visualization.
        Material yellow = createLitMaterial(app, 1f, 0.9f, 0.2f);
        result.setDebugMaterial(yellow);
        result.setDebugMeshNormals(MeshNormals.Facet);

        PhysicsSpace physicsSpace = getPhysicsSpace(app);
        physicsSpace.addCollisionObject(result);

        return result;
    }

    /**
     * Create a single-sided lit material with the specified reflectivities.
     *
     * @param app the application (not null, unaffected)
     * @param red the desired reflectivity for red light (&ge;0, &le;1)
     * @param green the desired reflectivity for green light (&ge;0, &le;1)
     * @param blue the desired reflectivity for blue light (&ge;0, &le;1)
     * @return a new instance (not null)
     */
    public static Material createLitMaterial(
            Application app, float red, float green, float blue) {
        AssetManager assetManager = app.getAssetManager();
        Material result = new Material(assetManager, Materials.LIGHTING);
        result.setBoolean("UseMaterialColors", true);

        float opacity = 1f;
        result.setColor("Ambient", new ColorRGBA(red, green, blue, opacity));
        result.setColor("Diffuse", new ColorRGBA(red, green, blue, opacity));

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
