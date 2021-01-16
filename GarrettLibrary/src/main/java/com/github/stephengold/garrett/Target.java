/*
 Copyright (c) 2020-2021, Stephen Gold
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

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.math.Vector3f;

/**
 * An interface to determine the location and forward direction of a camera
 * target that's part of a collision object.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public interface Target {
    /**
     * Determine the forward direction for chase purposes.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit vector in world coordinates (either storeResult or a new
     * instance)
     */
    Vector3f forwardDirection(Vector3f storeResult);

    /**
     * Access the collision object that contains the primary target.
     *
     * @return the pre-existing object (not null)
     */
    PhysicsCollisionObject getTargetPco();

    /**
     * Determine the world location of the primary target.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the world location vector (either storeResult or a new instance)
     */
    Vector3f target(Vector3f storeResult);
}
