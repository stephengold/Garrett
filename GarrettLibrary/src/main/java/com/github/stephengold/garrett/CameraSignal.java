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

/**
 * Enumerate the camera functions that can be controlled by signals.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public enum CameraSignal {
    // *************************************************************************
    // values

    /**
     * translate backward (camera's -Z direction)
     */
    Back,
    /**
     * enable (or disable) drag-to-orbit mode
     */
    DragToOrbit,
    /**
     * translate eastward (world +Z direction)
     */
    East,
    /**
     * translate forward (camera's look or +Z direction)
     */
    Forward,
    /**
     * enable (or disable) ghost mode (no contact response)
     */
    Ghost,
    /**
     * translate (slide or strafe) leftward (camera's +X direction)
     */
    Left,
    /**
     * translate northward (world +X direction)
     */
    North,
    /**
     * orbit counter-clockwise (world +Y axis), moving to the camera's right
     */
    OrbitCcw,
    /**
     * orbit clockwise (world -Y axis), moving to the camera's left
     */
    OrbitCw,
    /**
     * orbit in the camera's actual "down" direction
     */
    OrbitDown,
    /**
     * orbit in the camera's actual "up" direction
     */
    OrbitUp,
    /**
     * enable (or disable) point-to-look mode
     */
    PointToLook,
    /**
     * translate in the camera's preferred "down" direction
     */
    PreferredDown,
    /**
     * translate in the camera's preferred "up" direction
     */
    PreferredUp,
    /**
     * enable (or disable) high-mass mode (behave like a battering ram)
     */
    Ram,
    /**
     * translate (slide or strafe) rightward (camera's -X direction)
     */
    Right,
    /**
     * translate southward (world -X direction)
     */
    South,
    /**
     * translate in the camera's actual "down" direction
     */
    ViewDown,
    /**
     * translate in the camera's actual "up" direction
     */
    ViewUp,
    /**
     * translate westward (world -Z direction)
     */
    West,
    /**
     * translate downward (world -Y direction)
     */
    WorldDown,
    /**
     * translate upward (world +Y direction)
     */
    WorldUp,
    /**
     * enable (or disable) X-ray mode (no line-of-sight constraint)
     */
    Xray,
    /**
     * magnify the scene
     */
    ZoomIn,
    /**
     * de-magnify the scene
     */
    ZoomOut
}
