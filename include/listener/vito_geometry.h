// Software License Agreement (BSD License)
//
//   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
//   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef _VITO_GEOMETRY_H_
#define _VITO_GEOMETRY_H_

#include <common/box.h>
#include <array>

namespace pacv
{
//Globally define Vito Robot Geometry (Taken from meshes)
const static std::array<Box,7> lwr_arm{ {
  Box(-0.06, -0.06, -0.008,
       0.06,  0.094,  0.261), //Link1
  Box(-0.06, -0.094,  -0.06,
       0.06,  0.060, 0.192), //Link2
  Box(-0.06, -0.094, -0.008,
       0.06,  0.060, 0.261), //Link3
  Box(-0.06, -0.06, -0.06,
       0.06,  0.094, 0.192), //Link4
  Box(-0.06, -0.06, -0.008,
       0.06,  0.094, 0.251), //Link5
  Box(-0.071, -0.08, -0.071,
       0.071, 0.056,  0.057), //Link6
  Box(-0.04,  -0.04, -0.031,
       0.04,  0.04,  0)} };  //Link7

//Imu sensors and rubber gloves geometry
const static Box glove(-0.004, -0.004, 0,
                        0.004,  0.004, 0.004);
const static Box imu(-0.01, -0.005, -0.035,
                      0.01,  0.005, 0);
const static Box board(-0.06, -0.01, -0.01,
                        0,   0.01,  0);
//Soft hands
const static std::array<Box,21> soft_hand_right{ {
//base - coupler - clamps. in base ref system. take y of clamps, since biggest
  Box(-0.033, -0.037, 0.004 -0.035 -0.014,
       0.033,  0.037, 0.066),
//Right Palm + glove tolerance + board
  Box(-0.027 + glove.x1 + board.x1, -0.049 + glove.y1 + board.y1, -0.012 + glove.z1 + board.z1,
       0.012 + glove.x2 + board.x2,  0.04 + glove.y2 + board.y2, 0.113 + glove.z2 + board.z2),
//Soft Fingers: 4 equal + 1 thumb
//index finger
//Knuckle + glove tolerance + imu
  Box(-0.017 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.012 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.013 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Middle + glove tolerance + imu TODO
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu TODO
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//little finger
//Knuckle + glove tolerance + imu
  Box(-0.017 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.012 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.013 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Middle + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//middle finger
//Knuckle + glove tolerance + imu
  Box(-0.017 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.012 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.013 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Middle + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//ring finger
//Knuckle + glove tolerance + imu
  Box(-0.017 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.012 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.013 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Middle + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//thumb finger
//Knuckle + glove tolerance + imu
  Box(-0.006 +glove.x1 +imu.x1, -0.013 +glove.y1 +imu.y1, -0.011 +glove.z1 +imu.z1,
       0.031 +glove.x2 +imu.x2,  0.006 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2)
} };

//left hand
const static std::array<Box,21> soft_hand_left{ {
//base - coupler - clamps. in base ref system. take y of clamps, since biggest
  Box(-0.06, -0.06, 0.004 -0.05 -0.014,
       0.06,  0.06, 0.066),
//Left Palm + glove tolerance + board
  Box(-0.027 + glove.x1 + board.x1, -0.039 + glove.y1 + board.y1, -0.011 + glove.z1 + board.z1,
       0.012 + glove.x2 + board.x2,  0.049 + glove.y2 + board.y2, 0.113 + glove.z2 + board.z2),
//Soft Fingers: 4 equal + 1 thumb
//index finger
//Knuckle + glove tolerance + imu
  Box(-0.017 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.012 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.013 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Middle + glove tolerance + imu TODO
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu TODO
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//little finger
//Knuckle + glove tolerance + imu
  Box(-0.017 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.012 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.013 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Middle + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//middle finger
//Knuckle + glove tolerance + imu
  Box(-0.017 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.012 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.013 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Middle + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//ring finger
//Knuckle + glove tolerance + imu
  Box(-0.017 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.012 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.013 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Middle + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//thumb finger
//Knuckle + glove tolerance + imu
  Box(-0.006 +glove.x1 +imu.x1, -0.006 +glove.y1 +imu.y1, -0.011 +glove.z1 +imu.z1,
       0.031 +glove.x2 +imu.x2,  0.013 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2),
//Proximal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.018 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.01 +glove.z2 +imu.z2),
//Distal + glove tolerance + imu
  Box(-0.007 +glove.x1 +imu.x1, -0.007 +glove.y1 +imu.y1, -0.009 +glove.z1 +imu.z1,
       0.025 +glove.x2 +imu.x2,  0.007 +glove.y2 +imu.y2, 0.011 +glove.z2 +imu.z2)
} };

//approximate hand global dimensions without sensors
const static Box hand_right(-0.07, -0.06, -0.1,
                        0.025, 0.125, 0.21);
const static Box hand_left(-0.07, -0.125, -0.1,
                        0.025, 0.06, 0.21);

//naming
const static std::array<std::string, 7> arm_names {{
        "_arm_1_link",
        "_arm_2_link",
        "_arm_3_link",
        "_arm_4_link",
        "_arm_5_link",
        "_arm_6_link",
        "_arm_7_link"}};

const static std::array<std::string, 21> hand_names {{
    "_hand_softhand_base",
    "_hand_palm_link",
    "_hand_index_knuckle_link",
    "_hand_index_proximal_link",
    "_hand_index_middle_link",
    "_hand_index_distal_link",
    "_hand_little_knuckle_link",
    "_hand_little_proximal_link",
    "_hand_little_middle_link",
    "_hand_little_distal_link",
    "_hand_middle_knuckle_link",
    "_hand_middle_proximal_link",
    "_hand_middle_middle_link",
    "_hand_middle_distal_link",
    "_hand_ring_knuckle_link",
    "_hand_ring_proximal_link",
    "_hand_ring_middle_link",
    "_hand_ring_distal_link",
    "_hand_thumb_knuckle_link",
    "_hand_thumb_proximal_link",
    "_hand_thumb_distal_link"}};

}
#endif
