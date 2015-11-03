#ifndef _INCL_VITO_GEOMETRY
#define _INCL_VITO_GEOMETRY

#include <pacman_vision/utility.h>
#include <array>

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
  Box(-0.033, -0.037, 0.004 -0.035 -0.014,
       0.033,  0.037, 0.066),
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

#endif
