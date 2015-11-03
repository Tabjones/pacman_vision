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

//Soft hands
const static std::array<Box,21> soft_hand_right{ {
//base - coupler - clamps. in base ref system. take y of clamps, since biggest
  Box(-0.033, -0.037, 0.004 -0.035 -0.014,
       0.033,  0.037, 0.066),
//Right Palm + glove tolerance + board TODO
  Box(-0.027, -0.049, -0.012,
       0.012,  0.04, 0.113),
//Soft Fingers: 4 equal + 1 thumb
//index finger
//Knuckle + glove tolerance + imu TODO
  Box(-0.017, -0.007, -0.012,
       0.018,  0.007, 0.013),
//Proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Middle + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011),
//little finger
//Knuckle + glove tolerance + imu TODO
  Box(-0.017, -0.007, -0.012,
       0.018,  0.007, 0.013),
//Proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Middle + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011),
//middle finger
//Knuckle + glove tolerance + imu TODO
  Box(-0.017, -0.007, -0.012,
       0.018,  0.007, 0.013),
//Proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Middle + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011),
//ring finger
//Knuckle + glove tolerance + imu TODO
  Box(-0.017, -0.007, -0.012,
       0.018,  0.007, 0.013),
//Proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Middle + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011),
//thumb finger
//knuckle + glove tolerance + imu TODO
  Box(-0.006, -0.013, -0.011,
       0.031,  0.006,  0.011),
//proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011)
} };

//left hand
const static std::array<Box,21> soft_hand_left{ {
//base - coupler - clamps. in base ref system. take y of clamps, since biggest
  Box(-0.033, -0.037, 0.004 -0.035 -0.014,
       0.033,  0.037, 0.066),
//Left Palm + glove tolerance + board TODO
  Box(-0.027, -0.039, -0.011,
       0.012,  0.049, 0.113),
//Soft Fingers: 4 equal + 1 thumb
//index finger
//Knuckle + glove tolerance + imu TODO
  Box(-0.017, -0.007, -0.012,
       0.018,  0.007, 0.013),
//Proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Middle + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011),
//little finger
//Knuckle + glove tolerance + imu TODO
  Box(-0.017, -0.007, -0.012,
       0.018,  0.007, 0.013),
//Proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Middle + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011),
//middle finger
//Knuckle + glove tolerance + imu TODO
  Box(-0.017, -0.007, -0.012,
       0.018,  0.007, 0.013),
//Proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Middle + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011),
//ring finger
//Knuckle + glove tolerance + imu TODO
  Box(-0.017, -0.007, -0.012,
       0.018,  0.007, 0.013),
//Proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Middle + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//Distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011),
//thumb finger
//knuckle + glove tolerance + imu TODO
  Box(-0.006, -0.006, -0.011,
       0.031,  0.013,  0.011),
//proximal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.018,  0.007,  0.01),
//distal + glove tolerance + imu TODO
  Box(-0.007, -0.007, -0.009,
       0.025,  0.007,  0.011)
} };


//approximate hand global dimensions
const static Box hand_right(-0.07, -0.06, -0.1,
                        0.025, 0.125, 0.21);
const static Box hand_left(-0.07, -0.125, -0.1,
                        0.025, 0.06, 0.21);

#endif
