/*========================================================================*/
/* scrmbl.pov - Demo. the rubic.inc file.                                 */
/* by John L. Bolt, jlb@magpage.com                                       */

#include "colors.inc"
#include "woods.inc"
#include "metals.inc"

#include "rubic.inc"

/*========================================================================*/
/* Lights macro                                                           */

#macro mLights(Hgt, Spc, Rad, Fal, Obj)
//union {
/*
  light_source { <      0, Hgt,      0>+Obj, rgb <1.0, 1.0, 1.0>
    spotlight radius Rad falloff Fal point_at Obj
  }
*/
/*  light_source { <-Spc, Hgt,-Spc>+Obj, rgb <1.0, 1.0, 1.0>
    spotlight radius Rad falloff Fal point_at Obj
  }

  light_source { <-Spc, Hgt, Spc>+Obj, rgb <1.0, 1.0, 1.0>
    spotlight radius Rad falloff Fal point_at Obj
  }

  light_source { < Spc, Hgt, Spc>+Obj, rgb <1.0, 1.0, 1.0>
    spotlight radius Rad falloff Fal point_at Obj
  }

  light_source { < Spc, Hgt,-Spc>+Obj, rgb <1.0, 1.0, 1.0>
    spotlight radius Rad falloff Fal point_at Obj
  }
}*/
#end

/*========================================================================*/
/* The scene                                                              */

/*------------------------------------------------------------------------*/
/*                                                                        */

/*sky_sphere {

  pigment {
    gradient y
    color_map {
      [ 0.00 color rgb <0.1, 0.9, 0.1> ]
      [ 0.49 color rgb <0.5, 0.7, 0.5> ]
      [ 0.50 color rgb <0.5, 0.5, 0.7> ]
      [ 1.00 color rgb <0.1, 0.1, 0.9> ]
    }
    scale 2 translate -1
  }
}*/

/*------------------------------------------------------------------------*/

/*#declare loca = <-3, 6, -3>;
#declare look = <0, 3, -0>;
#declare lvec = look - loca;

camera {
  location loca
  look_at  look
}*/

/*light_source { <100, 100, 100> color rgb<1, 1, 1> }

camera {
    location <-3, 6, -3>
    look_at <0, 0, 0>
  }*/

background{ color rgb <0.25, 0.25, 0.25> }

/*------------------------------------------------------------------------*/

//object { mLights(40, 40, 45, 45, look) }

/*------------------------------------------------------------------------*/

/*disc { 0, y, 24
  texture { 
    T_Wood18 rotate 0.5*x scale 5
//  pigment { color rgb <0.8, 0.8, 0.8> }
    finish { ambient 0.1 }
  }
}*/

/*------------------------------------------------------------------------*/

#declare tTmp = array[3][3][3]          // Temporary translation array
#declare MvStr = "FARATrlRarlFRtAtrtLB" // Move string to scramble the cube

#declare SStrX = "TBTBTFlbtlbtbFbffbFBfbFBABaRbrBRbrBbbABaBLbllBLBFbfbrbRbfBFBBabA"
#declare SStrY = "bRBrRbfBFrAAttRRTFFLLFFLLFFLLtRRttAAlTLtlTLtblTLtlTLtblTLtlTLtbb"

#declare SStr  = concat(SStrX, SStrY)   // Move string to solve the cube
#declare MStr  = concat(MvStr, SStr)    // Complete move string

/* Uncomment the following line to enable animation code                  */
//#declare _animate=1;
#ifndef (_animate)

Manip(Tran, iTran, SStr)                            // Twist the cube
Rot("T", tTmp, Tran, iTran, 0.1)                    // Twist a slice just a little
object { Rubic(Cubes, tTmp)}         // ... The cube

#else // (_animate)
/*
#declare Astr = SStr                                // The move string to animate
#declare k = strlen(Astr);                          // Length of the move string
#declare Idx = int(clock*k);                        // Index into move string of current move
#declare Fra = clock*k - Idx;                       // Fraction of a move
#if (Idx < strlen(Astr))                            // Catch end-of-move-string
  #declare Move = substr(Astr, Idx+1, 1)            // Get move character
#else
  #declare Move = " "
#end
#declare Hist = concat(MvStr, substr(Astr, 1, Idx)) // Move string up to current move
Manip(Tran, iTran, Hist)                            // Twist the cube
Rot(Move, tTmp, Tran, iTran, Fra)                   // Do a fractional move
object { Rubic(Cubes, tTmp) translate 3*y }         // ...
*/
#end // (_animate)

/*------------------------------------------------------------------------*/
