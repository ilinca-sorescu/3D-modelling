#include "${filename}"
#include "colors.inc"

light_source { <100, 100, 100> color rgb<1, 1, 1> }
light_source { <10, -200, -34> color rgb<0.5, 1, 0.7> }
light_source { <-73, 124, 343> color rgb<0.2, 0.7, 0.5> }

camera {
    location <${", ".join(str(x) for x in camera)}>
    look_at <0, 0, 0>
  }

