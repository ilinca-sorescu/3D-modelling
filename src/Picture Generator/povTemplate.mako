#include "${filename}"
#include "colors.inc"

light_source { <100, 100, 100> color rgb<1, 1, 1> }
light_source { <100, 100, -100> color rgb<1, 1, 1> }
light_source { <100, -100, 100> color rgb<1, 1, 1> }
light_source { <100, -100, -100> color rgb<1, 1, 1> }
light_source { <-100, 100, 100> color rgb<1, 1, 1> }
light_source { <-100, 100, -100> color rgb<1, 1, 1> }
light_source { <-100, -100, 100> color rgb<1, 1, 1> }
light_source { <-100, -100, -100> color rgb<1, 1, 1> }

camera {
    location <${", ".join(str(x) for x in camera)}>
    look_at <0, 0, 0>
  }

