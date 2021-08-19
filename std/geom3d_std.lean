import ..phys.geom.geom3d

/-
Rule of thumb: Don't use std_space, std_frame in
application code. Define your own new world_space,
world_frame, etc. TODO: Enforce encapsulation.
-/

/-
We define a standard, globally-available standard
Euclidean coordinate space on an unstructured real
affine 3-space, representing real geometric 3-space.

Note: There's no canonical choice for a frame on 
real, physical, Euclidean 3-space. Pick any physical
interpretation you want of the origin and axes of the
new frame. Then build all other geometry relative to 
that.


  - origin: 
    - internal: abstract zero point (built in)
    - external: concrete real place (you provide)
  - frame (looking from door):
    - 0 : vector
      - direction: right along the back wall
      - : unit: meter
    - 1 : vector
      - direction: from back left corner towards the door
      - unit: meter
    - 2 : vector 
      - direction: up back left corner
      - unit: meter


In this case, we suppose that the world is a point and orientation in
Rice Hall, described below, from which a user can derive other coordinate spaces relative to this.

(1) ORIGIN: the world_geom_acs.origin represents the 
northwest-ish (back right) lower corner of the Rice Hall
Less Lab

(2) BASIS VECTORS
    basis0 
      - points right/east along the wall
      - unit length is 1m 
      - right handed chirality
    basis1 
      - points to the door along the west wall, 
      - unit length is 1m
      - RHC
    basis2 
      - points up along the NW corner of the room, 
      - unit length is one meter, 
      - RHC

Some notes on Chirality/"Handedness":
https://en.wikipedia.org/wiki/Orientation_(vector_space)
http://www.cs.cornell.edu/courses/cs4620/2008fa/asgn/ray1/fcg3-sec245-248.pdf


(3) ACS is given by [Origin, b0, b1, b2]
-/

