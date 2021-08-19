import ..phys.geom.geom3d

/-
We define a standard, globally-available coordinate space which can be consumed by a user of
Peirce as a set of "world coordinates". In this case, we suppose that the world is a point and orientation in
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

