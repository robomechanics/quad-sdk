function R = VectorCrossMatrix (r)

R = [0, -r(3), r(2);
 r(3), 0, -r(1);
 -r(2), r(1), 0];
