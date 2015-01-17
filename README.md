mpp-robot
=======
mpp-robot belongs to a set of packages to conduct motion planning for a sliding humanoid robot in arbitrary environments via motion prior informations.
### Description of package
Package precomputes the X space, a transformation of the original irreducible configuration space. Paper links will follow.
### Input: 
Specifications of the robotic system, in file mpp/robot/robotspecifications.py
### Output:
 * Samples from X, according to a sampling of C
 * Computation of polytope representation of C
 * (approximation of manifold by k-flats)
 * (connectivity graph of k-flats)

