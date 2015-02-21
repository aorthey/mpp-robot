mpp-robot
=======
mpp-robot belongs to a set of packages to conduct motion planning for a sliding humanoid robot in arbitrary environments via motion prior informations.
### Description of package
Package precomputes the X space, a transformation of the original irreducible configuration space. Paper links will follow.
### Input: 
H values specified in file mpp/robot/robotspecifications.py
### Output:
Convex polytopes of configurations and graph structure of connectivity between
configuration polytopes

### Example:
Given stepsizes of h values (default: 0.02m for h1,h2,h3), the following
function computes convex sets by computing
  * h1,h2,h3 to a configuration q
  * q to a cross section xl,xr (left side and right side)
  * cross sections xl,xr to polytopes
  * polytopes saved in output/polytopes folder

```
python main-convex-set-builder.py
```
