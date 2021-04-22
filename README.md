# Optimizations for rigid body animation 

Controls used for from assignment2 that can be ignored are collapsed in the panel, note that time `step size` can be found in `Substeps`. For the optimizations:
* SPHash can be enabled by selecting `Spatial Hash for Broad Phase`;
* sleeping and waking bodies is defaulted with a `kinetic energy threshold = 1e-4`, which can be changed to other settings in *lcp.CollisionProcessor.processCollision()*;
* constraint stabilization are enabled by `use compliance` and `use Baumgarte`;
* display of rigid bodies after collisions can be switched, options including:
   * three types of color changing,
   * texture mapping with two texture images. User defined image file can also be used by changing file path in *lcp.CollisionProcessor.processCollision()*.
* `saveTxt` and `loadTxt` can be used where sample text files are stored in *datalcp*.

To reproduce the result we show in the video, with all starting from the default settings:
1. load "hithere.png", set `restitution(bounce)` to 1;
2. load "domino.png", select `color changing 3` and apply an external force to the first domino;
3. load "falling domino.png", select `color changing 2`, larger the value for bounce or step size will speed up the simulation;
4. load "balance.png", deselect `color changing 1` and do `texture mapping2`;
5. load "wallWideDenseHigh",deselect `color changing 1` and do `texture mapping2`;
6. load "wallWideDenseHigh",deselect `color changing 1` and do `texture mapping1`.

Also, we mainly tested the performance of our SPHash, sleeping bodies, and constraint stabilization using factory. 

