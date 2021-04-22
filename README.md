# Optimizations for rigid body animation 

Controls used for from assignment2 that can be ignored are collapsed in the panel, note that time `step size` can be found in `Substeps`. For the optimizations:
* SPHash can be enabled by selecting `Spatial Hash for Broad Phase`;
* sleeping and waking bodies is defaulted with a `kinetic energy threshold = 1e-4`, which can be changed to other settings in *lcp.CollisionProcessor.processCollision()*;
* constraint stabilization are enabled by `use compliance` and `use Baumgarte`;
* display of rigid bodies after collisions can be switched, options including:
   * three types of color changing,
   * texture mapping with two texture images. User defined image file can also be used by changing file path in lcp.CollisionProcessor.processCollision().

To reproduce the result we show in the video:

