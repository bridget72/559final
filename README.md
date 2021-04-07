# 559final

TODO:
* Sleeping and waking bodies is a technique often employed in multi-body simulators to efficiently simulate and update only the bodies in motion.
* PGS parallelizes trivially by moving towards a Jacobi type solve. See for instance the colouring and block parallel solves used in Fratarcangeli et al. 2016 for soft body dynamics.
* Consider other forces, springs, damping, constraints. You may need a different way to define your scenes to include other mechanisms.
* 3D simulation
   * ~~Block~~
   * BVNode
   * CollisionComputationMonitor
   * CollisionProcessor
   * Contact
   * Disc
   * Factory
   * ImageBlocker
   * LCPApp
   * MinimumEnclosingCircle
   * MouseSpringForce
   * RigidBody
   * RigidBodySystem
   * RigidTransform.java
