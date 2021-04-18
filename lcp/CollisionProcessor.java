package comp559.lcp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;

/**
 * Class for detecting and resolving collisions. Currently this class uses
 * penalty forces between rigid bodies.
 * 
 * @author kry
 */
public class CollisionProcessor {

	private List<RigidBody> bodies;

	/**
	 * The current contacts that resulted in the last call to process collisions
	 */
	public ArrayList<Contact> contacts = new ArrayList<Contact>();

	/**
	 * Creates this collision processor with the provided set of bodies
	 * 
	 * @param bodies
	 */
	public CollisionProcessor(List<RigidBody> bodies) {
		this.bodies = bodies;
	}

	/** keeps track of the time used for collision detection on the last call */
	double collisionDetectTime = 0;

	/**
	 * keeps track of the time used to solve the LCP based velocity update on the
	 * last call
	 */
	double collisionSolveTime = 0;

	public Map<Contact, double[]> blockPairs = new HashMap<>();

	/**
	 * Processes all collisions
	 * 
	 * @param dt time step
	 */
	public void processCollisions(double dt) {
		contacts.clear();
		Contact.nextContactIndex = 0;

		long now = System.nanoTime();
		broadPhase();
		collisionDetectTime = (System.nanoTime() - now) * 1e-9;

		if (contacts.size() > 0 && doLCP.getValue()) {
			now = System.nanoTime();

			double bounce = restitution.getValue();
			double mu = friction.getValue();
			// TODO: obj3 Compute velocity update with iterative solve of contact constraint
			// matrix.

			/**
			 * J 2k*3n Minv 3n*3n Jt 3n*2k D 2k*1 b 2k*1 lambda 2k*1 Minv has (1/mass 1/mass
			 * 1/inertia 1/......) on diagonal
			 */

			// set up w= A lambda + b
			double[] b = new double[2 * contacts.size()];
			double[] D = new double[2 * contacts.size()];
			for (Contact c : contacts) {
				int idx = c.index;
				double[] v = { c.body1.v.x, c.body1.v.y, c.body1.omega, c.body2.v.x, c.body2.v.y, c.body2.omega };
				double[] f = { c.body1.force.x, c.body1.force.y, c.body1.torque, c.body2.force.x, c.body2.force.y,
						c.body2.torque };

				// compute b = J(v+dt*f/M)
				double tmp1 = 0;
				double tmp2 = 0;
				for (int i = 0; i < 6; i++) {
					tmp1 += c.J.get(0, i) * (v[i] + dt * f[i] * c.m[i]);
					// 3.4 include bounce contribution
					tmp1 += c.J.get(0, i) * v[i] * bounce;
					tmp2 += c.J.get(1, i) * (v[i] + dt * f[i] * c.m[i]);
				}
				b[2 * idx] = tmp1;
				b[2 * idx + 1] = tmp2;

				// compute Dii, list of diagonal of A = JM^-1J^T
				double tmp3 = 0;
				double tmp4 = 0;
				for (int j = 0; j < 6; j++) {
					tmp3 += c.J.get(0, j) * c.m[j] * c.J.get(0, j);
					tmp4 += c.J.get(1, j) * c.m[j] * c.J.get(1, j);
				}
				D[2 * idx] = tmp3;
				D[2 * idx + 1] = tmp4;
			}

			// divisions are avoided by computing bi' = bi/Dii
			double[] bp = new double[2 * contacts.size()];
			for (int i = 0; i < b.length; i++) {
				bp[i] = b[i] / D[i];
			}

			// initialize lambda with 0 as suggested in discussion board
			double[] lambda = new double[2 * contacts.size()];
			double[] dV = new double[3 * bodies.size()];

			for (int i = 0; i < iterations.getValue(); i++) {
				// TODO: obj4 randomize the order of inner loop, boolean parameter was added in
				// the control panel
				if (randomization.getValue()) {
					Collections.shuffle(contacts);
				}

				for (Contact c : contacts) {

					// TODO: obj5 warm start PGS solve at each step
					if (warm.getValue()) {
						if (blockPairs.containsKey(c)) {
							lambda[2 * c.index] = blockPairs.get(c)[0];
							lambda[2 * c.index + 1] = blockPairs.get(c)[1];
						}
					}

					double lambda1 = -bp[2 * c.index] + lambda[2 * c.index];
					// assign lambda = lambda - bp - Jrowi*dV
					for (int j = 0; j < 3; j++) {
						lambda1 -= c.J.get(0, j) * dV[3 * c.body1.index + j] / D[2 * c.index];
						lambda1 -= c.J.get(0, j + 3) * dV[3 * c.body2.index + j] / D[2 * c.index];
					}
					// lower BD for lambda1
					lambda1 = Math.max(0, lambda1);
					double dlambda1 = lambda1 - lambda[2 * c.index];
					lambda[2 * c.index] = lambda1;

					double lambda2 = -bp[2 * c.index + 1] + lambda[2 * c.index + 1];
					for (int j = 0; j < 3; j++) {
						lambda2 -= c.J.get(1, j) * dV[3 * c.body1.index + j] / D[2 * c.index + 1];
						lambda2 -= c.J.get(1, j + 3) * dV[3 * c.body2.index + j] / D[2 * c.index + 1];
					}
					// compute BD with eq(29) and project lambda i with eq(30)
					double hi = mu * lambda[2 * c.index];
					lambda2 = Math.min(Math.max(-hi, lambda2), hi);
					double dlambda2 = lambda2 - lambda[2 * c.index + 1];
					lambda[2 * c.index + 1] = lambda2;

					// update dV = dV + Tcloi dlambda, 12 nonzero entries
					dV[3 * c.body1.index] += c.m[0] * c.J.get(0, 0) * dlambda1;
					dV[3 * c.body1.index + 1] += c.m[1] * c.J.get(0, 1) * dlambda1;
					dV[3 * c.body1.index + 2] += c.m[2] * c.J.get(0, 2) * dlambda1;
					dV[3 * c.body2.index] += c.m[3] * c.J.get(0, 3) * dlambda1;
					dV[3 * c.body2.index + 1] += c.m[4] * c.J.get(0, 4) * dlambda1;
					dV[3 * c.body2.index + 2] += c.m[5] * c.J.get(0, 5) * dlambda1;

					dV[3 * c.body1.index] += c.m[0] * c.J.get(1, 0) * dlambda2;
					dV[3 * c.body1.index + 1] += c.m[1] * c.J.get(1, 1) * dlambda2;
					dV[3 * c.body1.index + 2] += c.m[2] * c.J.get(1, 2) * dlambda2;
					dV[3 * c.body2.index] += c.m[3] * c.J.get(1, 3) * dlambda2;
					dV[3 * c.body2.index + 1] += c.m[4] * c.J.get(1, 4) * dlambda2;
					dV[3 * c.body2.index + 2] += c.m[5] * c.J.get(1, 5) * dlambda2;
				}
			}

			for (RigidBody rb : bodies) {
				rb.v.x += dV[3 * rb.index + 0];
				rb.v.y += dV[3 * rb.index + 1];
				rb.omega += dV[3 * rb.index + 2];
			}

			if (warm.getValue()) {
				for (Contact c : contacts) {
					double[] lambdaVal = { lambda[2 * c.index], lambda[2 * c.index + 1] };
					blockPairs.put(c, lambdaVal);
				}
			}
			collisionSolveTime = (System.nanoTime() - now) * 1e-9;
		}
	}

	/**
	 * Checks for collisions between bodies. Note that you can optionally implement
	 * some broad phase test such as spatial hashing to reduce the n squared
	 * body-body tests. Currently this does the naive n squared collision check.
	 */
	private void broadPhase() {
		// Naive n squared body test.. might not be that bad for small number of bodies
		visitID++;
		for (RigidBody b1 : bodies) {
			for (RigidBody b2 : bodies) { // not so inefficient given the continue on the next line
				/*
				 * if(b1.getKineticEnergy()>=1e-5 || b2.getKineticEnergy()>=1e-5) {
				 * System.out.println(b1.getKineticEnergy()); if ( b1.index >= b2.index )
				 * continue; if ( b1.pinned && b2.pinned ) continue; narrowPhase( b1, b2 ); }
				 */
				
				  if ( b1.index >= b2.index ) continue; 
				 // if ( b1.pinned && b2.pinned ) continue;
				  if ( (b1.pinned || b1.sleep) && (b2.pinned || b2.sleep) ) continue;
				  narrowPhase( b1, b2 );
				 
			}
		}
	}

	/**
	 * Checks for collision between boundary blocks on two rigid bodies. TODO: obj2
	 * This needs to be improved as the n-squared block test is too slow!
	 * 
	 * @param body1
	 * @param body2
	 */
	private void narrowPhase(RigidBody body1, RigidBody body2) {
		if (!useBVTree.getValue()) {
			for (Block b1 : body1.blocks) {
				for (Block b2 : body2.blocks) {
					processCollision(body1, b1, body2, b2);
				}
			}
		} else {
			// TODO: obj2 implement code to use hierarchical collision detection on body
			// pairs
			traverse(body1, body1.root, body2, body2.root);
		}
	}

	public void traverse(RigidBody body1, BVNode root1, RigidBody body2, BVNode root2) {
		// update the center before calling intersect

		if (root1.visitID != visitID) {
			root1.boundingDisc.updatecW();
			root1.visitID = visitID;
		}
		if (root2.visitID != visitID) {
			root2.boundingDisc.updatecW();
			root2.visitID = visitID;
		}

		body2.root.boundingDisc.updatecW();

		if (body1.root.boundingDisc.intersects(body2.root.boundingDisc)) {
			if (root1.isLeaf() && root2.isLeaf()) {
				processCollision(body1, root1.leafBlock, body2, root2.leafBlock);
			} else if (root1.isLeaf()) {
				traverse(body1, root1, body2, root2.child1);
				traverse(body1, root1, body2, root2.child2);
			} else if (root2.isLeaf()) {
				traverse(body1, root1.child1, body2, root2);
				traverse(body1, root1.child2, body2, root2);
			} else {
				traverse(body1, root1.child1, body2, root2.child1);
				traverse(body1, root1.child1, body2, root2.child2);
				traverse(body1, root1.child2, body2, root2.child1);
				traverse(body1, root1.child2, body2, root2.child2);
			}

		}
	}

	/**
	 * The visitID is used to tag boundary volumes that are visited in a given time
	 * step. Marking boundary volume nodes as visited during a time step allows for
	 * a visualization of those used, but it can also be used to more efficiently
	 * update the centers of bounding volumes (i.e., call a BVNode's updatecW method
	 * at most once on any given timestep)
	 */
	int visitID = 0;

	/**
	 * Resets the state of the collision processor by clearing all currently
	 * identified contacts, and reseting the visitID for tracking the bounding
	 * volumes used
	 */
	public void reset() {
		contacts.clear();
		Contact.nextContactIndex = 0;
		visitID = 0;
	}

	// some working variables for processing collisions
	private Point2d tmp1 = new Point2d();
	private Point2d tmp2 = new Point2d();
	private Point2d contactW = new Point2d();
	private Vector2d force = new Vector2d();
	private Vector2d contactV1 = new Vector2d();
	private Vector2d contactV2 = new Vector2d();
	private Vector2d relativeVelocity = new Vector2d();
	private Vector2d normal = new Vector2d();

	/**
	 * Processes a collision between two bodies for two given blocks that are
	 * colliding. Currently this implements a penalty force
	 * 
	 * @param body1
	 * @param b1
	 * @param body2
	 * @param b2
	 */
	private void processCollision(RigidBody body1, Block b1, RigidBody body2, Block b2) {
		double k = contactSpringStiffness.getValue();
		double c1 = contactSpringDamping.getValue();
		double threshold = separationVelocityThreshold.getValue();
		boolean useSpring = enableContactSpring.getValue();
		boolean useDamping = enableContactDamping.getValue();
		double kineticEnergyThres = 1e-1;
		//System.out.println(body1.getKineticEnergy()+" "+body2.getKineticEnergy());
		if(body1.getKineticEnergy()>=kineticEnergyThres || body2.getKineticEnergy()>=kineticEnergyThres) {
			body1.transformB2W.transform(b1.pB, tmp1);
			body2.transformB2W.transform(b2.pB, tmp2);
			double distance = tmp1.distance(tmp2);
			if (distance < Block.radius * 2) {
				// contact point at halfway between points
				// NOTE: this assumes that the two blocks have the same radius!
				contactW.interpolate(tmp1, tmp2, .5);
				// contact normal
				normal.sub(tmp2, tmp1);
				normal.normalize();
				// create the contact
				Contact contact = new Contact(body1, body2, contactW, normal);
				// simple option... add to contact list.				
				contacts.add(contact);
				 
				if (!doLCP.getValue()) {
					// compute relative body velocity at contact point
					body1.getSpatialVelocity(contactW, contactV1);
					body2.getSpatialVelocity(contactW, contactV2);
					relativeVelocity.sub(contactV1, contactV2);
					if (-relativeVelocity.dot(normal) < threshold) {
						if (useSpring) {
							// spring force
							double interpenetration = distance - Block.radius * 2; // a negative quantity
							force.scale(-interpenetration * k, normal);
							body2.applyContactForceW(contactW, force);
							force.scale(-1);
							body1.applyContactForceW(contactW, force);
						}
						if (useDamping) {
							// spring damping forces!
							// vertical
							force.scale(relativeVelocity.dot(normal) * c1, normal);
							body2.applyContactForceW(contactW, force);
							force.scale(-1);
							body1.applyContactForceW(contactW, force);
						}
					}
				}
			}
			body1.sleep = false;
			body2.sleep = false;
	    }
		else if(body1.getKineticEnergy()<kineticEnergyThres) {
			body1.sleep = true;
		}else if(body2.getKineticEnergy()<kineticEnergyThres) {
			body2.sleep = true;
		}
	}

	/** Stiffness of the contact penalty spring */
	private DoubleParameter contactSpringStiffness = new DoubleParameter("penalty contact stiffness", 1e3, 1, 1e5);

	/** Viscous damping coefficient for the contact penalty spring */
	private DoubleParameter contactSpringDamping = new DoubleParameter("penalty contact damping", 10, 1, 1e4);

	/**
	 * Threshold for the relative velocity in the normal direction, for determining
	 * if spring force will be applied.
	 */
	private DoubleParameter separationVelocityThreshold = new DoubleParameter(
			"penalty separation velocity threshold (controls bounce)", 1e-9, 1e-9, 1e3);

	/** Enables the contact penalty spring */
	private BooleanParameter enableContactSpring = new BooleanParameter("enable penalty contact spring", true);

	/** Enables damping of the contact penalty spring */
	private BooleanParameter enableContactDamping = new BooleanParameter("enable penalty contact damping", true);

	/** Restitution parameter for contact constraints */
	public DoubleParameter restitution = new DoubleParameter("restitution (bounce)", 0, 0, 1);

	/** Coulomb friction coefficient for contact constraint */
	public DoubleParameter friction = new DoubleParameter("Coulomb friction", 0.33, 0, 2);

	/** Number of iterations to use in projected Gauss Seidel solve */
	public IntParameter iterations = new IntParameter("iterations for GS solve", 10, 1, 500);

	/** Flag for switching between penalty based contact and contact constraints */
	private BooleanParameter doLCP = new BooleanParameter("do LCP solve", false);

	/**
	 * Flag for enabling the use of hierarchical collision detection for body pairs
	 */
	private BooleanParameter useBVTree = new BooleanParameter("use BVTree", false);

	private BooleanParameter randomization = new BooleanParameter("use randomization", false);

	private BooleanParameter warm = new BooleanParameter("use warm starts", false);

	/**
	 * @return controls for the collision processor
	 */
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.setBorder(new TitledBorder("Collision Processing Controls"));
		vfp.add(useBVTree.getControls());
		vfp.add(doLCP.getControls());
		vfp.add(randomization.getControls());
		vfp.add(warm.getControls());
		vfp.add(iterations.getSliderControls());
		vfp.add(restitution.getSliderControls(false));
		vfp.add(friction.getSliderControls(false));

		VerticalFlowPanel vfp2 = new VerticalFlowPanel();
		vfp2.setBorder(new TitledBorder("penalty method controls"));
		vfp2.add(contactSpringStiffness.getSliderControls(true));
		vfp2.add(contactSpringDamping.getSliderControls(true));
		vfp2.add(separationVelocityThreshold.getSliderControls(true));
		vfp2.add(enableContactDamping.getControls());
		vfp2.add(enableContactSpring.getControls());

		CollapsiblePanel cp = new CollapsiblePanel(vfp2.getPanel());
		cp.collapse();
		vfp.add(cp);
		return vfp.getPanel();
	}

}
