package comp559.lcp;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import no.uib.cipr.matrix.DenseMatrix;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

/**
 * Implementation of a contact constraint.
 * @author kry
 */
public class Contact {

    /** Next available contact index, used for determining which rows of the jacobian a contact uses */
    static public int nextContactIndex = 0;
    
    /** Index of this contact, determines its rows in the jacobian */
    int index;
    
    /** First RigidBody in contact */
    RigidBody body1;
    
    /** Second RigidBody in contact */
    RigidBody body2;
    
    /** Contact normal in world coordinates */
    Vector3d normal = new Vector3d();
    
    /** Position of contact point in world coordinates */
    Point3d contactW = new Point3d();
    
    
    DenseMatrix J = new DenseMatrix(3, 12);
    Point3d ra = new Point3d();
    Point3d rb = new Point3d();
    Vector3d tangent1 = new Vector3d();
    Vector3d tangent2 = new Vector3d();
    double[] m;
    
    /**
     * Creates a new contact, and assigns it an index
     * @param body1
     * @param body2
     * @param contactW
     * @param normal
     */
    public Contact( RigidBody body1, RigidBody body2, Point3d contactW, Vector3d normal ) {
        this.body1 = body1;
        this.body2 = body2;
        this.contactW.set( contactW );
        this.normal.set( normal );      
        this.m = new double[] {body1.minv, body1.minv, body1.jinv, body2.minv, body2.minv, body2.jinv};	
        index = nextContactIndex++;        
        // TODO: obj3 you may want to add code here to compute and store the contact Jacobian
        ra.set(contactW.x - body1.x.x, contactW.y - body1.x.y, contactW.z - body1.x.z);
        rb.set(contactW.x - body2.x.x, contactW.y - body2.x.y, contactW.z - body1.x.z);
        // Jrow1
        J.add(0, 0, -normal.x);
        J.add(0, 1, -normal.y);
        J.add(0, 2, -normal.z);
        
        J.add(0, 3, -normal.z * ra.y + normal.y * ra.z);
        J.add(0, 4,  normal.z * ra.x - normal.x * ra.z);
        J.add(0, 5, -normal.y * ra.x + normal.x * ra.y);
       
        J.add(0, 6,  normal.x);
        J.add(0, 7,  normal.y);
        J.add(0, 8,  normal.z);
        
        J.add(0, 9,  normal.z * rb.y - normal.y * rb.z);
        J.add(0,10, -normal.z * rb.x + normal.x * rb.z);
        J.add(0,11,  normal.y * rb.x - normal.x * rb.y);
        
        
        //Jrow2
        tangent1.set(-normal.y, normal.x, 0);
        
        J.add(0, 0, -tangent1.x);
        J.add(0, 1, -tangent1.y);
        J.add(0, 2, -tangent1.z);
        
        J.add(0, 3, -tangent1.z * ra.y + tangent1.y * ra.z);
        J.add(0, 4,  tangent1.z * ra.x - tangent1.x * ra.z);
        J.add(0, 5, -tangent1.y * ra.x + tangent1.x * ra.y);
       
        J.add(0, 6,  tangent1.x);
        J.add(0, 7,  tangent1.y);
        J.add(0, 8,  tangent1.z);
        
        J.add(0, 9,  tangent1.z * rb.y - tangent1.y * rb.z);
        J.add(0,10, -tangent1.z * rb.x + tangent1.x * rb.z);
        J.add(0,11,  tangent1.y * rb.x - tangent1.x * rb.y);
        
        
        //Jrow3
        tangent2.set(-normal.x*normal.z, -normal.y*normal.z, normal.x*normal.x + normal.y*normal.y);
        
        J.add(0, 0, -tangent2.x);
        J.add(0, 1, -tangent2.y);
        J.add(0, 2, -tangent2.z);
        
        J.add(0, 3, -tangent2.z * ra.y + tangent2.y * ra.z);
        J.add(0, 4,  tangent2.z * ra.x - tangent2.x * ra.z);
        J.add(0, 5, -tangent2.y * ra.x + tangent2.x * ra.y);
       
        J.add(0, 6,  tangent2.x);
        J.add(0, 7,  tangent2.y);
        J.add(0, 8,  tangent2.z);
        
        J.add(0, 9,  tangent2.z * rb.y - tangent2.y * rb.z);
        J.add(0,10, -tangent2.z * rb.x + tangent2.x * rb.z);
        J.add(0,11,  tangent2.y * rb.x - tangent2.x * rb.y);
    }
    
    /**
     * Draws the contact points
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glPointSize(3);
        gl.glColor3f(.7f,0,0);
        gl.glBegin( GL.GL_POINTS );
        gl.glVertex3d(contactW.x, contactW.y, contactW.z);
        gl.glEnd();
    }
    
    /**
     * Draws the connections between bodies to visualize the 
     * the adjacency structure of the matrix as a graph.
     * @param drawable
     */
    public void displayConnection( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        // draw a line between the two bodies but only if they're both not pinned
        if ( !body1.pinned && ! body2.pinned ) {
            gl.glLineWidth(2);
            gl.glColor4f(0,.3f,0, 0.5f);
            gl.glBegin( GL.GL_LINES );
            gl.glVertex3d(body1.x.x, body1.x.y, body1.x.z);
            gl.glVertex3d(body2.x.x, body2.x.y, body2.x.z);
            gl.glEnd();
        }
    }
    
}
