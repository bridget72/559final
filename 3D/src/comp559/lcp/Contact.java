package comp559.lcp;
//Agnes Liu 260713093
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
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
    
    /** Store whether to iterate or not for warm start*/
    /** First RigidBody in contact */
    RigidBody body1;
    
    /** Second RigidBody in contact */
    RigidBody body2;
    
    /** Contact normal in world coordinates */
    Vector3d normal = new Vector3d();
    
    /** Position of contact point in world coordinates */
    Point3d contactW = new Point3d();
    
    Point3d ra = new Point3d();
    Point3d rb = new Point3d();
    Vector3d tangent1 = new Vector3d();
    Vector3d tangent2 = new Vector3d();
    
    double[] J1 = new double[12];
    double[] J2 = new double[12];
    double[] J3 = new double[12];
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
        this.contactW.set(contactW);;
        this.normal.set( normal );        
        index = nextContactIndex++;        
        //TODO: you may want to add code here to compute and store the contact Jacobian 
        ra.set(contactW.x - body1.x.x, contactW.y - body1.x.y, contactW.z - body1.x.z);
        rb.set(contactW.x - body2.x.x, contactW.y - body2.x.y, contactW.z - body1.x.z);
        this.tangent1.set(-normal.y, normal.x, 0); 
        this.tangent2.set(-normal.x*normal.z, -normal.y*normal.z, normal.x*normal.x + normal.y*normal.y);
        this.J1 = new double[] {
        		-normal.x, -normal.y,-normal.z,
        		-normal.z * ra.y + normal.y * ra.z, normal.z * ra.x - normal.x * ra.z,-normal.y * ra.x + normal.x * ra.y,
        		 normal.x,  normal.y, normal.z,
        		 normal.z * rb.y - normal.y * rb.z,-normal.z * rb.x + normal.x * rb.z, normal.y * rb.x - normal.x * rb.y
        };
        		
        this.J2 = new double[] {
        		-tangent1.x, -tangent1.y, -tangent1.z,
        		-tangent1.z * ra.y + tangent1.y * ra.z, tangent1.z * ra.x - tangent1.x * ra.z,-tangent1.y * ra.x + tangent1.x * ra.y,
        		 tangent1.x,  tangent1.y,  tangent1.z,
        		 tangent1.z * rb.y - tangent1.y * rb.z,-tangent1.z * rb.x + tangent1.x * rb.z, tangent1.y * rb.x - tangent1.x * rb.y
        };
        
        this.J3 = new double[] {
        		-tangent2.x, -tangent2.y, -tangent2.z,
        		-tangent2.z * ra.y + tangent2.y * ra.z, tangent2.z * ra.x - tangent2.x * ra.z,-tangent2.y * ra.x + tangent2.x * ra.y,
        		 tangent2.x,  tangent2.y,  tangent2.z,
        		 tangent2.z * rb.y - tangent2.y * rb.z,-tangent2.z * rb.x + tangent2.x * rb.z, tangent2.y * rb.x - tangent2.x * rb.y
        };
        
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
        gl.glVertex3d(contactW.x, contactW.y,contactW.z);
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
            gl.glVertex3d(body1.x.x, body1.x.y,body1.x.z);
            gl.glVertex3d(body2.x.x, body2.x.y,body2.x.z);
            gl.glEnd();
        }
    }
    
}
