package comp559.lcp;
import java.util.ArrayList;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import comp559.lcp.RigidBody;
import comp559.lcp.Quad;
public class Box{
	public RigidBody b;
	public Quad quad= new Quad();
	public double width;
	public double height;
	public double depth =3.0;
//	public Camera camera = new Camera();
	public void set(RigidBody b) {
		this.b = b;
		this.quad.set(b.blocks.get(0).c,b.blocks.get(0).alpha, b.vertex);
		this.width = b.vertex.get(2).x-b.vertex.get(0).x;
		this.height = b.vertex.get(1).y-b.vertex.get(0).y;
	}
	public void draw( GLAutoDrawable drawable, Point3d pos) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glRotatef(45.0f, 0.0f, 0.0f, 1.0f);
		gl.glTranslatef(2.5f, 0.0f, 0.0f );
		gl.glScalef(0.5f, 0.5f, 0.5f);
		
		gl.glColor3f(1.0f,0.0f,0.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f(-1.0f, 1.0f, 1.0f);
	    gl.glVertex3f(-1.0f,-1.0f, 1.0f);
	    gl.glVertex3f( 1.0f,-1.0f, 1.0f);
	    gl.glVertex3f( 1.0f, 1.0f, 1.0f);
	    gl.glEnd();
	    gl.glColor3f(0.0f,1.0f,0.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f( 1.0f, 1.0f,-1.0f);
	    gl.glVertex3f( 1.0f,-1.0f,-1.0f);
	    gl.glVertex3f(-1.0f,-1.0f,-1.0f);
	    gl.glVertex3f(-1.0f, 1.0f,-1.0f);
	    gl.glEnd();
	    gl.glColor3f(0.0f,0.0f,1.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f( 1.0f, 1.0f, 1.0f);
	    gl.glVertex3f( 1.0f,-1.0f, 1.0f);
	    gl.glVertex3f( 1.0f,-1.0f,-1.0f);
	    gl.glVertex3f( 1.0f, 1.0f,-1.0f);
	    gl.glEnd();
	    gl.glColor3f(1.0f,1.0f,0.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f(-1.0f, 1.0f,-1.0f);
	    gl.glVertex3f(-1.0f,-1.0f,-1.0f);
	    gl.glVertex3f(-1.0f,-1.0f, 1.0f);
	    gl.glVertex3f(-1.0f, 1.0f, 1.0f);
	    gl.glEnd();
	    gl.glColor3f(1.0f,1.0f,1.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f( 1.0f, 1.0f,-1.0f);
	    gl.glVertex3f(-1.0f, 1.0f,-1.0f);
	    gl.glVertex3f(-1.0f, 1.0f, 1.0f);
	    gl.glVertex3f( 1.0f, 1.0f, 1.0f);
	    gl.glEnd();
	    gl.glColor3f(0.0f,1.0f,1.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f( 1.0f, -1.0f, 1.0f);
	    gl.glVertex3f(-1.0f, -1.0f, 1.0f);
	    gl.glVertex3f(-1.0f, -1.0f,-1.0f);
	    gl.glVertex3f( 1.0f, -1.0f,-1.0f);  
	    gl.glEnd();
//		}
	}
}
