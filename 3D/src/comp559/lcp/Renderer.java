package comp559.lcp;
import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;

import javax.swing.*;
import javax.vecmath.Point3d;

import com.jogamp.opengl.*;
import com.jogamp.opengl.awt.GLCanvas;
import com.jogamp.opengl.util.FPSAnimator;
import com.jogamp.opengl.GL.*;  
import com.jogamp.opengl.GL2.*; 
import com.jogamp.opengl.glu.GLU;

public class Renderer {
	// This code example is created for educational purpose
	// by Thorsten Thormaehlen (contact: www.thormae.de).
	// It is distributed without any warranty.
	  private GLU glu = new GLU();
	  public RigidBody b;
	  public Block block;
	  public float t = 1.0f;
	  
	  public void resize(GLAutoDrawable d, int w, int h) {
			GL2 gl = d.getGL().getGL2(); // get the OpenGL 2 graphics context
			gl.glViewport(0, 0, w, h);
			gl.glMatrixMode(GL2.GL_PROJECTION);
			gl.glLoadIdentity();
			glu.gluPerspective (30.0, (double)w/(double)h, 2.0, 20.0);
	  }
	  
    public void display(GLAutoDrawable d, RigidBody b) {
    	this.b = b;
    	GL2 gl = d.getGL().getGL2();  // get the OpenGL 2 graphics context
    	gl.glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    	gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
		
    	gl.glMatrixMode(GL2.GL_MODELVIEW);
    	gl.glLoadIdentity();
		
		// camera orbits in the y=10 plane
		// and looks at origin
    	double rad = Math.PI / 180.0f * t;
    	glu.gluLookAt(-20.0, 15.0 , -10.0, // eye
		          0.0, 0.0, 0.0, // look at
		          0.0, 5.0, 0.0); // up
		
		//		gl.glRotatef(45.0f, 0.0f, 0.0f, 1.0f);
		//		gl.glTranslatef(2.5f, 0.0f, 0.0f );
		//		gl.glTranslated( b.x.x, b.x.y,b.x.z);
		//      gl.glRotated(b.thed, b.rotAxis.x,b.rotAxis.y,b.rotAxis.z);
    	gl.glScalef(0.5f, 0.5f, 0.5f);
		//		
    	drawBox(d);
	}
	
	public void init(GLAutoDrawable d) {
	    GL2 gl = d.getGL().getGL2();
	    gl.glEnable(GL2.GL_DEPTH_TEST);
	  }
	  
	private void drawBox(GLAutoDrawable d) {
	    GL2 gl = d.getGL().getGL2();  
	    Block bl = b.blocks.get(0);
	    ArrayList<Point3d> vertex = b.vertex;
	    gl.glColor4f(bl.c.x,bl.c.y,bl.c.z,bl.alpha);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3d(vertex.get(0).x,vertex.get(0).y,vertex.get(0).z);
	    gl.glVertex3d(vertex.get(1).x,vertex.get(1).y,vertex.get(1).z);
	    gl.glVertex3d(vertex.get(2).x,vertex.get(2).y,vertex.get(2).z);
	    gl.glVertex3d(vertex.get(3).x,vertex.get(3).y,vertex.get(3).z);
	    gl.glEnd();
	    gl.glColor4f(bl.c.x,bl.c.y,bl.c.z,bl.alpha);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3d(vertex.get(0).x,vertex.get(0).y,-vertex.get(0).z);
	    gl.glVertex3d(vertex.get(1).x,-vertex.get(1).y,-vertex.get(1).z);
	    gl.glVertex3d(-vertex.get(2).x,-vertex.get(2).y,-vertex.get(2).z);
	    gl.glVertex3d(-vertex.get(3).x,vertex.get(3).y,-vertex.get(3).z);
	    gl.glEnd();
	    gl.glColor4f(bl.c.x/2,bl.c.y/2,bl.c.z/2,bl.alpha);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3d(vertex.get(0).x,vertex.get(0).y,vertex.get(0).z);
	    gl.glVertex3d(vertex.get(1).x,-vertex.get(1).y,vertex.get(1).z);
	    gl.glVertex3d(vertex.get(2).x,-vertex.get(2).y,-vertex.get(2).z);
	    gl.glVertex3d(vertex.get(3).x,vertex.get(3).y,-vertex.get(3).z);
	    gl.glEnd();
	    gl.glColor4f(bl.c.x/2,bl.c.y/2,bl.c.z/2,bl.alpha);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3d(-vertex.get(0).x,vertex.get(0).y,-vertex.get(0).z);
	    gl.glVertex3d(-vertex.get(1).x,-vertex.get(1).y,-vertex.get(1).z);
	    gl.glVertex3d(-vertex.get(2).x,-vertex.get(2).y,vertex.get(2).z);
	    gl.glVertex3d(-vertex.get(3).x,vertex.get(3).y,vertex.get(3).z);
	    gl.glEnd();
	    gl.glColor4f(bl.c.x/2,bl.c.y/2,bl.c.z/2,bl.alpha);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3d(vertex.get(0).x,vertex.get(0).y,-vertex.get(0).z);
	    gl.glVertex3d(-vertex.get(1).x,vertex.get(1).y,-vertex.get(1).z);
	    gl.glVertex3d(-vertex.get(2).x,vertex.get(2).y,vertex.get(2).z);
	    gl.glVertex3d(vertex.get(3).x,vertex.get(3).y,vertex.get(3).z);
	    gl.glEnd();
	    gl.glColor4f(bl.c.x/2,bl.c.y/2,bl.c.z/2,bl.alpha);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3d(vertex.get(0).x,-vertex.get(0).y,vertex.get(0).z);
	    gl.glVertex3d(-vertex.get(1).x,-vertex.get(1).y,vertex.get(1).z);
	    gl.glVertex3d(-vertex.get(2).x,-vertex.get(2).y,-vertex.get(2).z);
	    gl.glVertex3d(vertex.get(3).x,-vertex.get(3).y,-vertex.get(3).z);
	    gl.glEnd();
	}
}
