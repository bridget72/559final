package comp559.lcp;
import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;

import javax.swing.*;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.*;
import com.jogamp.opengl.awt.GLCanvas;
import com.jogamp.opengl.util.FPSAnimator;
import com.jogamp.opengl.GL.*;  
import com.jogamp.opengl.GL2.*; 
import com.jogamp.opengl.glu.GLU;

public class Renderer {
	  private GLU glu = new GLU();
	  public RigidBody b;
	  public Block block;
	  public float t = 1.0f;
	  public void resize(GLAutoDrawable d, int w, int h) {
			GL2 gl = d.getGL().getGL2();
			gl.glViewport(0, 0, w, h);
			gl.glMatrixMode(GL2.GL_PROJECTION);
			gl.glLoadIdentity();
			glu.gluPerspective (30.0, (double)w/(double)h, 2.0, 20.0);
	  }
	  
    public void display(GLAutoDrawable d, RigidBody b,int w, int h) {
    	this.b = b;
    	GL2 gl = d.getGL().getGL2();
    	gl.glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    	gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
		
    	gl.glMatrixMode(GL2.GL_MODELVIEW);
    	gl.glLoadIdentity();
		// camera orbits in the y=10 plane
		// and looks at origin
    	double rad = Math.PI / 180.0f;
		      
    	glu.gluLookAt(10.0*Math.cos(rad), 10.0*Math.sin(rad), 10.0 ,// eye
                 0.0, 0.0, 0.0, // look at
                 0.0, 3.0, 0.0); // up
//
//		gl.glRotated(b.thed, b.rotAxis.x, b.rotAxis.y, b.rotAxis.z);
    	double xt = b.x.x/w;
    	double yt = b.x.y/h;
    	double zt = 0.0;
    	gl.glPushMatrix();
    	gl.glRotated(b.thed, b.rotAxis.x, b.rotAxis.y, b.rotAxis.z);
		gl.glTranslated( xt, -yt,zt);
//    	Vector3d tl = new Vector3d (b.x.x,b.x.y,b.x.z);
//    	tl.scale(h/b.imh);
//    	gl.glTranslated(tl.x,tl.y,tl.z);
		float yfact= (float)(b.height/b.width);
		float zfact= (float)(b.depth/b.width);
		if (b.depth==0.0) zfact = 0.5f;
	    gl.glScalef(1.0f, yfact, zfact);
		
	   drawBox(d);
	   gl.glPopMatrix();
	}
	
	public void init(GLAutoDrawable d) {
	    GL2 gl = d.getGL().getGL2();
	    gl.glEnable(GL2.GL_DEPTH_TEST);
	  }
	  
	private void drawBox(GLAutoDrawable d) {
	    GL2 gl = d.getGL().getGL2();  
	    Block bl = b.blocks.get(0);
	    ArrayList<Point3d> vertex = b.vertex;
	    gl.glColor4d(bl.c.x,bl.c.y,bl.c.z,bl.alpha);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f(-1.0f, 1.0f, 1.0f);
	    gl.glVertex3f(-1.0f,-1.0f, 1.0f);
	    gl.glVertex3f( 1.0f,-1.0f, 1.0f);
	    gl.glVertex3f( 1.0f, 1.0f, 1.0f);
	    gl.glEnd();
	    gl.glColor4d(bl.c.x,bl.c.y,bl.c.z,bl.alpha);
//	    gl.glColor3f( 1.0f, 1.0f,0.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f( 1.0f, 1.0f,-1.0f);
	    gl.glVertex3f( 1.0f,-1.0f,-1.0f);
	    gl.glVertex3f(-1.0f,-1.0f,-1.0f);
	    gl.glVertex3f(-1.0f, 1.0f,-1.0f);
	    gl.glEnd();
	    gl.glColor4d(bl.c.x,bl.c.y,bl.c.z,bl.alpha);
//	    gl.glColor3f( 1.0f, 0.0f,0.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f( 1.0f, 1.0f, 1.0f);
	    gl.glVertex3f( 1.0f,-1.0f, 1.0f);
	    gl.glVertex3f( 1.0f,-1.0f,-1.0f);
	    gl.glVertex3f( 1.0f, 1.0f,-1.0f);
	    gl.glEnd();
	    gl.glColor4d(bl.c.x,bl.c.y,bl.c.z,bl.alpha);
//	    gl.glColor3f(1.0f, 0.0f,1.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f(-1.0f, 1.0f,-1.0f);
	    gl.glVertex3f(-1.0f,-1.0f,-1.0f);
	    gl.glVertex3f(-1.0f,-1.0f, 1.0f);
	    gl.glVertex3f(-1.0f, 1.0f, 1.0f);
	    gl.glEnd();
	    gl.glColor4d(bl.c.x,bl.c.y,bl.c.z,bl.alpha);
//	    gl.glColor3f(0.0f, 1.0f,1.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f( 1.0f, 1.0f,-1.0f);
	    gl.glVertex3f(-1.0f, 1.0f,-1.0f);
	    gl.glVertex3f(-1.0f, 1.0f, 1.0f);
	    gl.glVertex3f( 1.0f, 1.0f, 1.0f);
	    gl.glEnd();
	    gl.glColor4d(bl.c.x,bl.c.y,bl.c.z,bl.alpha);
//	    gl.glColor3f(0.0f, 0.0f,1.0f);
	    gl.glBegin(GL2.GL_POLYGON);
	    gl.glVertex3f( 1.0f, -1.0f, 1.0f);
	    gl.glVertex3f(-1.0f, -1.0f, 1.0f);
	    gl.glVertex3f(-1.0f, -1.0f,-1.0f);
	    gl.glVertex3f( 1.0f, -1.0f,-1.0f);  
	    gl.glEnd();
	}
}
