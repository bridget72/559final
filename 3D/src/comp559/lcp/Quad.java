package comp559.lcp;
import java.nio.FloatBuffer;

import java.nio.ShortBuffer;

import javax.vecmath.Color3f;
import java.util.ArrayList;
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GL4;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLException;
import com.jogamp.opengl.util.GLBuffers;
import com.jogamp.opengl.util.glsl.ShaderCode;
import com.jogamp.opengl.util.glsl.ShaderProgram;
import com.sun.scenario.effect.Color4f;
import javax.vecmath.Point3d;

import comp559.lcp.RigidBody;
public class Quad {
	public Color3f c= new Color3f();
	public float alpha = 0;
	public ArrayList<Point3d> vertex;
	
	public void set(Color3f c, float alpha, ArrayList<Point3d> vertex) {
		this.c.set(c);
		this.alpha=alpha;
		this.vertex = vertex;
	}
	public void draw( GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();
        gl.glColor4f( c.x, c.y, c.z, alpha );
        gl.glBegin(GL.GL_TRIANGLE_STRIP);
        gl.glVertex3d(vertex.get(0).x,vertex.get(0).y,vertex.get(0).z);
        gl.glVertex3d(vertex.get(1).x,vertex.get(1).y,vertex.get(1).z);
        gl.glVertex3d(vertex.get(2).x,vertex.get(2).y,vertex.get(2).z);
        gl.glVertex3d(vertex.get(3).x,vertex.get(3).y,vertex.get(3).z);
        gl.glEnd();
	}
}
