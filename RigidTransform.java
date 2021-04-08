package comp559.lcp;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

/**
 * 2D rigid transformation
 * @author kry
 */
public class RigidTransform {

    /** homogeneous representation of 3D transformation */
	
	Matrix4d T = new Matrix4d();
    
    /** 
     * Creates a new identity transformation 
     */
    public RigidTransform() {
        T.setIdentity();
    }
    
    /**
     * Sets this rigid transformation from another rigid transformation
     * @param R
     */
    public void set( RigidTransform R ) {
        T.set(R.T);
    }
    
    /**
     * Sets this transformation to be the given rotation and translation
     * (i.e., points will be transformed by rotation followed by translation)
     * @param theta
     * @param direction
     * @param p
     */
    public void set( Vector3d theta, Tuple3d p ) {
    	
        double cx = Math.cos(theta.x);
        double sx = Math.sin(theta.x);
        Matrix4d Tx = new Matrix4d();
        //rotation about x
        Tx.m00 = 1; Tx.m01 =  0;  Tx.m02 = 0;    Tx.m03 = p.x;
        Tx.m10 = 0; Tx.m11 =  cx; Tx.m12 = -sx;  Tx.m13 = p.y;
        Tx.m10 = 0; Tx.m11 =  sx; Tx.m12 = cx;   Tx.m33 = p.z;
        Tx.m30 = 0; Tx.m31 =  0;  Tx.m33 = 0;    Tx.m33 = 1;
        
        double cy = Math.cos(theta.y);
        double sy = Math.sin(theta.y);
        Matrix4d Ty = new Matrix4d();
        //rotation about y 
        Ty.m00 = cy; Ty.m01 =  0; Ty.m02 = sy;   Ty.m03 = p.x;
        Ty.m10 = 0;  Ty.m11 =  1; Ty.m12 = 0;    Ty.m13 = p.y;
        Ty.m10 = -sy;Ty.m11 =  0; Ty.m12 = cy;   Ty.m33 = p.z;
        Ty.m30 = 0;  Ty.m31 =  0; Ty.m33 = 0;    Ty.m33 = 1;
        
        double cz = Math.cos(theta.z);
        double sz = Math.sin(theta.z);
        Matrix4d Tz = new Matrix4d();
        //rotation about z 
        Tz.m00 = cz; Tz.m01 = -sz; Tz.m02 = 0;   Tz.m03 = p.x;
        Tz.m10 = sz; Tz.m11 =  cz; Tz.m12 = 0;   Tz.m13 = p.y;
        Tz.m10 = 0;  Tz.m11 =  0;  Tz.m12 = 1;   Tz.m33 = p.z;
        Tz.m30 = 0;  Tz.m31 =  0;  Tz.m33 = 0;   Tz.m33 = 1;
        
        //does it matter which rotation happens first?
        T.mul(Tx,Ty);
        T.mul(T,Tz);
        
        
    }
    
    /**
     * Inverts this transformation
     */
    public void invert() {
        // gross but convenient... 
    	// System.out.println(T);
        T.invert();
    }
    
    /**
     * Transforms the given point
     * @param p
     */
    public void transform( Point3d p ) {
        double x = T.m00 * p.x + T.m01 * p.y + T.m02 * p.z + T.m03;
        double y = T.m10 * p.x + T.m11 * p.y + T.m12 * p.z + T.m13;
        double z = T.m20 * p.x + T.m21 * p.y + T.m22 * p.z + T.m23;
        p.x = x;
        p.y = y;
        p.z = z;
    }
 
    /**
     * Transforms the given vector
     * @param p
     */
    public void transform( Vector3d p ) {
        double x = T.m00 * p.x + T.m01 * p.y + T.m02 * p.z;
        double y = T.m10 * p.x + T.m11 * p.y + T.m12 * p.z;
        double z = T.m20 * p.x + T.m21 * p.y + T.m22 * p.z;
        p.x = x;
        p.y = y;
        p.z = z;
    }

    /**
     * Transforms the given point
     * @param p
     * @param result
     */
    public void transform( Point3d p, Point3d result ) {
        result.x = T.m00 * p.x + T.m01 * p.y + T.m02 * p.z + T.m03;
        result.y = T.m10 * p.x + T.m11 * p.y + T.m12 * p.z + T.m13;
        result.z = T.m20 * p.x + T.m21 * p.y + T.m22 * p.z + T.m23;
    }
 
    /**
     * Transforms the given vector
     * @param p
     * @param result
     */
    public void transform( Vector3d p, Vector3d result ) {
        result.x = T.m00 * p.x + T.m01 * p.y + T.m02 * p.z;
        result.y = T.m10 * p.x + T.m11 * p.y + T.m12 * p.z;
        result.z = T.m20 * p.x + T.m21 * p.y + T.m22 * p.z;
    }

}
