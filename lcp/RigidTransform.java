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
    public void set( double theta, int direction, Tuple3d p ) {
    	
        double c = Math.cos(theta);
        double s = Math.sin(theta);
        
        if(direction == 0){
        	//rotation about x
        	T.m00 = 1; T.m01 =  0; T.m02 = 0;   T.m03 = p.x;
            T.m10 = 0; T.m11 =  c; T.m12 = -s;  T.m13 = p.y;
            T.m10 = 0; T.m11 =  s; T.m12 = c;   T.m33 = p.z;
            T.m30 = 0; T.m31 =  0; T.m33 = 0;   T.m33 = 1;
        }else if(direction == 1){
        	//rotation about y 
        	T.m00 = c; T.m01 =  0; T.m02 = s;   T.m03 = p.x;
            T.m10 = 0; T.m11 =  1; T.m12 = 0;   T.m13 = p.y;
            T.m10 = -s;T.m11 =  0; T.m12 = c;   T.m33 = p.z;
            T.m30 = 0; T.m31 =  0; T.m33 = 0;   T.m33 = 1;
        }else if(direction == 2){
        	//rotation about z 
        	T.m00 = c; T.m01 = -s; T.m02 = 0;   T.m03 = p.x;
            T.m10 = s; T.m11 =  c; T.m12 = 0;   T.m13 = p.y;
            T.m10 = 0; T.m11 =  0; T.m12 = 1;   T.m33 = p.z;
            T.m30 = 0; T.m31 =  0; T.m33 = 0;   T.m33 = 1;
        }
        
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
