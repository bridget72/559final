package comp559.lcp;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
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
     * @param p
     */
    public void set( Vector3d theta, Tuple3d p, Matrix3d th) {
    	Matrix3d rot = getRotation(theta,th);
        T.set(rot);
        
        T.m03 = p.x;
        T.m13 = p.y;
        T.m23 = p.z;
        T.m33 = 1;
        //not sure
//        T.mul(Tx,Ty);
//        T.mul(T,Tz);

    }
    public Matrix3d getRotation(Vector3d theta, Matrix3d th) {
    	double cx = Math.cos(theta.x);
        double sx = Math.sin(theta.x);
        
        double cy = Math.cos(theta.y);
        double sy = Math.sin(theta.y);
        
        double cz = Math.cos(theta.z);
        double sz = Math.sin(theta.z);

        Matrix3d rot = new Matrix3d (1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0);
//        double sx = Math.sin(theta.x);
        Matrix3d sin = new Matrix3d(sx,0.0,0.0,sy,0.0,0.0,sz,0.0,0.0);
        Matrix3d cos = new Matrix3d(1-cx,0.0,0.0,1-cy,0.0,0.0,1-cz,0.0,0.0);
        Matrix3d second = new Matrix3d();
        second.set(th);
        second.mul(sin);
        rot.add(second);
        second.set(th);
        second.mul(th);
        second.mul(cos);
        rot.add(second);
        return rot;
    }
    public double getTheta (Vector3d theta, Matrix3d th) {
    	Matrix3d rotation = getRotation (theta, th);
    	double thed = rotation.m00+rotation.m11+rotation.m22;
    	System.out.println("trace value is "+thed);
    	thed = ((thed%180-1)/2)/180*Math.PI;
    	System.out.println("angle in acos transferred to radian is "+thed);
    	thed = Math.acos(thed);
    	return thed;
    }
    /**
     * Inverts this transformation
     */
    public void invert() {
        // gross but convenient... 
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
        result.x = T.m00 * p.x + T.m01 * p.y +T.m02*p.z;
        result.y = T.m10 * p.x + T.m11 * p.y +T.m12*p.z;
        result.z = T.m20 * p.x + T.m21 * p.y +T.m22*p.z;
    }

}
