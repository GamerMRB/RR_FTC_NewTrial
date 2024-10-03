package org.firstinspires.ftc.teamcode;

public class Vec3 {
    public static final Vec3 i = new Vec3(1, 0, 0);
    public static final Vec3 j = new Vec3(0, 1, 0);
    public static final Vec3 k = new Vec3(0, 0, 1);

    public final double x;
    public final double y;
    public final double z;

    private boolean magCalculated = false;
    private double mag;

    private boolean yawCalculated = false;
    private double yaw;

    private boolean pitchCalculated = false;
    private double pitch;

    private Vec3(double xIn, double yIn, double zIn){
        x = xIn;
        y = yIn;
        z = zIn;
    }
    public static Vec3 xyz(double x, double y, double z){
        return new Vec3(x, y, z);
    }
    public static Vec3 v2z(Vec2 xy, double z){
        return new Vec3(xy.x, xy.y, z);
    }
    public static Vec3 polar(double r, double yaw, double pitch){
        return new Vec3(r * Math.cos(pitch) * Math.cos(yaw), r * Math.cos(pitch) * Math.sin(yaw), r * Math.sin(pitch));
    }
    public double mag(){
        if(!magCalculated){
            mag = Math.sqrt(x*x + y*y + z*z);
            magCalculated = true;
        }
        return mag;
    }
    public double yaw(){
        if(!yawCalculated){
            yaw = Math.atan2(y, x);
            yawCalculated = true;
        }
        return yaw;
    }
        public double pitch(){
        if(!pitchCalculated){
            pitch = Math.atan2(z, Math.sqrt(x*x+y*y));
            pitchCalculated = true;
        }
        return pitch;
    }
    public Vec3 unit(){
        return Vec3.xyz(x, y, z).div(mag());
    }
    public Vec3 add(Vec3 v){
        return Vec3.xyz(x + v.x, y + v.y, z + v.z);
    }
    public Vec3 add(double vx, double vy, double vz){
        return Vec3.xyz(x + vx, y + vy, z + vz);
    }
    public Vec3 add(double s){
        return Vec3.xyz(x + s, y + s, z+ s);
    }
    public Vec3 sub(Vec3 v){
        return Vec3.xyz(x - v.x, y - v.y, z - v.z);
    }
    public Vec3 sub(double vx, double vy, double vz){
        return Vec3.xyz(x - vx, y - vy, z - vz);
    }
    public Vec3 sub(double s){
        return Vec3.xyz(x - s, y - s, z - s);
    }
    public Vec3 mult(Vec3 v){
        return Vec3.xyz(x * v.x, y * v.y, z * v.z);
    }
    public Vec3 mult(double vx, double vy, double vz){
        return Vec3.xyz(x * vx, y * vy, z * vz);
    }
    public Vec3 mult(double s){
        return Vec3.xyz(x * s, y * s, z * s);
    }
    public Vec3 div(Vec3 v){
        return Vec3.xyz(x / v.x, y / v.y, z / v.z);
    }
    public Vec3 div(double vx, double vy, double vz){
        return Vec3.xyz(x / vx, y / vy, z/ vz);
    }
    public Vec3 div(double s){
        return Vec3.xyz(x / s, y / s, z / s);
    }
    public double dot(Vec3 v){
        return x * v.x + y * v.y + z * v.z;
    }
    public double dot(double vx, double vy, double vz){
        return x * vx + y * vy + z * vz;
    }
    public double dot(double s){
        return x * s + y * s + z * s;
    }
    public Vec3 cross(Vec3 v){
        return Vec3.xyz(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.z);
    }
    public Vec3 cross(double vx, double vy, double vz){
        return Vec3.xyz(y * vz - z * vy, z * vx - x * vz, x * vy - y * vz);
    }
    public double angle(Vec3 v){
        return dot(v)/mag()/v.mag();
    }
    public double angle(double vx, double vy, double vz){
        return dot(vx, vy, vz)/mag()/Math.sqrt(vx*vx + vy*vy + vz*vz);
    }
    public double angle(double s){
        return dot(s, s, s)/mag()/(Math.sqrt(3) * s);
    }
    public Vec3 proj(Vec3 v){
        return (v.mult(this.dot(v)/ dot(this)));
    }
}
