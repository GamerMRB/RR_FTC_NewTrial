package org.firstinspires.ftc.teamcode;

public class Vec2 {

    public static final Vec2 i = new Vec2(1, 0);
    public static final Vec2 j = new Vec2(0, 1);

    public final double x;
    public final double y;

    private boolean magCalculated = false;
    private double mag;

    private boolean angleCalculated = false;
    private double angle;

    private Vec2(double xIn, double yIn){
        x = xIn;
        y = yIn;
    }
    public static Vec2 xy(double x, double y){
        return new Vec2(x, y);
    }
    public static Vec2 yx(double y, double x){
        return new Vec2(x, y);
    }
    public static Vec2 polar(double r, double theta){
        return new Vec2(r * Math.cos(theta), r * Math.sin(theta));
    }
    public Vec2 yx(){
        return Vec2.xy(y, x);
    }


    public double mag(){
        if(!magCalculated){
            mag = Math.sqrt(x*x + y*y);
            magCalculated = true;
        }
        return mag;
    }

    public double angle(){
        if(!angleCalculated){
            angle = Math.atan2(y, x);
            angleCalculated = true;
        }
        return angle;
    }

    public Vec2 unit(){
        return Vec2.xy(x, y).div(mag());
    }
    public Vec2 perp(){
        return Vec2.xy(-y, x);
    }
    public Vec2 add(Vec2 v){
        return Vec2.xy(x + v.x, y + v.y);
    }
    public Vec2 add(double vx, double vy){
        return Vec2.xy(x + vx, y + vy);
    }
    public Vec2 add(double s){
        return Vec2.xy(x + s, y + s);
    }
    public Vec2 sub(Vec2 v){
        return Vec2.xy(x - v.x, y - v.y);
    }
    public Vec2 sub(double vx, double vy){
        return Vec2.xy(x - vx, y - vy);
    }
    public Vec2 sub(double s){
        return Vec2.xy(x - s, y - s);
    }
    public Vec2 mult(Vec2 v){
        return Vec2.xy(x * v.x, y * v.y);
    }
    public Vec2 mult(double vx, double vy){
        return Vec2.xy(x * vx, y * vy);
    }
    public Vec2 mult(double s){
        return Vec2.xy(x * s, y * s);
    }
    public Vec2 div(Vec2 v){
        return Vec2.xy(x / v.x, y / v.y);
    }
    public Vec2 div(double vx, double vy){
        return Vec2.xy(x / vx, y / vy);
    }
    public Vec2 div(double s){
        return Vec2.xy(x / s, y / s);
    }
    public double dot(Vec2 v){
        return x * v.x + y * v.y;
    }
    public double dot(double vx, double vy){
        return x * vx + y * vy;
    }
    public double dot(double s){
        return x * s + y * s;
    }
    public double cross(Vec2 v){
        return x * v.y - y * v.x;
    }
    public double cross(double vx, double vy){
        return x * vy - y * vx;
    }
    public double cross(double s){
        return x * s - y * s;
    }
    public Vec3 crossV3(Vec2 v){
        return Vec3.xyz(0, 0, x * v.y - y * v.x);
    }
    public Vec3 xy0(){
        return Vec3.xyz(x, y, 0);
    }
    public double angle(Vec2 v){
        return dot(v)/mag()/v.mag();
    }
    public double angle(double vx, double vy){
        return dot(vx, vy)/mag()/Math.sqrt(vx*vx + vy*vy);
    }
    public double angle(double s){
        return dot(s, s)/mag()/(Math.sqrt(2) * s);
    }
}
