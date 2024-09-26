package org.firstinspires.ftc.teamcode;

public class Vector {

    public double x;
    public double y;
    public double z;
    public double mag;
    public double roll;
    public double pitch;
    public double yaw;

    private Vector(double xIn, double yIn){
        x = xIn;
        y = yIn;
        mag = Math.hypot(x,y);
        yaw = Math.atan2(y,x);
        z = 0;
    }
    private Vector(double xIn, double yIn, double zIn){
        x = xIn;
        y = yIn;
        z = zIn;
        mag = Math.hypot(Math.hypot(x,y),z);
        yaw = Math.atan2(y,x);
        pitch = Math.atan2(z,y);
    }
    public static Vector xy(double x, double y){
        return new Vector(x,y);
    }
    public static Vector polar (double r, double theta){
        return new Vector(r * Math.cos(theta), r * Math.sin(theta));
    }
    public static Vector xyz(double x, double y, double z){
        return new Vector(x, y, z);
    }
    public static Vector polar3D(double r, double bear, double pit){
        double x = r*Math.cos(bear)*Math.cos(pit);
        double y = r*Math.sin(bear)*Math.cos(pit);
        double z = r*Math.sin(pit);
        return new Vector(x, y, z);
    }

}
