package org.firstinspires.ftc.teamcode;

public class PID1 {
    public double p;
    public double i;
    public double d;
    public double eLast = 0;
    public double eDiff = 0;
    public double eInt = 0;
    public boolean skipDiff = true;

    public PID1(double p, double i, double d){
       this.p = p;
       this.i = i;
       this.d = d;
    }
    public void update(double e, double dt){
        eInt += e * dt;
        if(skipDiff){
            skipDiff = false;
        }
        else{
            eDiff = (e - eLast) / dt;
        }
        eLast = e;
    }
    public double getPow(){
        return p * eLast + i * eInt + d * eDiff;
    }
    public void reset(){
        eLast = 0;
        eDiff = 0;
        eInt = 0;
        skipDiff = true;
    }
}
