package org.firstinspires.ftc.teamcode.dynamicsLibrary;

public class PID2 {
    public double p;
    public double i;
    public double d;
    public Vec2 eLast = Vec2.zero;
    public Vec2 eDiff = Vec2.zero;
    public Vec2 eInt = Vec2.zero;
    public boolean skipDiff = true;

    public PID2(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }
    public void update(Vec2 e, double dt){
        eInt = eInt.add(e.mult(dt));
        if(skipDiff){
            skipDiff = false;
        }
        else{
            eDiff = e.sub(eLast).div(dt);
        }
        eLast = e;
    }
    public Vec2 getPow(){
        return eLast.mult(p).add(eInt.mult(i)).add(eDiff.mult(d));
    }
    public void reset(){
        eLast = Vec2.zero;
        eDiff = Vec2.zero;
        eInt = Vec2.zero;
        skipDiff = true;
    }
}