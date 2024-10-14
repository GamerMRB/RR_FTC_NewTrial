package org.firstinspires.ftc.teamcode;

public class Position {
    final public Vec2 disp;
    final public double rot;
    final public Vec2 dirInternal;
    public Position(Vec2 dispIn, double rotIn){
        disp = dispIn;
        rot = rotIn;
        dirInternal = Vec2.polar(1, rot);
    }
    public Vec2 dir(){
        return dirInternal;
    }
    public Position add(Position pos){
        return new Position(disp.add(pos.disp.rotate(rot)), rot + pos.rot);
    }
    public Position sub(Position pos){
        return new Position(disp.sub(pos.disp.rotate(rot - pos.rot)), rot - pos.rot);
    }
}
