package org.firstinspires.ftc.teamcode.dynamicsLibrary;

public class Position {
    final public Vec2 disp;
    final public double rot;
    final public Vec2 dirInternal;
    public Position(Vec2 disp, double rot){
        this.disp = disp;
        this.rot = rot;
        dirInternal = Vec2.polar(1, rot);
    }
    public static Position xy(double x, double y){
        return new Position(Vec2.xy(x, y), 0);
    }
    public static Position v(Vec2 pos){
        return new Position(pos, 0);
    }
    public static Position xr(double x, double r){
        return new Position(Vec2.xy(x, 0), r);
    }
    public static Position yr(double y, double r){
        return new Position(Vec2.xy(0, y), r);
    }
    public static Position xyr(double x, double y, double r){
        return new Position(Vec2.xy(x, y), r);
    }
    public static Position vr(Vec2 pos, double r){
        return new Position(pos, r);
    }
    public Vec2 dir(){
        return dirInternal;
    }
    public Position append(Position pos){
        return Position.vr(disp.add(pos.disp.rotate(rot)), rot + pos.rot);
    }
    public Position remove(Position pos){
        return Position.vr(disp.sub(pos.disp.rotate(rot - pos.rot)), rot - pos.rot);
    }
    public Position difference(Position pos){
        return Position.vr(disp.sub(pos.disp), rot - pos.rot);
    }
}
