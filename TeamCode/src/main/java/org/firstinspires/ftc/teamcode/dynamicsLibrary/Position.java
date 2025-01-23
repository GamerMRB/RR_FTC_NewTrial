package org.firstinspires.ftc.teamcode.dynamicsLibrary;

public class Position {
    final public static Position zero = Position.xya(0, 0, 0);

    final public Vec2 disp;
    final public double angle;
    final public Vec2 dirInternal;
    Position(Vec2 disp, double angle){
        this.disp = disp;
        this.angle = angle;
        dirInternal = Vec2.polar(1, angle);
    }
    public static Position xy(double x, double y){
        return new Position(Vec2.xy(x, y), 0);
    }
    public static Position v(Vec2 pos){
        return new Position(pos, 0);
    }
    public static Position a(double a){
        return new Position(Vec2.zero, a);
    }
    public static Position xr(double x, double a){
        return new Position(Vec2.xy(x, 0), a);
    }
    public static Position ya(double y, double a){
        return new Position(Vec2.xy(0, y), a);
    }
    public static Position xya(double x, double y, double a){
        return new Position(Vec2.xy(x, y), a);
    }
    public static Position va(Vec2 pos, double a){
        return new Position(pos, a);
    }
    public Vec2 dir(){
        return dirInternal;
    }
    public Position append(Position pos){
        return Position.va(disp.add(pos.disp.rotate(angle)), angle + pos.angle);
    }
    public Position subtract(Position pos){
        return Position.va(disp.sub(pos.disp.rotate(angle - pos.angle)), angle - pos.angle);
    }
    public Position difference(Position pos){
        return Position.va(disp.sub(pos.disp), angle - pos.angle);
    }
    public String toString(){
        return String.format("%s, %s", disp, angle);
    }
}
