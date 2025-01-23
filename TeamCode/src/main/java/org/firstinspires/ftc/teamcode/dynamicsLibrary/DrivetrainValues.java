package org.firstinspires.ftc.teamcode.dynamicsLibrary;

public class DrivetrainValues {
    final double fl;
    final double fr;
    final double bl;
    final double br;

    public DrivetrainValues(double fl, double fr, double bl, double br){
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }
    public DrivetrainValues(Position pos){
        fl = pos.disp.x + pos.disp.y - pos.angle;
        fr = pos.disp.x - pos.disp.y + pos.angle;
        bl = pos.disp.x - pos.disp.y - pos.angle;
        br = pos.disp.x + pos.disp.y + pos.angle;
    }

    public DrivetrainValues sub(DrivetrainValues values){
        return new DrivetrainValues(
                fl - values.fl,
                fr - values.fr,
                bl - values.bl,
                br - values.br
        );
    }
    public DrivetrainValues scale(double scale){
        return new DrivetrainValues(
                scale*fl,
                scale*fr,
                scale*bl,
                scale*br
        );
    }

    public double mag(){
        return Math.abs(fl) + Math.abs(fr) + Math.abs(bl) + Math.abs(br);
    }

    public DrivetrainValues normalize(){
        return scale(1/mag());
    }

    public Position toPosition(){
        return Position.xya(
                (fl + fr + bl + br) / 4,
                (fl - fr - bl + br) / 4,
                (-fl + fr - bl + br) / 4
        );
    }
}
