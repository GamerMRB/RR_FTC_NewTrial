package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

public class DriveEasier extends DriveBetter {
    public DriveEasier(double x, double y){
        super(new LinearPath(Vec2.xy(x, y), 1500, 1000));
    }
}