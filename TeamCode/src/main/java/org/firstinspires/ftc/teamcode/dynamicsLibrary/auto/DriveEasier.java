package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

public class DriveEasier extends DriveBetter {
    public DriveEasier(Vec2 diff){
        super(new LinearPath(diff, 1000, 1000));
    }
}