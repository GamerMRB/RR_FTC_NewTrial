package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

public class LinearPath extends Path {
    Vec2 diff;
    double velocity;
    double acceleration;

    double accelerateTime;
    double accelerateDist;
    double coastTime;
    double coastDist;

    public LinearPath(Vec2 diff, double velocity, double acceleration){
        this.diff = diff;
        this.velocity = velocity;
        this.acceleration = acceleration;


//        m * v^2 / 2 = m * a * d / 2
        double maxVel = Math.min(velocity, Math.sqrt(acceleration * diff.mag()));
        accelerateTime = maxVel / acceleration;
        accelerateDist = acceleration * Math.pow(accelerateTime, 2) / 2;
        coastDist = diff.mag() - 2 * accelerateDist;
        coastTime = coastDist / velocity;
    }

    public Vec2 pos(double t) {
        double distance;
        if (t < accelerateTime){
            distance = acceleration * Math.pow(t, 2) / 2;
        } else if (t < accelerateTime + coastTime) {
            distance = accelerateDist + velocity * (t - accelerateTime);
        } else if (t < 2 * accelerateTime + coastTime) {
            distance = diff.mag() - acceleration * Math.pow(t - (2*accelerateTime + coastTime), 2) / 2;
        }else{
            distance = diff.mag();
        }
        return diff.unit().mult(distance);
    }

    public Vec2 vel(double t) {
        return diff.unit().mult(Math.max(0, Math.min(Math.min(acceleration * t, - acceleration * (t - (2 * accelerateTime + coastTime))), velocity)));
    }
}
