package org.firstinspires.ftc.teamcode;

import java.util.Objects;

public class AutoThread extends Thread {
    SentinelPrime op = new SentinelPrime();
    public AutoThread(double changeY, double directionY, double changeX, double directionX, double changeHeading, double directionHeading, double changePivot, double directionPivot, int changeClaw, double changeSlide, double directionSlide, double changeStrafe, double directionStrafe, String team) {
        this.op.CHANGE_Y = changeY * 25.4;
        this.op.DIRECTION_Y = directionY;

        this.op.CHANGE_X = changeX * 25.4;
        this.op.DIRECTION_X = directionX;

        this.op.CHANGE_HEADING = changeHeading;
        this.op.DIRECTION_HEADING = directionHeading;

        this.op.CHANGE_PIVOT = changePivot * 25.4;
        this.op.DIRECTION_PIVOT = directionPivot;

        this.op.CLAW_OPEN = changeClaw;

        this.op.CHANGE_SLIDE = changeSlide * 25.4;
        this.op.DIRECTION_SLIDE = directionSlide;

        this.op.CHANGE_STRAFE = changeStrafe * 25.4;
        this.op.DIRECTION_STRAFE = directionStrafe;

        this.op.TEAM_COLOR = team;
    }

    @Override
    public void run() {
        Thread yThread = new Thread(() -> {
            if (op.CHANGE_Y != 0 & op.DIRECTION_Y != 0) {
                if (op.DIRECTION_Y == 1) { //North
                    op.yController(op.CHANGE_Y, 2,2,2,2);
                } else if (op.DIRECTION_Y == 2) { //Northeast
                    op.yController(op.CHANGE_Y, 1,1,-1,-1);
                } else if (op.DIRECTION_Y == 4) { //Southeast
                    op.yController(op.CHANGE_Y, -1,-1,1,1);
                } else if (op.DIRECTION_Y == 5) { //South
                    op.yController(op.CHANGE_Y, -2,-2,-2,-2);
                } else if (op.DIRECTION_Y == 6) { //Southwest
                    op.yController(op.CHANGE_Y, -1,-1,-1,-1);
                }  else if (op.DIRECTION_Y == 8) { //Northwest
                    op.yController(op.CHANGE_Y, 1,1,1,1);
                }

            }
        });

        Thread xThread = new Thread(() -> {
            if (op.CHANGE_X != 0 & op.DIRECTION_X != 0) {
                if (op.DIRECTION_X == 2) { //Northeast
                    op.xController(op.CHANGE_X, 1,1,1,1);
                } else if (op.DIRECTION_X == 3) { //East
                    op.xController(op.CHANGE_X, 2,2,2,2);
                } else if (op.DIRECTION_X == 4) { //Southeast
                    op.xController(op.CHANGE_X, -1,1,1,-1);
                } else if (op.DIRECTION_X == 6) { //Southwest
                    op.xController(op.CHANGE_X, -1,-1,-1,-1);
                } else if (op.DIRECTION_X == 7) { //West
                    op.xController(op.CHANGE_X, -2,-2,-2,-2);
                } else if (op.DIRECTION_X == 8) { //Northwest
                    op.xController(op.CHANGE_X, 1,1,1,1);
                }
            }
        });

        Thread headingThread = new Thread(() -> {
            if (op.CHANGE_HEADING != 0 & op.DIRECTION_HEADING != 0) {
                op.turn(op.CHANGE_HEADING, op.DIRECTION_HEADING);
            }
        });

        Thread pivotThread = new Thread(() -> {
            if (op.CHANGE_PIVOT != 0 & op.DIRECTION_PIVOT != 0) {
                op.pivot(op.CHANGE_PIVOT, op.DIRECTION_PIVOT);
            }
        });

        Thread clawThread = new Thread(() -> {
            if (op.CLAW_OPEN == 1 | op.CLAW_OPEN == 2) {
                op.claw(op.CLAW_OPEN);
            }
        });

        Thread slideThread = new Thread(() -> {
            if (op.CHANGE_SLIDE != 0 & op.DIRECTION_SLIDE != 0) {
                op.slide(op.CHANGE_SLIDE, op.DIRECTION_SLIDE);
            }
        });

        Thread strafeThread = new Thread(() -> {
            if (op.CHANGE_STRAFE != 0 & op.DIRECTION_STRAFE != 0) {
                if (op.DIRECTION_STRAFE == 2) { //Northeast
                    op.strafe(op.CHANGE_STRAFE, 1,-1,-1,1);
                } else if (op.DIRECTION_STRAFE == 3) { //East
                    op.strafe(op.CHANGE_STRAFE, 2,2,2,2);
                } else if (op.DIRECTION_STRAFE == 4) { //Southeast
                    op.strafe(op.CHANGE_STRAFE, -1,1,1,-1);
                } else if (op.DIRECTION_STRAFE == 6) { //Southwest
                    op.strafe(op.CHANGE_STRAFE, -1,-1,-1,-1);
                } else if (op.DIRECTION_STRAFE == 7) { //West
                    op.strafe(op.CHANGE_STRAFE, -2,-2,-2,-2);
                } else if (op.DIRECTION_STRAFE == 8) { //Northwest
                    op.strafe(op.CHANGE_STRAFE, 1,1,1,1);
                }
            }
        });

        Thread camThread = new Thread(() -> {
            if (!Objects.equals(op.TEAM_COLOR, "")) {
                op.camera(op.TEAM_COLOR);
            }
        });

        yThread.start();
        strafeThread.start();
        xThread.start();
        camThread.start();
        headingThread.start();
        pivotThread.start();
        slideThread.start();
        clawThread.start();


    }

}
