package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;


import java.util.ArrayList;
import java.util.Objects;

//@SuppressWarnings("unused")
@Autonomous
public class SentinelPrime extends UscOpMode {
    static final double COUNTS_PER_MOTOR_REV = 538;
    static final double WHEEL_DIAMETER_MM = 96;
    static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER_MM * Math.PI);
    static final int FILTER_SIZE = 25;
    static final double TOLERANCE = 25.4;
    static final double COUNTS_PER_RADIAN = COUNTS_PER_MOTOR_REV/(2 * Math.PI);

    double[] headingFilter = new double[FILTER_SIZE];
    double[] xFilter = new double[FILTER_SIZE];
    double[] yFilter = new double[FILTER_SIZE];
    double[] armFilter = new double[FILTER_SIZE];
    double[] armFilter1 = new double[FILTER_SIZE];

    int filterIndex = 0;

    double posX, posY, heading, armDistance, armLength = 0;
    double P = 1; //Estimate error covariance
    double Q = 0.1; // Process noise covariance
    double R = 0.1; // Measurement noise covariance

    double avgHeading, avgX, avgY, avgArmDistance, avgArmLength = 0;

    public double CHANGE_Y, CHANGE_X ,CHANGE_HEADING ,CHANGE_PIVOT, CHANGE_SLIDE, CHANGE_STRAFE = 1;
    public double DIRECTION_Y, DIRECTION_X ,DIRECTION_HEADING ,DIRECTION_PIVOT, DIRECTION_SLIDE, DIRECTION_STRAFE = 0;
    int CLAW_OPEN = 0;
    String TEAM_COLOR = "";
    double directionLeft, directionRight, directionBackLeft, directionBackRight = 0;

    ArrayList<Double> getHeading, getX, getY, getArmLength, getArmDistance = new ArrayList<>();

    boolean stopFlag = false;

    @Override
    public void runOpMode() throws InterruptedException {
        setUpHardware();

        //Encoder setup
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while(opModeIsActive()) {
            Thread setUpThread = new Thread(() -> {
                ////Setup

                //Encoders
                double frontRightDistance = frontRight.getCurrentPosition() / COUNTS_PER_MM;
                armDistance = armPivot.getCurrentPosition() / COUNTS_PER_RADIAN;
                armLength = armSlide.getCurrentPosition() / COUNTS_PER_RADIAN;

                //IMU data
                Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double imuHeading = angles.firstAngle;

                // Measurements: [posX, posY, heading]
                double[] measurement = new double[5];
                measurement[0] = frontRightDistance * Math.cos(Math.toRadians(imuHeading)); // X Position
                measurement[1] = frontRightDistance * Math.sin(Math.toRadians(imuHeading)); // Y Position
                measurement[2] = imuHeading;
                measurement[3] = armDistance;
                measurement[4] = armLength;

                //Kalman filter updates
                double[] state = {posX, posY, heading, armDistance, armLength};
                kalmanUpdate(state, measurement);

                //Update state variables
                posX = state[0];
                posY = state[1];
                heading = state[2];
                armDistance = state[3];
                armLength = state[4];

                //Application
                headingFilter[filterIndex] = heading;
                xFilter[filterIndex] = posX;
                yFilter[filterIndex] = posY;
                armFilter[filterIndex] = armDistance;
                armFilter1[filterIndex] = armLength;

                for (int i = 0; i < FILTER_SIZE; i++) {
                    avgHeading += headingFilter[i];
                    avgX += xFilter[i];
                    avgY += yFilter[i];
                    avgArmDistance += armFilter[i];
                    avgArmLength += armFilter1[i];
                }

                avgHeading = (avgHeading * 180) / (FILTER_SIZE * Math.PI);
                avgX /= FILTER_SIZE;
                avgY /= FILTER_SIZE;
                avgArmDistance /= FILTER_SIZE;
                avgArmLength /= FILTER_SIZE;

                //To accurately represent change when the methods are called upon
                getHeading.add(avgHeading);
                getX.add(avgX);
                getY.add(avgY);
                getArmLength.add(avgArmLength);
                getArmDistance.add(avgArmDistance);
            });


            ////Code (Sample, alterations TBD 1/6/25)

            setUpThread.start();

            //First put the specimen on highest bar
            //Y,x,heading,pivot,claw,slide,strafe,camera
            threadRipper(7,1,0,0,0,0,10,1,0,0,0,0,0,"");
            threadRipper(7,5,0,0,0,0,-10,1,2,0,0,0,0,"");

            //Next, go over to team member
            strafe(0,1,1,1,1);
            turn(180,1);

            //Pick up specimen and score on top bar
            threadRipper(0,0,0,0,0,0,-10,-1,2,8,1,0,0,"");
            threadRipper(0,0,0,0,0,180,10,1,0,0,0,0,0,"");
            threadRipper(15,8,0,0,0,0,0,0,0,0,0,-24,8,"");
            threadRipper(10,5,0,0,0,0,-10,-1,1,8,-1,0,0,"");

            //Next, go pick up neutral sample and give to team member
            threadRipper(0,0,0,0,0,0,-8,-1,2,4,1,12,-1,"");
            threadRipper(0,0,0,0,180,1,8,1,0,0,0,0,0,"");
            threadRipper(24,1,0,0,0,0,0,0,1,0,0,0,0,"");

            strafe(12,1,1,-1,-1);
            strafe(12,-1,-1,1,1);

            //Pick up specimen and score on top bar
            threadRipper(6,1,0,0,0,0,8,1,2,0,0,0,0,"");

            turn(180,1);

            threadRipper(18,8,0,0,0,0,8,-1,1,0,0,24,8,"");

            //Next, go pick up 2nd neutral sample and give to team member
            threadRipper(-12,5,0,0,0,0,-8,-1,0,8,-1,24,8,"");

            claw(2);

            threadRipper(0,0,0,0,180,1,-8,-1,0,0,0,0,0,"");
            threadRipper(12,1,0,0,0,0,8,1,1,0,0,0,0,"");

            strafe(12,1,1,-1,-1);
            strafe(12,-1,-1,1,1);

            //Pick up specimen and score on top bar
            threadRipper(6,0,0,0,0,0,-8,1,2,0,0,0,0,"");

            turn(180,1);

            threadRipper(12,8,0,0,0,0,-8,-1,1,0,0,-24,8,"");

            //Next, go pick up 3rd neutral sample and give to team member
            threadRipper(-12,5,0,0,0,0,-8,-1,0,8,-1,24,8,"");

            claw(2);

            threadRipper(0,0,0,0,180,1,-8,-1,0,0,0,0,0,"");
            threadRipper(12,1,0,0,0,0,8,1,1,0,0,0,0,"");

            strafe(12,1,1,-1,-1);
            strafe(12,-1,-1,1,1);

            //Pick up specimen and score on top bar
            threadRipper(6,0,0,0,0,0,8,1,2,0,0,0,0,"");

            turn(180,1);

            threadRipper(12,8,0,0,0,0,-8,-1,1,0,0,-24,8,"");

            //Setup for color recognition
            threadRipper(6,1,0,0,0,0,-8,-1,0,0,0,0,0,"");

            //Go use color recognition for next sample
            threadRipper(0,0,0,0,0,0,0,0,0,0,0,24,1,"blue");

            //Pick up specimen and score on top bar
            threadRipper(-12,4, 0,0,0,0,8,1,0,0,0,-24,4,"");

            turn(180,1);

            threadRipper(8,1,0,0,0,0,0,0,1,0,0,0,0,"");

            strafe(12,1,1,-1,-1);
            strafe(12,-1,-1,1,1);

            threadRipper(6,1,0,0,0,0,8,1,2,0,0,0,0,"");

            turn(180,1);

            threadRipper(12,8,0,0,0,0,-8,-1,1,0,0,-24,8,"");
            threadRipper(-6,5,0,0,0,0,0,0,0,-5,-1,0,0,"");

            //Setup for color recognition
            threadRipper(6,1,0,0,0,0,-8,-1,0,0,0,0,0,"");

            //Go use color recognition for next sample
            threadRipper(0,0,0,0,0,0,0,0,0,0,0,24,1,"blue");

            //Pick up specimen and score on top bar
            threadRipper(-12,4, 0,0,0,0,8,1,0,0,0,-24,4,"");

            turn(180,1);

            threadRipper(8,1,0,0,0,0,0,0,1,0,0,0,0,"");

            strafe(12,1,1,-1,-1);
            strafe(12,-1,-1,1,1);

            threadRipper(6,1,0,0,0,0,8,1,2,0,0,0,0,"");

            turn(180,1);

            threadRipper(12,8,0,0,0,0,-8,-1,1,0,0,-24,8,"");
            threadRipper(-6,5,0,0,0,0,0,0,0,-5,-1,0,0,"");

            //Park
            threadRipper(0,0,0,0,0,0,0,0,0,0,0,-26,3,"");

            ////Telemetry
            telemetry();

        }


    }

    public void kalmanUpdate(@NonNull double[] state, double[] measurement) {
        //Calculate the Kalman gain which determines the weight given to the new measurement
        double K = P / (P + R);

        //Update each state element
        for (int i = 0; i < state.length; i++) {
            state[i] = state[i] + K * (measurement[i] - state[i]);
        }

        //Update the new error covariance to reflect the new uncertainty
        P = (1 - K) * P + Q;
    }

    // ***DISCLAIMER*** If CHANGE is positive, DIRECTION must be positive, but if CHANGE is negative, DIRECTION must be negative (EXCEPTION: HEADING)

    //y controller
    public void yController(double change, double directionLeft, double directionRight, double directionBackLeft, double directionBackRight) {
        this.CHANGE_Y = change * 25.4;
        this.directionLeft = directionLeft;
        this.directionRight = directionRight;
        this.directionBackLeft = directionBackLeft;
        this.directionBackRight = directionBackRight;

        frontLeft.setPower(this.directionLeft * 0.5);
        frontRight.setPower(this.directionRight * 0.5);
        backLeft.setPower(this.directionBackLeft * 0.5);
        backRight.setPower(this.directionBackRight * 0.5);

        if (Math.abs(avgY - (CHANGE_Y + getY.get(getY.size() - 1))) <= TOLERANCE) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    //x controller
    public void xController(double change, double directionLeft, double directionRight, double directionBackLeft, double directionBackRight) {
        this.CHANGE_X = change * 25.4;
        this.directionLeft = directionLeft;
        this.directionRight = directionRight;
        this.directionBackLeft = directionBackLeft;
        this.directionBackRight = directionBackRight;

        frontLeft.setPower(this.directionLeft * 0.5);
        frontRight.setPower(this.directionRight * 0.5);
        backLeft.setPower(this.directionBackLeft * 0.5);
        backRight.setPower(this.directionBackRight * 0.5);

        if (Math.abs(avgX - (CHANGE_X + getX.get(getX.size() - 1))) <= TOLERANCE) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    //heading, -Direction goes to the left, while +Direction goes to the right
    public void turn(double change, double direction) {
        this.CHANGE_HEADING = change;
        this.DIRECTION_HEADING = direction;

        int initGetX = getX.size();
        int initGetY = getY.size();

        getX.add(0.0);
        getY.add(0.0);

        frontLeft.setPower(-DIRECTION_HEADING);
        frontRight.setPower(DIRECTION_HEADING);
        backLeft.setPower(-DIRECTION_HEADING);
        backRight.setPower(DIRECTION_HEADING);
        if (Math.abs(avgHeading - (CHANGE_HEADING + getHeading.get(getHeading.size() - 1))) <= 4) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            getX.subList(initGetX, getX.size()).clear();
            getY.subList(initGetY, getY.size()).clear();
        }

        if (getX.get(getX.size() - 1) != 0) {
            avgX -= getX.get(getX.size() - 1);
            getX.set(getX.size() - 1, 0.0);
        }

        if (getY.get(getY.size() - 1) != 0) {
            avgY -= getY.get(getY.size() - 1);
            getY.set(getY.size() - 1, 0.0);
        }

    }

    //arm pivot controller; change = sin(theta) * armLength
    public void pivot(double change, double direction) {
        this.CHANGE_PIVOT = change * 25.4;
        this.DIRECTION_PIVOT = direction;

        armPivot.setPower(DIRECTION_PIVOT * 0.5);
        if (Math.abs(avgArmDistance - (CHANGE_PIVOT + getArmDistance.get(getArmDistance.size() - 1))) <= TOLERANCE) {
            armPivot.setPower(0);
        }
    }

    //claw controller
    public void claw(int claw) {
        this.CLAW_OPEN = claw;

        if (CLAW_OPEN == 1) {
            leftClaw.setPosition(LEFT_OPEN);
            rightClaw.setPosition(RIGHT_OPEN);
        } else if (CLAW_OPEN == 2) {
            leftClaw.setPosition(LEFT_CLOSE);
            rightClaw.setPosition(RIGHT_CLOSE);
        }

    }

    //arm controller
    public void slide(double change, double direction) {
        this.CHANGE_SLIDE = change * 25.4;
        this.DIRECTION_SLIDE = direction;

        armSlide.setPower(DIRECTION_SLIDE);
        if (Math.abs(avgArmLength - (CHANGE_SLIDE + getArmLength.get(getArmLength.size() - 1))) <= TOLERANCE) {
            armSlide.setPower(0);
        }
    }

    //strafe controller
    public void strafe(double change, double directionLeft, double directionRight, double directionBackLeft, double directionBackRight) {
        this.CHANGE_STRAFE = change * 25.4;
        this.directionLeft = directionLeft;
        this.directionRight = directionRight;
        this.directionBackLeft = directionBackLeft;
        this.directionBackRight = directionBackRight;

        frontLeft.setPower(this.directionLeft * 0.5);
        frontRight.setPower(this.directionRight * 0.5);
        backLeft.setPower(-this.directionBackLeft * 0.5);
        backRight.setPower(-this.directionBackRight * 0.5);

        if (Math.abs(avgX - (CHANGE_STRAFE + getX.get(getX.size() - 1))) <= TOLERANCE) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

    }

    //Color recognition using camera
    public void camera(String teamColor) {
        this.TEAM_COLOR = teamColor;

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        VideoCapture camera = new VideoCapture();

        Mat frame = new Mat();
        Scalar lowerColor = new Scalar(0,0,50); //Blue
        Scalar upperColor = new Scalar(0,0,175);

        Scalar lowerColor1 = new Scalar(50,0,0); //Red
        Scalar upperColor1 = new Scalar(175,0,0);

        Scalar lowerColor2 = new Scalar(50,50,50); //Yellow
        Scalar upperColor2 = new Scalar(175,175,175);

        while (!stopFlag) {
            if (camera.read(frame)) {
                Mat hsvImage = new Mat();
                Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);

                Mat mask1 = new Mat();
                Core.inRange(hsvImage, lowerColor, upperColor, mask1);

                Mat mask2 = new Mat();
                Core.inRange(hsvImage, lowerColor1, upperColor1, mask2);

                Mat mask3 = new Mat();
                Core.inRange(hsvImage, lowerColor2, upperColor2, mask3);

                if (Objects.equals(teamColor, "blue")) {
                    if (Core.countNonZero(mask1) > 1000 | Core.countNonZero(mask3) > 1000) {
                        stopFlag = true;
                        slide(0,1);
                        claw(2);
                        slide(0,-1);
                    }
                } else if (Objects.equals(teamColor, "red")) {
                    if (Core.countNonZero(mask2) > 1000 | Core.countNonZero(mask3) > 1000) {
                        stopFlag = true;
                        slide(0,1);
                        claw(2);
                        slide(0,-1);
                    }
                }
                hsvImage.release();
                mask1.release();
                mask2.release();
                mask3.release();

            }
        }
        camera.release();
    }

    public void threadRipper(double changeY, double directionY, double changeX, double directionX, double changeHeading, double directionHeading, double changePivot, double directionPivot, int changeClaw, double changeSlide, double directionSlide, double changeStrafe, double directionStrafe, String team) throws InterruptedException {

        this.CHANGE_Y = changeY;
        this.DIRECTION_Y = directionY;

        this.CHANGE_X = changeX;
        this.DIRECTION_X = directionX;

        this.CHANGE_HEADING = changeHeading;
        this.DIRECTION_HEADING = directionHeading;

        this.CHANGE_PIVOT = changePivot;
        this.DIRECTION_PIVOT = directionPivot;

        this.CLAW_OPEN = changeClaw;

        this.CHANGE_SLIDE = changeSlide;
        this.DIRECTION_SLIDE = directionSlide;

        this.CHANGE_STRAFE = changeStrafe;
        this.DIRECTION_STRAFE = directionStrafe;

        this.TEAM_COLOR = team;

        AutoThread main = new AutoThread(CHANGE_Y,DIRECTION_Y,CHANGE_X,DIRECTION_X,CHANGE_HEADING,DIRECTION_HEADING,CHANGE_PIVOT,DIRECTION_PIVOT,CLAW_OPEN,CHANGE_SLIDE,DIRECTION_SLIDE,CHANGE_STRAFE,DIRECTION_STRAFE,TEAM_COLOR);
        main.start();
        main.join();
    }

    public void telemetry() {
        telemetry.addData("Heading", getHeading.get(getHeading.size() - 1));
        telemetry.addData("Position", "X: %.2f, Y: %.2f", getX.get(getX.size() - 1), getY.get(getY.size() - 1));
        telemetry.addLine();
        telemetry.addData("Arm Distance", getArmDistance.get(getArmDistance.size() - 1));
        telemetry.addData("Arm Length", getArmLength.get(getArmLength.size() - 1));
        telemetry.update();
    }

}


