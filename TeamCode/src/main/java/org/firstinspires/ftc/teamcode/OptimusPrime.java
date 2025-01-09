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

@SuppressWarnings("unused")
@Autonomous
public class OptimusPrime extends UscOpMode {
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

    double CHANGE = 0;
    double DIRECTION = 0;
    int CLAW_OPEN = 0;

    ArrayList<Double> getHeading, getX, getY, getArmLength, getArmDistance = new ArrayList<>();

    boolean stopFlag = false;

    @Override
    public void runOpMode() {
        setUpHardware();

        //Encoder setup
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while(opModeIsActive()) {

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
            measurement[0] = (frontRightDistance) / 2 * Math.cos(Math.toRadians(imuHeading)); // X Position
            measurement[1] = (frontRightDistance) / 2 * Math.sin(Math.toRadians(imuHeading)); // Y Position
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



            ////Code (Sample, alterations TBD 1/6/25)

            //First put the specimen on highest bar
            arcee(0,1);
            bumblebee(0,1);
            arcee(0,-1);
            ironhide(1);
            bumblebee(0,-1);
            //Next, go over to team member
            wheelie(0,1);
            ratchet(180,1);
            //Pick up specimen and score on top bar
            bulkhead(0,1);
            arcee(0,1);
            ironhide(2);
            arcee(0,-1);
            ratchet(180,1);
            wheelie(0,-1);
            bumblebee(0,1);
            arcee(0,-1);
            ironhide(1);
            bumblebee(0,-1);
            bulkhead(0,-1);
            //Next, go pick up neutral sample and give to team member
            wheelie(0,-1);
            arcee(0,-1);
            bulkhead(0,1);
            ironhide(2);
            arcee(0,-1);
            ratchet(180,1);
            bumblebee(0,1);
            arcee(0,1);
            ironhide(1);
            wheelie(0,1);
            //Pick up specimen and score on top bar
            wheelie(0,-1);
            bumblebee(0,1);
            ironhide(2);
            arcee(0,1);
            ratchet(180,1);
            wheelie(0,-1);
            bumblebee(0,1);
            arcee(0,-1);
            ironhide(1);
            bumblebee(0,-1);
            bulkhead(0,-1);
            //Next, go pick up 2nd neutral sample and give to team member
            wheelie(0,-1);
            arcee(0,-1);
            bulkhead(0,1);
            ironhide(2);
            arcee(0,-1);
            ratchet(180,1);
            bumblebee(0,1);
            arcee(0,1);
            ironhide(1);
            wheelie(0,1);
            //Pick up specimen and score on top bar
            wheelie(0,-1);
            bumblebee(0,1);
            ironhide(2);
            arcee(0,1);
            ratchet(180,1);
            wheelie(0,-1);
            bumblebee(0,1);
            arcee(0,-1);
            ironhide(1);
            bumblebee(0,-1);
            bulkhead(0,-1);
            //Next, go pick up 3rd neutral sample and give to team member
            wheelie(0,-1);
            arcee(0,-1);
            bulkhead(0,1);
            ironhide(2);
            arcee(0,-1);
            ratchet(180,1);
            bumblebee(0,1);
            arcee(0,1);
            ironhide(1);
            wheelie(0,1);
            //Pick up specimen and score on top bar
            wheelie(0,-1);
            bumblebee(0,1);
            ironhide(2);
            arcee(0,1);
            ratchet(180,1);
            wheelie(0,-1);
            bumblebee(0,1);
            arcee(0,-1);
            ironhide(1);
            bumblebee(0,-1);
            bulkhead(0,-1);
            //Setup for color recognition
            arcee(0,-1);
            bumblebee(0,1);
            //Go use color recognition for next sample
            jazz("blue");
            //Pick up specimen and score on top bar
            bumblebee(0,-1);
            wheelie(0,1);
            ratchet(180,1);
            arcee(0,1);
            bumblebee(0,1);
            ironhide(1);
            wheelie(0,1);
            //wait...find variable for waiting
            wheelie(0,-1);
            bumblebee(0,1);
            ironhide(2);
            arcee(0,1);
            ratchet(180,1);
            wheelie(0,-1);
            bumblebee(0,1);
            arcee(0,-1);
            ironhide(1);
            bumblebee(0,-1);
            bulkhead(0,-1);
            //Setup for color recognition
            arcee(0,-1);
            bumblebee(0,1);
            //Go use color recognition for next sample
            jazz("blue");
            //Pick up specimen and score on top bar
            bumblebee(0,-1);
            wheelie(0,1);
            ratchet(180,1);
            arcee(0,1);
            bumblebee(0,1);
            ironhide(1);
            wheelie(0,1);
            //wait...find variable for waiting
            wheelie(0,-1);
            bumblebee(0,1);
            ironhide(2);
            arcee(0,1);
            ratchet(180,1);
            wheelie(0,-1);
            bumblebee(0,1);
            arcee(0,-1);
            ironhide(1);
            bumblebee(0,-1);
            bulkhead(0,-1);
            //Park
            wheelie(0,-1);

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
    public void bumblebee(double change, int direction) {
        this.CHANGE = change * 25.4;
        this.DIRECTION = direction;

        frontLeft.setPower(DIRECTION);
        frontRight.setPower(DIRECTION);
        backLeft.setPower(DIRECTION);
        backRight.setPower(DIRECTION);
        if (Math.abs(avgY - (CHANGE + getY.get(getY.size() - 1))) <= TOLERANCE) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    //x controller
    public void wheeljack(double change, int direction) {
        this.CHANGE = change * 25.4;
        this.DIRECTION = direction;

        frontLeft.setPower(DIRECTION);
        frontRight.setPower(DIRECTION);
        backLeft.setPower(DIRECTION);
        backRight.setPower(DIRECTION);
        if (Math.abs(avgX - (CHANGE + getX.get(getX.size() - 1))) <= TOLERANCE) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    //heading, -Direction goes to the left, while +Direction goes to the right
    public void ratchet(double change, int direction) {
        this.CHANGE = change;
        this.DIRECTION = direction;

        frontLeft.setPower(-DIRECTION);
        frontRight.setPower(DIRECTION);
        backLeft.setPower(-DIRECTION);
        backRight.setPower(DIRECTION);
        if (Math.abs(avgHeading - (CHANGE + getHeading.get(getHeading.size() - 1))) <= 4) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        if (getX.get(getX.size() - 1) != 0) {
            avgX -= getX.get(getX.size() - 1);
        }

        if (getY.get(getY.size() - 1) != 0) {
            avgY -= getY.get(getY.size() - 1);
        }

    }

    //arm pivot controller; change = sin(theta) * armLength
    public void arcee(double change, int direction) {
        this.CHANGE = change * 25.4;
        this.DIRECTION = direction;

        armPivot.setPower(DIRECTION * 0.5);
        if (Math.abs(avgArmDistance - (CHANGE + getArmDistance.get(getArmDistance.size() - 1))) <= TOLERANCE) {
            armPivot.setPower(0);
        }
    }

    //claw controller
    public void ironhide(int claw) {
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
    public void bulkhead(double change, int direction) {
        this.CHANGE = change * 25.4;
        this.DIRECTION = direction;

        armSlide.setPower(DIRECTION);
        if (Math.abs(avgArmLength - (CHANGE + getArmLength.get(getArmLength.size() - 1))) <= TOLERANCE) {
            armSlide.setPower(0);
        }
    }

    //strafe controller
    public void wheelie(double change, double direction) {
        this.CHANGE = change * 25.4;
        this.DIRECTION = direction;

        frontLeft.setPower(DIRECTION * 0.75);
        frontRight.setPower(DIRECTION * 0.75);
        backLeft.setPower(-DIRECTION * 0.75);
        backRight.setPower(-DIRECTION * 0.75);
        while (!stopFlag) {
            if (Math.abs(avgX - (CHANGE + getX.get(getX.size() - 1))) <= TOLERANCE) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
        }
    }

    //Color recognition using camera
    public void jazz(String teamColor) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        VideoCapture camera = new VideoCapture();

        Mat frame = new Mat();
        Scalar lowerColor = new Scalar(0,0,50); //Blue
        Scalar upperColor = new Scalar(0,0,175);

        Scalar lowerColor1 = new Scalar(50,0,0); //Red
        Scalar upperColor1 = new Scalar(175,0,0);

        Scalar lowerColor2 = new Scalar(50,50,50); //Yellow
        Scalar upperColor2 = new Scalar(175,175,175);

        Thread wheelieThread = new Thread(() -> wheelie(24, -1)); // Start the robot's movement in a separate thread
        wheelieThread.start();

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
                        bulkhead(0,1);
                        ironhide(2);
                        bulkhead(0,-1);
                    }
                } else if (Objects.equals(teamColor, "red")) {
                    if (Core.countNonZero(mask2) > 1000 | Core.countNonZero(mask3) > 1000) {
                        stopFlag = true;
                        bulkhead(0,1);
                        ironhide(2);
                        bulkhead(0,-1);
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

    public void telemetry() {
        telemetry.addData("Heading", avgHeading);
        telemetry.addData("Position", "X: %.2f, Y: %.2f", avgX, avgY);
        telemetry.addLine();
        telemetry.addData("Arm Distance", avgArmDistance);
        telemetry.addData("Arm Length", avgArmLength);
        telemetry.update();
    }

}

