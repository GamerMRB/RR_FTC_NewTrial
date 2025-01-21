package org.firstinspires.ftc.teamcode.dynamicsLibrary;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Instruction;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;


public abstract class UscOpMode extends LinearOpMode {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx armPivot;
    public DcMotorEx armSlide;
    public Servo leftClaw;
    public Servo rightClaw;

    public static Camera[] cameras = {
    };
    public WebcamName camera1;
    public WebcamName camera2;
    public WebcamName camera3;
    public VisionPortal visionPortal;
    public VisionPortal visionPortal2;
    public VisionPortal visionPortal3;

    public Position robotPos = Position.xyr(0, 0, 0);
    public Vec3 clawPos;
    public double armAngle;
    public double armLength;
    public PID2 movementPID = new PID2(1, 0, 0);
    public PID rotationPID = new PID(1, 0, 0);

    public PID armPID = new PID(1, 0, 0);
    public double armThen = 0;
    public boolean armDisabled = true;

    public final double WHEEL_DIAMETER = 96.0;
    public final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public final double TICKS_PER_REVOLUTION = 538;
    public final double SPEED_MAX = 1.0;
    public final double STRAFE_SPEED = 0.75;
    public final Vec3 pivotPos = Vec3.xyz(0, 0, 13.25);
    public final double INITIAL_ARM_ANGLE = - Math.PI/4;
    public final double MIN_ARM_LENGTH = 10.375;
    public final double LEFT_CLOSE = 0.75;
    public final double RIGHT_CLOSE = 0.3 ;
    public final double LEFT_OPEN = 0.45;
    public final double RIGHT_OPEN = 0.60;
    public final double MAX_VELOCITY = 1500;



    public void setUpHardware(){
        setUpDrivetrain();
        setUpCameras();
        setUpArm();

    }

    public void setUpCameras(){
        cameras = new Camera[]{
//                Camera.makeIt(Position.xyr(0, 3, 0), hardwareMap.get(WebcamName.class, "Webcam 1"), 0),
//                Camera.makeIt(Position.xyr(0, -3, Math.PI), hardwareMap.get(WebcamName.class, "Webcam 2"), 0),
        };
    }
    public ArrayList<Orientation> updatePos(){
        Vec2 pos = Vec2.zero;
        Vec2 dir = Vec2.zero;
        long tagCount = 0;
        ArrayList<Orientation> orientations = new ArrayList<>();
        for(Camera camera : cameras){
            ArrayList<AprilTagDetection> detections = camera.processor.getDetections();
            for(AprilTagDetection detection : detections){
                Position tag = Position.xyr(detection.metadata.fieldPosition.get(0), detection.metadata.fieldPosition.get(1), 0);
                orientations.add(detection.metadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS));
                Position cam = tag.remove(Position.xyr(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw));
                Position robot = cam.remove(camera.pos);
                pos = pos.add(robot.disp);
                dir = dir.add(robot.rot);
            }
            tagCount += detections.size();
        }
        if(tagCount > 0) {
            robotPos = Position.vr(pos.div(tagCount), dir.angle());
        }
        return orientations;
    }

    protected void setUpDirections(){
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void setUpDrivetrain() {
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft"); // Motor 0
        backRight = hardwareMap.get(DcMotorEx.class, "backRight"); // Motor 2
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft"); // Motor 1
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight"); // Motor 4
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        setUpDirections();
    }
    public void setUpArm(){
        armAngle = INITIAL_ARM_ANGLE;
        armLength = MIN_ARM_LENGTH;
        armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
    }
    public void moveArmToward(double position){
        double now = getRuntime();
        double dt = now - armThen;
        if(armDisabled){
            armDisabled = false;
            armPID.reset();
        }
        armPID.update(position - armPivot.getCurrentPosition(), dt);
        armPivot.setPower(armPID.getPow());
        armThen = now;
    }
    public void disableArm(){
        armDisabled = true;
        armPivot.setPower(0);
    }
    public void runOpMode() throws InterruptedException {
    }

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    protected void resetMotors() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    protected void setRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void setPower(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }
    protected void setVelocity(double velocity) {
        frontLeft.setVelocity(velocity);
        backLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        backRight.setVelocity(velocity);
    }

    protected void setTargetPosition(int numberOfTicks) {
        frontLeft.setTargetPosition(numberOfTicks);
        frontRight.setTargetPosition(numberOfTicks);
        backLeft.setTargetPosition(numberOfTicks);
        backRight.setTargetPosition(numberOfTicks);
    }


    public void pow(double forward, double side, double rot){
        double maxPow = Math.max(1, Math.abs(forward) + Math.abs(side) + Math.abs(rot));
        frontLeft.setPower((forward - side - rot) / maxPow);
        frontRight.setPower((forward + side + rot) / maxPow);
        backLeft.setPower((forward + side - rot) / maxPow);
        backRight.setPower((forward - side + rot) / maxPow);
    }
    protected void pow(Position position){
        pow(position.disp.y, position.disp.x, position.rot);
    }
    protected void pow(Vec2 disp, double rot){
        pow(disp.y, disp.x, rot);
    }

    protected void movementPowerDisable() {
        frontLeft.setMotorDisable();
        frontRight.setMotorDisable();
        backLeft.setMotorDisable();
        backRight.setMotorDisable();
    }

    protected void movementPowerEnable() {
        frontLeft.setMotorEnable();
        frontRight.setMotorEnable();
        backLeft.setMotorEnable();
        backRight.setMotorEnable();
    }
    protected double mod(double a, double b){
        return a - b*Math.floor(a / b);
    }
    protected double simplifyAngle(double angle){
        return mod(angle + Math.PI, 2*Math.PI) - Math.PI;
    }
    protected void moveTo(Position position){
        Position diff = robotPos.difference(position);
        double posAllowance = 1.5;
        double angAllowance = Math.PI/8;
        double then = (double) System.currentTimeMillis() / 1000;
        while(
                diff.disp.mag() >= posAllowance ||
                Math.abs(simplifyAngle(diff.rot)) >= angAllowance
        ){
            double now = getRuntime();
            double dt = now - then;
            movementPID.update(diff.disp, dt);
            rotationPID.update(simplifyAngle(diff.rot), dt);
            pow(movementPID.getPow(), rotationPID.getPow());
            updatePos();
            diff = robotPos.difference(position);
            then = now;
        }
    }


    protected void calculateClaw(){
        clawPos = pivotPos.add(Vec3.v2z(robotPos.dir().mult(armLength * Math.cos(armAngle)) , armLength * Math.sin(armAngle)));
    }
    protected void moveClawTo(Vec3 target){
    // needs movement code first
    }
    protected void setClaw(boolean closed){
        if (closed) {
            leftClaw.setPosition(LEFT_CLOSE);
            rightClaw.setPosition(RIGHT_CLOSE);
        } else {
            leftClaw.setPosition(LEFT_OPEN);
            rightClaw.setPosition(RIGHT_OPEN);
        }
    }


    // Visualization of scaling functions: https://www.desmos.com/calculator/attwysf9bd

    protected double scaleMovement(double vIn){
        return Math.pow(Math.sin((Math.PI * vIn) / 2), 3);
    }

    public void executeInstructions(Instruction[] instructions){
        for(Instruction instruction : instructions){
            instruction.start(this);
            boolean done = false;
            while(!done && opModeIsActive()){
                done = instruction.update(this);
            }
            instruction.end(this);
        }
    }
}