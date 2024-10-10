package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


public class Camera {
    public final Position pos;
    public final WebcamName webcam;
    public final VisionPortal portal;
    public final AprilTagProcessor processor;
    private Camera (Position posIn, WebcamName webcamIn, AprilTagProcessor processorIn, VisionPortal portalIn){
        pos = posIn;
        webcam = webcamIn;
        processor = processorIn;
        portal = portalIn;
    }
    public static Camera makeIt(Position posIn, WebcamName webcamName){
        VisionPortal.Builder builder = new VisionPortal.Builder();

        AprilTagProcessor processor = AprilTagProcessor.easyCreateWithDefaults();
        builder.addProcessor(processor);

        return new Camera(posIn, webcamName, processor, builder.build());
    }
}

