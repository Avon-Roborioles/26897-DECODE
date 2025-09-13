package org.firstinspires.ftc.teamcode.Limelight_Things;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;

@TeleOp(name = "Limelight_AprilTag_Reader")
public class LimelightAprilTagReader extends LinearOpMode {
    @Override
    public void runOpMode() {

        // Step 1: Initialize the AprilTag Processor
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        // Step 2: Initialize the Webcam using a VisionPortal.Builder
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.addProcessor(aprilTagProcessor);

        VisionPortal visionPortal = builder.build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Get a list of detected tags
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            // Output data for each detected tag
            for (AprilTagDetection detection : detections) {
                telemetry.addData("ID", detection.id);
                telemetry.addData("X (Inches)", detection.ftcPose.x);
                telemetry.addData("Y (Inches)", detection.ftcPose.y);
                telemetry.addData("Z (Inches)", detection.ftcPose.z);
            }

            // Update telemetry
            telemetry.update();
        }
    }
}