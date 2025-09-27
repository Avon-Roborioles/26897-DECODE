package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Tracking Drive", group = "Concept")
public class TeleopButWebcamBcAidenAndRithikTookEverything extends OpMode {

    // Drive motors
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    // Servo that tracks the AprilTag
    private Servo trackerServo;

    // Vision objects
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Servo gain for adjustments
    private static final double kServoGain = 0.005;

    // Servo's "center" = starting position
    private double servoCenter;

    @Override
    public void init() {
        // Motors
        leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear   = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear  = hardwareMap.get(DcMotor.class, "backRight");

        // Servo
        trackerServo = hardwareMap.get(Servo.class, "trackerServo");
        servoCenter = trackerServo.getPosition(); // Read current position as center

        // AprilTag processor + VisionPortal
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1920,1080))
                .build();
    }

    @Override
    public void loop() {
        // --- DRIVE CONTROL ---
        double drive  = -gamepad1.left_stick_y;  // forward/back
        double strafe =  gamepad1.left_stick_x;  // left/right
        double turn   =  gamepad1.right_stick_x; // rotation

        double lf = drive + strafe + turn;
        double rf = drive - strafe - turn;
        double lr = drive - strafe + turn;
        double rr = drive + strafe - turn;

        // Normalize powers if >1
        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lr), Math.abs(rr)));
        if (max > 1.0) {
            lf /= max; rf /= max; lr /= max; rr /= max;
        }

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);

        // --- APRILTAG TRACKING ---
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);

            // Camera image center (adjust if resolution != 640)
            double imageCenterX = 320.0;

            // Proportional error: tag center - image center
            double errorX = tag.center.x - imageCenterX;
            errorX = tag.ftcPose.bearing;
            // Servo adjustment (tune kServoGain)
            double servoAdjustment = errorX * kServoGain;
            servoAdjustment=servoAdjustment*Math.abs(servoAdjustment);

            // Update servo position relative to starting center
            double newPos = servoCenter + servoAdjustment;

            // Clamp
            newPos = Math.max(0.0, Math.min(1.0, newPos));

            trackerServo.setPosition(newPos);

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Servo Pos", newPos);
            telemetry.addData("ErrorX", errorX);
        } else {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.update();

    }
}

        // --- APRILTAG TRACKING ---
//        List<AprilTagDetection> detections = aprilTag.getDetections();
//
//        // A variable to store your proportional gain. Start with a small value and adjust.
//// This value is crucial for tuning the servo's responsiveness.
//        double PROPORTIONAL_GAIN = 0.0005;
//
//        if (!detections.isEmpty()) {
//            AprilTagDetection tag = detections.get(0);
//            double imageCenterX = 120.01;
//            double errorX = tag.center.x - imageCenterX;
//
//            // Apply proportional control by multiplying the error by the gain
//            double servoAdjustment = errorX * PROPORTIONAL_GAIN;
//
//            // Adjust the servo position based on the calculated adjustment.
//            // The servoCenter variable should be your initial, known center position.
//            double newPos = servoCenter - servoAdjustment;
//
//            double bearing = tag.ftcPose.bearing;
//
//            while(bearing != 0) {
//                if(bearing < 0){
//                    newPos = servoCenter + servoAdjustment;
//                } else {
//                    newPos = servoCenter - servoAdjustment;
//                }
//            }
//
//            // Clamp between 0.0-1.0 to prevent the servo from receiving invalid commands
//            newPos = Math.max(0.0, Math.min(1.0, newPos));
//
//            trackerServo.setPosition(newPos);
//
//            telemetry.addData("Tag ID", tag.id);
//            telemetry.addData("Servo Pos", newPos);
//            telemetry.addData("ErrorX", errorX);
//            telemetry.addData("Bearing", tag.ftcPose.bearing);
//            telemetry.addData("Range", tag.ftcPose.range);
//        } else {
//            telemetry.addLine("No AprilTag detected");
//            // You might want to consider what the servo should do when no tag is found.
//            // For example, it could return to its center position or hold its last known position.
//        }
//
//        telemetry.update();
//    }
//}