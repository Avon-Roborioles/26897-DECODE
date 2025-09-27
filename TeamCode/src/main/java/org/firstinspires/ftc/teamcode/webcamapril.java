package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class webcamapril extends OpMode {
    private DcMotor leftFront, rightFront, leftRear, rightRear;
    AprilTagProcessor processor;
    VisionPortal vPortal;
    Servo servo;
    int width = 1920;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear = hardwareMap.get(DcMotor.class, "backRight");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);


        servo = hardwareMap.get(Servo.class, "servo");
        processor = AprilTagProcessor.easyCreateWithDefaults();
        vPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                .addProcessor(processor)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .setAutoStopLiveView(true)
                .build();

        telemetry.addLine("Finished Init");
        telemetry.update();
        servo.setPosition(.6);
    }
    @Override
    public void loop() {
        // --- Driving (unchanged) ---
        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   =  gamepad1.right_stick_x;

        double lf = drive + strafe + turn;
        double rf = drive - strafe - turn;
        double lr = drive - strafe + turn;
        double rr = drive + strafe - turn;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lr), Math.abs(rr)));
        if (max > 1.0) {
            lf /= max; rf /= max; lr /= max; rr /= max;
        }

        leftFront.setPower(-lf);
        rightFront.setPower(-rf);
        leftRear.setPower(-lr);
        rightRear.setPower(-rr);

        // --- Servo / AprilTag tracking ---
        ArrayList<AprilTagDetection> detections = processor.getDetections();
        boolean isDetected = false;
        double headingError = 0;

        for (AprilTagDetection detection : detections) {
            if (detection != null && detection.id == 20) {
                headingError = (width / 2.0) - detection.center.x;
                telemetry.addData("X", detection.center.x);
                telemetry.addData("Heading Error", headingError);
                isDetected = true;
            }
        }

        // Servo smoothing variables
        double kP = 0.00045;
        double alpha=.25;             // smoothing factor
        double deadband = 5.0;

        // If driving, make servo react faster
//        boolean isDriving = Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05;
//        if (isDriving) {
//            alpha = 0.35;   // faster response while driving
//        } else {
//            alpha = 0.15;   // smoother/slower when still
//        }
        double newAlpha = alpha*(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))+1.67);
        double currentPos = servo.getPosition();
        double targetPos = currentPos;

        if (isDetected) {
            if (Math.abs(headingError) > deadband) {
                targetPos = currentPos +(headingError * kP)*Math.abs(headingError*kP);
            }
        }

        // Clamp servo range [0, 1]
        targetPos = Math.max(0.0, Math.min(1.0, targetPos));

        // Smooth interpolation toward target
        double newPos = currentPos + newAlpha * (targetPos - currentPos);

        servo.setPosition(newPos);

        telemetry.addData("Servo Target", targetPos);
        telemetry.addData("Servo now", newPos);
        telemetry.addData("Servo Actual", servo.getPosition());

        if (!isDetected) {
            telemetry.addLine("No tag detected");
        }
        if (detections != null) {
            telemetry.addData("Num Detections", detections.size());
        }
        telemetry.update();
    }


//    @Override
//    public void loop() {
//        // --- Driving (unchanged) ---
//        double drive = -gamepad1.left_stick_y;
//        double strafe = -gamepad1.left_stick_x;
//        double turn = gamepad1.right_stick_x;
//
//        double lf = drive + strafe + turn;
//        double rf = drive - strafe - turn;
//        double lr = drive - strafe + turn;
//        double rr = drive + strafe - turn;
//
//        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lr), Math.abs(rr)));
//        if (max > 1.0) {
//            lf /= max;
//            rf /= max;
//            lr /= max;
//            rr /= max;
//        }
//
//        leftFront.setPower(-lf);
//        rightFront.setPower(-rf);
//        leftRear.setPower(-lr);
//        rightRear.setPower(-rr);
//
//        // --- Servo / AprilTag tracking ---
//        ArrayList<AprilTagDetection> detections = processor.getDetections();
//        boolean isDetected = false;
//        double headingError = 0;
//
//        for (AprilTagDetection detection : detections) {
//            if (detection != null && detection.id == 20) {
//                headingError = (width / 2.0) - detection.center.x;
//                telemetry.addData("X", detection.center.x);
//                telemetry.addData("Heading Error", headingError);
//                isDetected = true;
//            }
//        }
//
//        // Servo smoothing variables
//        double kP = 0.000015;
//        double alpha = 0.15;
//        double deadband = 5.0;
//
//        double currentPos = servo.getPosition();
//        double targetPos = currentPos;
//
//        if (isDetected) {
//            if (Math.abs(headingError) > deadband) {
//                targetPos = currentPos + (headingError * kP);
//            }
//        }
//
//        // Clamp servo range [0, 1]
//        targetPos = Math.max(0.0, Math.min(1.0, targetPos));
//
//        // Smooth interpolation toward target
//        double newPos = currentPos + alpha * (targetPos - currentPos);
//
//        servo.setPosition(newPos);
//
//        telemetry.addData("Servo Target", targetPos);
//        telemetry.addData("Servo now", newPos);
//        telemetry.addData("Servo Actual", servo.getPosition());
//
//        if (!isDetected) {
//            telemetry.addLine("No tag detected");
//        }
//        if (detections != null) {
//            telemetry.addData("Num Detections", detections.size());
//        }
//        telemetry.update();
//    }
}