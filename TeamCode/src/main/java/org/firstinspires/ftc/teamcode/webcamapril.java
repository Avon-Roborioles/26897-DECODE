package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class webcamapril extends OpMode {
    MecanumDrive drive;
    private DcMotor leftFront, rightFront, leftRear, rightRear;
    AprilTagProcessor processor;
    VisionPortal vPortal;
    Servo servo;
    int width = 1920;
    IMU imu;

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

        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);
        servo = hardwareMap.get(Servo.class, "servo");
        processor = AprilTagProcessor.easyCreateWithDefaults();

        processor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .build();
        vPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                .addProcessor(processor)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry.addLine("Finished Init");
        telemetry.update();
        servo.setPosition(.6);
        Motor fl  = new Motor(hardwareMap,"frontLeft");
        Motor fr = new Motor(hardwareMap,"frontRight");
        Motor bl = new Motor(hardwareMap,"backLeft");
        Motor br = new Motor(hardwareMap,"backRight");
        fl.setInverted(true);
        br.setInverted(false);
        drive= new MecanumDrive(
                false,
                fl,
                fr,
                bl,
                br

        );
    }
    @Override
    public void loop() {
        // --- Driving (unchanged) ---
        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   =  gamepad1.right_stick_x;

//        double lf = drive + strafe + turn;
//        double rf = drive - strafe - turn;
//        double lr = drive - strafe + turn;
//        double rr = drive + strafe - turn;
//
//        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lr), Math.abs(rr)));
//        if (max > 1.0) {
//            lf /= max; rf /= max; lr /= max; rr /= max;
//        }
//
//        leftFront.setPower(-lf);
//        rightFront.setPower(-rf);
//        leftRear.setPower(-lr);
//        rightRear.setPower(-rr);
        this.drive.driveFieldCentric(strafe,drive,turn,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // --- Servo / AprilTag tracking ---
        ArrayList<AprilTagDetection> detections = processor.getDetections();
        boolean isDetected = false;
        double headingError = 0;
        boolean isTrue = false;
        boolean usingBearing = false;
        double yawVal = 1;
        double distance = -1;
        for (AprilTagDetection detection : detections) {
            if (detection != null && detection.id == 20) {
                headingError = (width / 2.0) - detection.center.x;
                telemetry.addData("X", detection.center.x);

                if (detection.ftcPose!=null){
                    isTrue=true;
                    headingError=detection.ftcPose.bearing;
                    usingBearing=true;
                    double yaw = detection.ftcPose.yaw;
                    if (yaw!=0) {
                        yawVal = 5 / yaw;
                    } else {
                        yawVal = 2;
                    }
                    if (Math.abs(yawVal)<1){
                        yawVal= Math.signum(yawVal)*1;
                    }
                    distance =Math.sqrt(
                            Math.pow(detection.rawPose.x,2)+
                                    Math.pow(detection.rawPose.y,2)+
                                    Math.pow(detection.rawPose.z,2));
                    telemetry.addData("Distance",distance
                            );

                }
                telemetry.addData("Heading Error",headingError);
                isDetected = true;
            }
        }

        // Servo smoothing variables
        double kP = 0.003;
        if (!usingBearing){
            kP=.00009;
        }
        double alpha=.3406967;             // smoothing factor
        double deadband = 5.0;

        // If driving, make servo react faster
//        boolean isDriving = Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05;
//        if (isDriving) {
//            alpha = 0.35;   // faster response while driving
//        } else {
//            alpha = 0.15;   // smoother/slower when still
//        }
        double newAlpha = (alpha*1.6769)*(yawVal*.75*Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))+1.);
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
        telemetry.addData("3D Data valid",isTrue);
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