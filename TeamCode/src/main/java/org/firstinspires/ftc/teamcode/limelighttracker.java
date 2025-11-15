package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "LimelightTracker", group = "Limelight")
public class limelighttracker extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo panServo;
    private Limelight3A limelight;

    private final double SERVO_HOME_POS = 0.5;
    private final double SERVO_TUNE_KP = 0.012;
    private final double MAX_SERVO_INCREMENT = 0.015;
    private final double TX_DEADBAND = 1.5;

    // These must match your robot's physical setup
    private static final double TARGET_HEIGHT_IN = 14.0; // AprilTag center height in inches
    private static final double CAMERA_HEIGHT_IN = 6.0;  // Limelight lens height from floor in inches
    private static final double CAMERA_ANGLE_DEG = 0; // Limelight angle relative to horizontal




    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        panServo = hardwareMap.get(Servo.class, "pan_servo");
        panServo.setPosition(SERVO_HOME_POS);

        limelight = hardwareMap.get(Limelight3A.class, "limeLight");
        limelight.start();
        limelight.pipelineSwitch(0);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        double currentServoPos = SERVO_HOME_POS;

        while (opModeIsActive()) {

            // Get latest Limelight result
            LLResult result = limelight.getLatestResult();

            boolean tagDetected = (result != null && result.isValid());
            double tx = 0, ty = 0;
            double distanceInches = 0;

            if (tagDetected) {
                tx = result.getTx();
                ty = result.getTy();

                distanceInches = calculateDistance(ty);
            }


            // --- Mecanum drive control ---
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            frontLeft.setPower(drive + strafe + twist);
            frontRight.setPower(drive - strafe - twist);
            backLeft.setPower(drive - strafe + twist);
            backRight.setPower(drive + strafe - twist);

            // --- Servo tracking ---
            if (tagDetected) {
                double error = -tx;
                if (Math.abs(error) > TX_DEADBAND) {
                    double change = Range.clip(error * SERVO_TUNE_KP, -MAX_SERVO_INCREMENT, MAX_SERVO_INCREMENT);
                    currentServoPos = Range.clip(currentServoPos + change, 0.0, 1.0);
                    panServo.setPosition(currentServoPos);
                }

                telemetry.addData("Status", "Tracking Tag");
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("Distance (in)", "%.2f", distanceInches);

            } else {
                panServo.setPosition(SERVO_HOME_POS);
                telemetry.addData("Status", "No Tag Detected");
            }

            telemetry.update();
            idle();
        }


    }
    private double calculateDistance(double ty) {
        // Convert angle to radians
        double angleRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);

        // d = (h_target - h_camera) / tan(angle_to_target)
        return (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleRad);
    }


}
