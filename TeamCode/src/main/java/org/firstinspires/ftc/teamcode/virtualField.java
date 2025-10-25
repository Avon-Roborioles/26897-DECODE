package org.firstinspires.ftc.teamcode; // adjust as needed

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "virtual field testing", group = "drive")
public class virtualField extends OpMode {

    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private DcMotor leftOdom, rightOdom, lateralOdom;
    private IMU imu;

    private static final String LEFT_ODOM_NAME = "left_odom";
    private static final String RIGHT_ODOM_NAME = "right_odom";
    private static final String LATERAL_ODOM_NAME = "lateral_odom";
    private static final String IMU_NAME = "imu";

    private static final double TICKS_PER_REV = 32.5;
    private static final double ODOM_WHEEL_DIAMETER_IN = 2.0; // Corrected to 2.0 inches
    private static final double ODOM_TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * ODOM_WHEEL_DIAMETER_IN);
    private static final double TRACK_WIDTH_IN = 12.0;

    private static final double TARGET_RED_BASKET_X = 72.0;
    private static final double TARGET_RED_BASKET_Y = 72.0;

    // --- Odometry state ---
    private double poseX = 0.0;
    private double poseY = 0.0;
    private double poseHeading = 0.0; // radians

    private long prevLeftTicks = 0;
    private long prevRightTicks = 0;
    private long prevLatTicks = 0;

    private boolean poseLocked = false;
    private double headingOffset = 0.0;

    // Increased to 0.01 inches to filter out stronger mechanical noise/slip during rotation.
    private static final double MIN_TRANSLATION_THRESHOLD = 0.01; // inches

    @Override
    public void init() {
        // --- Drive motors ---
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear = hardwareMap.get(DcMotor.class, "backRight");

        leftOdom = hardwareMap.get(DcMotor.class, LEFT_ODOM_NAME);
        rightOdom = hardwareMap.get(DcMotor.class, RIGHT_ODOM_NAME);
        lateralOdom = hardwareMap.get(DcMotor.class, LATERAL_ODOM_NAME);

        imu = hardwareMap.get(IMU.class, IMU_NAME);

        // default orthogonal orientation (logo up, usb forward)
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lateralOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        prevLeftTicks = leftOdom.getCurrentPosition();
        prevRightTicks = rightOdom.getCurrentPosition();
        prevLatTicks = lateralOdom.getCurrentPosition();

        telemetry.addLine("DecodeField Odom Telemetry ready");
        telemetry.addLine("Press A/B/X/Y to set starting pose during INIT");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (!poseLocked) {
            if (gamepad1.a) { // Red Right Corner
                poseX = 72.0;
                poseY = -72.0;
                poseHeading = Math.toRadians(90);
                imu.resetYaw();
                headingOffset = poseHeading;
                poseLocked = true;
            }
            if (gamepad1.b) { // Red Left Corner
                poseX = -72.0;
                poseY = -72.0;
                poseHeading = Math.toRadians(90);
                imu.resetYaw();
                headingOffset = poseHeading;
                poseLocked = true;
            }
            if (gamepad1.x) { // Blue Left Corner
                poseX = -72.0;
                poseY = 72.0;
                poseHeading = Math.toRadians(-90);
                imu.resetYaw();
                headingOffset = poseHeading;
                poseLocked = true;
            }
            if (gamepad1.y) { // Blue Right Corner
                poseX = 72.0;
                poseY = 72.0;
                poseHeading = Math.toRadians(-90);
                imu.resetYaw();
                headingOffset = poseHeading;
                poseLocked = true;
            }
        }

        telemetry.addLine("Press A/B/X/Y to set starting pose");
        telemetry.addData("poseX", "%.2f", poseX);
        telemetry.addData("poseY", "%.2f", poseY);
        telemetry.addData("poseHeading deg", "%.1f", Math.toDegrees(poseHeading));
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Input Read ---
        // drive: Forward/Backward (Negated because Y is usually inverted on gamepads)
        double drive = -gamepad1.left_stick_y;
        // strafe: Left/Right strafing
        double strafe = gamepad1.left_stick_x;
        // turn: Rotation (Yaw)
        double turn = gamepad1.right_stick_x;

        // --- Mecanum Power Calculation (Corrected) ---
        // LF: +D +S +T
        double lfPower = drive + strafe + turn;
        // RF: +D -S -T
        double rfPower = drive - strafe - turn;
        // LR: +D -S +T  <-- This formula now goes to the leftRear motor
        double lrPower = drive - strafe + turn;
        // RR: +D +S -T  <-- This formula now goes to the rightRear motor
        double rrPower = drive + strafe - turn;

        // *** FIX: SWAP THE REAR POWER ASSIGNMENTS ***
        // Based on the standard Mecanum matrix, the power formulas were incorrectly
        // assigned to the leftRear and rightRear variables.
        leftFront.setPower(clip(lfPower));
        rightFront.setPower(clip(rfPower));
        rightRear.setPower(clip(rrPower)); // RR power formula assigned to rightRear
        leftRear.setPower(clip(lrPower));   // LR power formula assigned to leftRear

        long curLeftTicks = leftOdom.getCurrentPosition();
        long curRightTicks = rightOdom.getCurrentPosition();
        long curLatTicks = lateralOdom.getCurrentPosition();

        long dLeftTicks = curLeftTicks - prevLeftTicks;
        long dRightTicks = curRightTicks - prevRightTicks;
        long dLatTicks = curLatTicks - prevLatTicks;

        prevLeftTicks = curLeftTicks;
        prevRightTicks = curRightTicks;
        prevLatTicks = curLatTicks;

        double dLeftIn = ticksToInches(dLeftTicks);
        double dRightIn = ticksToInches(dRightTicks);
        double dLatIn = ticksToInches(dLatTicks);

        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double robotYaw = ypr.getYaw(AngleUnit.RADIANS);
        // This calculates the current heading (used only for telemetry/drive control)
        double imuHeading = headingOffset + robotYaw;

        // *** INVERTED ODOMETRY AND TRANSLATIONAL MOVEMENT ***
        // We invert the direction of the odometry readings to fix the "distance is backwards" problem.
        double dForward = -1.0 * (dLeftIn + dRightIn) / 2.0;
        double dLateral = -1.0 * dLatIn;

        // *** PURE TRANSLATIONAL ODOMETRY ***
        // Position update uses the constant starting heading (headingOffset) to completely ignore
        // the effects of current rotation (imuHeading) on the robot's calculated position.
        double referenceHeading = headingOffset;

        double dxField = dForward * Math.cos(referenceHeading) - dLateral * Math.sin(referenceHeading);
        double dyField = dForward * Math.sin(referenceHeading) + dLateral * Math.cos(referenceHeading);

        // *** NOISE GUARD ***
        // Calculate the magnitude of the translational movement
        double translationalMovementMagnitude = Math.hypot(dForward, dLateral);

        // If the movement is below the threshold (i.e., noise or pure rotation), zero out the displacement
        if (translationalMovementMagnitude < MIN_TRANSLATION_THRESHOLD) {
            dxField = 0.0;
            dyField = 0.0;
        }

        // Apply displacement (must be ADDITION)
        poseX += dxField;
        poseY += dyField;

        // The current heading is still tracked via IMU for display and control systems.
        poseHeading = imuHeading;

        // The distance calculation is now based on the corrected pose
        double distToRedBasket = Math.hypot(TARGET_RED_BASKET_X - poseX, TARGET_RED_BASKET_Y - poseY);

        telemetry.addData("Pose X (in)", "%.2f", poseX);
        telemetry.addData("Pose Y (in)", "%.2f", poseY);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(poseHeading));
        telemetry.addData("Dist -> Red Basket (in)", "%.2f", distToRedBasket);
        telemetry.addData("Raw enc (L,R,Lat) in", "%.2f, %.2f, %.2f", dLeftIn, dRightIn, dLateral);
        telemetry.update();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    private double ticksToInches(long ticks) {
        return ticks / ODOM_TICKS_PER_INCH;
    }

    private double clip(double v) {
        if (v > 1.0) return 1.0;
        if (v < -1.0) return -1.0;
        return v;
    }

    private double normalizeAngle(double ang) {
        while (ang >= Math.PI) ang -= 2.0 * Math.PI;
        while (ang < -Math.PI) ang += 2.0 * Math.PI;
        return ang;
    }
}
