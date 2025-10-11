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
    private static final double ODOM_WHEEL_DIAMETER_IN = 1.0;
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
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double lfPower = drive + strafe + turn;
        double rfPower = drive - strafe - turn;
        double lrPower = drive - strafe + turn;
        double rrPower = drive + strafe - turn;

        leftFront.setPower(clip(lfPower));
        rightFront.setPower(clip(rfPower));
        leftRear.setPower(clip(lrPower));
        rightRear.setPower(clip(rrPower));

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
        double imuHeading = headingOffset + robotYaw;

        double dTheta = normalizeAngle(imuHeading - poseHeading);
        double dForward = (dLeftIn + dRightIn) / 2.0;
        double dLateral = dLatIn;

        double midHeading = poseHeading + dTheta / 2.0;
        double dxField = dForward * Math.cos(midHeading) - dLateral * Math.sin(midHeading);
        double dyField = dForward * Math.sin(midHeading) + dLateral * Math.cos(midHeading);

        poseX -= dxField;
        poseY -= dyField;
        poseHeading = imuHeading;

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