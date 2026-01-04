package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;

import java.util.List;

@TeleOp(name = "Test Teleop")
public class TestTeleOp extends LinearOpMode {
    // Shoot Kinematics
    final double GRAVITY = 386.22; // inches per second squared
    final double SHOOTER_ANGLE = 32; // Angle of your shooter in degrees
    final double SHOOTER_HEIGHT = 16.8; // Height of shooter exit point from floor in inches
    final double GOAL_HEIGHT = 29.867; // Height of the target opening in inches
    final double TICKS_PER_REV = 28.0;
    final double WHEEL_DIAMETER = 3.779; // inches (96mm wheel)
    final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private double VELOCITY_MULTIPLIER = 1.1;

    private MecanumDrivetrain driveTrain;
    private DcMotor intake;
    private CRServo artifactServo;
    private ArtifactSensor artifactSensor;
    private DcMotor kicker;
    private DcMotorEx motor;
    private Limelight3A limelight;
    private Servo servo;
    private double servoPos = 0.3;
    public static int tag;

    private limelightcommand limelightCommand;
    private limelightsubsystem limelightSubsystem;
    private LLResult result;

    private double targetVelocity = 100;
    private double servoPosition = 0.1; // Current position of the servo
    private ArtifactColor targetColor = ArtifactColor.NOTHING;

    private ElapsedTime timer = new ElapsedTime();
    private boolean lastA = false;
    private boolean lastB = false;
    private double sensorCheckTime = -1;
    private double moveFinishTime = 0;


    private DcMotorEx artifactEncoder; // We use the motor class to read encoder pins
    private int targetTicks = 0;
    private final double MAX_SORT_POWER = 0.2;
    private final int TICKS_PER_SLOT = 2710;    // 8192 / 3 slots
    private final int TICK_TOLERANCE = 1670;

    @Override
    public void runOpMode() {
//        intake = hardwareMap.get(DcMotor.class, "intake");
        driveTrain = new MecanumDrivetrain(hardwareMap);

        artifactServo = hardwareMap.get(CRServo.class, "sorterservo");
        artifactEncoder = hardwareMap.get(DcMotorEx.class, "backLeft"); // Use same name as the encoder port
        artifactEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        artifactEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        artifactSensor = new ArtifactSensor(hardwareMap);

        // Kicking
        kicker = hardwareMap.get(DcMotor.class, "kicker");

        // Shooting
        motor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Limelight Tracking
        servo = hardwareMap.get(Servo.class, "pan_servo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        driveTrain = new MecanumDrivetrain(hardwareMap);


        limelightCommand = new limelightcommand(limelightSubsystem, result);

        telemetry.setMsTransmissionInterval(11);
        servo.setPosition(servoPos);
        limelight.pipelineSwitch(0);
        limelight.start();


//

        telemetry.addLine("A: Seek Green | B: Seek Purple | X: Reset");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Limelight Tracking
            LLStatus status = limelight.getStatus();

            // ---- Limelight logic (servo adjustment) ----
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tags", result.getFiducialResults());
                telemetry.addData("distance", getDistance());
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
//                for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                    if (fr.getFiducialId() == 21) {
//                        tag = 21;
//                        limelight.pipelineSwitch(3);
//                    } else if (fr.getFiducialId() == 22) {
//                        tag = 22;
//                        limelight.pipelineSwitch(3);
//                    } else if (fr.getFiducialId() == 23) {
//                        tag = 23;
//                        limelight.pipelineSwitch(3);
//                    }
//                }
//                telemetry.addData("tag", tag);
                telemetry.update();


                double tx_deadband = 0.076767676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767;

                double cameraOffsetDegrees = 2.5;
                double correctedTx = result.getTx() - cameraOffsetDegrees;

                if (Math.abs(correctedTx) > tx_deadband) {
                    servoPos -= 0.0001267 * correctedTx;
                }
            }

            if (servoPos > 0.95) {
                servoPos = 0.95;
            } else if (servoPos < 0.05) {
                servoPos = 0.05;
            }
            servo.setPosition(servoPos);


            double currentDistance = getDistance();

// AUTO VELOCITY CALCULATION
            if (currentDistance > 0) {
                // Calculate ideal velocity based on distance
                targetVelocity = calculateShooterVelocity(currentDistance);
            } else {
                // Optional: Set a default velocity if target is lost
                targetVelocity = 0;
            }

// Manual Override
            if (gamepad1.right_bumper) VELOCITY_MULTIPLIER += 0.01;
            if (gamepad1.left_bumper) VELOCITY_MULTIPLIER -= 0.01;

// Apply velocity
            if (targetVelocity > 500) {
                motor.setVelocity(targetVelocity);
            } else {
                motor.setVelocity(1500);
            }


            // 1. Drive
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            driveTrain.drive(strafe, forward, turn);


            // 2. SORTING LOGIC
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;

            if (currentA && !lastA) {
                targetColor = ArtifactColor.GREEN;
                targetTicks = artifactEncoder.getCurrentPosition();
                sensorCheckTime = timer.milliseconds() + 500;
            } else if (currentB && !lastB) {
                targetColor = ArtifactColor.PURPLE;
                targetTicks = artifactEncoder.getCurrentPosition();
                sensorCheckTime = timer.milliseconds() + 500;
            }
            lastA = currentA;
            lastB = currentB;

            if (targetColor != ArtifactColor.NOTHING) {
                int currentTicks = artifactEncoder.getCurrentPosition();
                int error = targetTicks - currentTicks;
                int absError = Math.abs(error);

                // STEP 1: Fast movement if we are far away
                if (absError > TICK_TOLERANCE) { // 1670+ ticks away
                    double power = (error > 0) ? MAX_SORT_POWER : -MAX_SORT_POWER;
                    artifactServo.setPower(power);
                    sensorCheckTime = timer.milliseconds() + 400;
                }
                // STEP 2: The slow zone to get closer
                else if (absError <= TICK_TOLERANCE && absError > 50) {
                    // Drop power to 0.02 to slowly approach the final position
                    double slowPower = (error > 0) ? 0.02 : -0.02;
                    artifactServo.setPower(slowPower);
                    sensorCheckTime = timer.milliseconds() + 400;
                }
                // STEP 3: Final Stop (Within 50 ticks)
                else {
                    artifactServo.setPower(0);
                    if (timer.milliseconds() >= sensorCheckTime) {
                        if (artifactSensor.read() == targetColor) {
                            kicker.setPower(1);
                            sleep(500);
                            kicker.setPower(-1);
                            sleep(200);
                            kicker.setPower(0);
                            targetColor = ArtifactColor.NOTHING;
                            intake.setPower(0);
                        } else {
                            // Wrong color, move to next slot
                            targetTicks += TICKS_PER_SLOT;
                            sensorCheckTime = timer.milliseconds() + 600;
                        }
                    }
                }
            } else {
                artifactServo.setPower(0);
            }


            // 3. Kicker and Shooter
            if (gamepad1.right_trigger > 0.2) kicker.setPower(1);
            else kicker.setPower(0);


            // 4. Intake

            intake.setPower(-gamepad1.left_trigger);

            if (gamepad1.left_trigger > 0.01 && gamepad1.x) {
                intake.setPower(gamepad1.left_trigger);
            }

            // Telemetry
            telemetry.addData("Targeting", targetColor);
            telemetry.addData("Encoder Pos", artifactEncoder.getCurrentPosition());
            telemetry.addData("Target Ticks", targetTicks);
            //telemetry.addData("Sensor Sees", currentColor);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", motor.getVelocity());
            telemetry.update();
        }

        // Get Latest results from the limelight
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
    }

    double getDistance() {
        LLResult result = limelight.getLatestResult();

        // SAFETY CHECK: If no target is seen, return -1 or keep previous distance
        if (result == null || !result.isValid()) {
            return -1;
        }

        double targetOffsetAngle_Vertical = result.getTy();
        double limelightMountAngleDegrees = 16.7;
        double limelightLensHeightInches = 14.9;
        double goalHeightInches = 29.867; // High Basket height?

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        // Prevent division by zero if angle is exactly 0
        if (angleToGoalRadians == 0) return -1;

        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    public double calculateShooterVelocity(double distanceInches) {
        if (distanceInches <= 0) return 0; // Invalid distance

        // 1. Calculate relative height (y)
        double y = GOAL_HEIGHT - SHOOTER_HEIGHT;

        // 2. Convert angle to radians
        double thetaRad = Math.toRadians(SHOOTER_ANGLE);

        // 3. Calculate Term A: (x * tan(theta) - y)
        double termA = (distanceInches * Math.tan(thetaRad)) - y;

        // Safety: If termA is negative or zero, the target is physically unreachable
        // at this angle (projectile would need to go through the floor or backwards).
        if (termA <= 0) return 0;

        // 4. Calculate Squared Velocity using the projectile formula
        // v^2 = (g * x^2) / (2 * cos^2(theta) * termA)
        double numerator = GRAVITY * Math.pow(distanceInches, 2);
        double denominator = 2 * Math.pow(Math.cos(thetaRad), 2) * termA;

        double velocitySquared = numerator / denominator;
        double targetLinearVelocity = Math.sqrt(velocitySquared); // Inches per second

        // 5. Apply "Fudge Factor" for air resistance/friction
        targetLinearVelocity *= VELOCITY_MULTIPLIER;

        // 6. Convert Linear Velocity (in/s) to Motor Ticks per Second
        // ticksPerSec = (velocity / circumference) * ticksPerRev
        double targetTicksPerSec = (targetLinearVelocity / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV;

        return targetTicksPerSec;
    }

    double getYawAprilTag() {
        LLResult result = limelight.getLatestResult();
        return result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
    }
}