package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;

public class ShooterSubsystem {
    private DcMotorEx shooterMotor;
    private Servo panServo;
    private Limelight3A limelight;

    private double servoPos = -0.8;
    private double velocityMultiplier = 1.1;

    // Physics Constants
    final double GRAVITY = 386.22;
    final double SHOOTER_ANGLE = 32;
    final double SHOOTER_HEIGHT = 16.8;
    final double GOAL_HEIGHT = 29.867;
    final double TICKS_PER_REV = 28.0;
    final double WHEEL_CIRCUMFERENCE = 3.779 * Math.PI;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        panServo = hardwareMap.get(Servo.class, "pan_servo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    // This method handles the TX-based servo tracking
    public void updateTracking(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Re-adding your specific telemetry from the original code
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                telemetry.addData("Fiducial", "ID: %d, X: %.2f", fr.getFiducialId(), fr.getTargetXDegrees());
            }

            // The Servo adjustment logic
            double cameraOffsetDegrees = 2.5;
            double correctedTx = result.getTx() - cameraOffsetDegrees;
            double tx_deadband = 0.0767;

            if (Math.abs(correctedTx) > tx_deadband) {
                servoPos -= 0.0001267 * correctedTx;
            }
        }

        // Clamp and set position
        servoPos = Math.max(0.05, Math.min(0.95, servoPos));
        panServo.setPosition(servoPos);
    }

    // This method handles the auto-velocity logic
    public void runAutoVelocity() {
        double currentDistance = getDistance();
        double targetVelocity;

        if (currentDistance > 0) {
            targetVelocity = calculateShooterVelocity(currentDistance);
        } else {
            targetVelocity = 0;
        }

        // Your "If > 500 use calc, else use 1500" logic
        if (targetVelocity > 500) {
            shooterMotor.setVelocity(targetVelocity);
        } else {
            shooterMotor.setVelocity(0);
        }
    }

    public double getDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        double targetOffsetAngle_Vertical = result.getTy();
        double limelightMountAngleDegrees = 16.7;
        double limelightLensHeightInches = 14.9;

        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);
        if (angleToGoalRadians == 0) return -1;

        return (GOAL_HEIGHT - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    private double calculateShooterVelocity(double distanceInches) {
        double y = GOAL_HEIGHT - SHOOTER_HEIGHT;
        double thetaRad = Math.toRadians(SHOOTER_ANGLE);
        double termA = (distanceInches * Math.tan(thetaRad)) - y;

        if (termA <= 0) return 0;

        double numerator = GRAVITY * Math.pow(distanceInches, 2);
        double denominator = 2 * Math.pow(Math.cos(thetaRad), 2) * termA;

        double targetLinearVelocity = Math.sqrt(numerator / denominator) * velocityMultiplier;
        return (targetLinearVelocity / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV;
    }

    public void adjustMultiplier(double delta) {
        velocityMultiplier += delta;
    }

    public double getVelocity() { return shooterMotor.getVelocity(); }
}