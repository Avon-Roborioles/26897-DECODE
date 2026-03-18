package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;
import dev.nextftc.core.commands.Command;

public class RedShooterSubsystem {
    private DcMotorEx shooterMotor;
    private Servo panServo;
    private Limelight3A limelight;
    private Servo indicatorLight;

    private Telemetry telemetry;

    private double servoPos = 0.5;

    // --- NEW MANUAL VELOCITY VARIABLES ---
    private double targetVelocity = 0; // Starts at 100 as requested
    private double velocityIncrement = 25; // Amount to increase by when button is pressed

    // Constants needed for Distance Calculation only
    final double GOAL_HEIGHT = 29.867;



    public RedShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setVelocityPIDFCoefficients(610,0,11.2,15);
        panServo = hardwareMap.get(Servo.class, "pan_servo");
        panServo.setPosition(0.5);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        indicatorLight = hardwareMap.get(Servo.class, "teamlight");
    }

    public Command updateCommand() {
        return new Command() {
            @Override
            public void update() {
                // This runs every loop cycle
                updateTracking(telemetry);
                runAutoVelocity(telemetry);
            }

            @Override
            public boolean isDone() {
                return false;
            }
        };
    }



    // This method handles the TX-based servo tracking
    public void updateTracking(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                //telemetry.addData("Fiducial", "ID: %d, X: %.2f", fr.getFiducialId(), fr.getTargetXDegrees());
//                if(fr.getFiducialId() == 24) {
//                    //Red
//                    indicatorLight.setPosition(0.277);
//                } else if(fr.getFiducialId() == 20) {
//                    //Blue
//                    indicatorLight.setPosition(0.611);
//                }
            }

            double correctedTx = result.getTx();
            double tx_deadband = 0.0167;

            //if (result.getTx() != 0) {
            //servoPos -= 0.00006 * result.getTx();
            //}

            if (Math.abs(correctedTx) > tx_deadband) {
                servoPos -= 0.0002137 * correctedTx;
            }
            else if ((Math.abs(correctedTx) < tx_deadband) && (Math.abs(correctedTx) > 0.0001)) {
                servoPos -= 0.0000257 * correctedTx;
            }
        }

        servoPos = Math.max(0.05, Math.min(0.95, servoPos));
        panServo.setPosition(servoPos);


    }

    // --- NEW MANUAL METHOD ---
    // Call this in your loop instead of runAutoVelocity
    public void runAutoVelocity(Telemetry telemetry) {

        // 2. Get the distance (Still calculated, just not used for speed)
        double currentDistance = getDistance();

        targetVelocity = (5.04145 * currentDistance) + 654.27;

        shooterMotor.setVelocity(targetVelocity);
        if(shooterMotor.getVelocity() >= targetVelocity - 20) {
            indicatorLight.setPosition(0.500);
        } else {
            indicatorLight.setPosition(0);
        }

        // 3. Telemetry: Show what velocity we are setting and the distance
        telemetry.addData("--- SHOOTER STATUS ---", "");
        telemetry.addData("Target Velocity (Set)", targetVelocity);
        telemetry.addData("Actual Motor Vel", shooterMotor.getVelocity());
        telemetry.addData("Shooter Servo Pos",panServo.getPosition());
        telemetry.addData("Distance to Target", currentDistance);
    }

    public void increaseVelocity() {
        targetVelocity += velocityIncrement;
    }

    public void decreaseVelocity() {
        targetVelocity -= velocityIncrement;
    }

    public void setZero() {
        shooterMotor.setVelocity(0);
    }

    public double getDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        double targetOffsetAngle_Vertical = result.getTy();
        double limelightMountAngleDegrees = 6;
        double limelightLensHeightInches = 14.9;

        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);
        if (angleToGoalRadians == 0) return -1;

        return (GOAL_HEIGHT - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    // Helper to get the variable value directly if needed elsewhere
    public double getTargetVelocity() {
        return targetVelocity;
    }
    public void mid() {
        panServo.setPosition(0.5);
    }
}