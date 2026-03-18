package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {
    private DcMotorEx swivel;
    private Limelight3A limelight;
    private DcMotor intake;
    private CRServo pass;
    private Servo left, right, kicker;
    private DcMotorEx shooter1, shooter2;
    private Servo teamlight, colorlight;

    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean isShooting = false;

    // Constants
    private final double GOAL_HEIGHT = 29.867;
    private final double LL_MOUNT_ANGLE = 8.0;
    private final double LL_LENS_HEIGHT = 14.9;

    private boolean autoInitialized = false;

    double speed = 0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        swivel = hardwareMap.get(DcMotorEx.class, "swivel");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pass = hardwareMap.get(CRServo.class, "pass");
        kicker = hardwareMap.get(Servo.class, "kicker");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class,"shooter2");
        shooter1.setVelocityPIDFCoefficients(101, 0, 23.18, 19.11);
        shooter2.setVelocityPIDFCoefficients(101, 0, 23.18, 19.11);

        teamlight = hardwareMap.get(Servo.class,"teamlight");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void update() {
        // --- TURRET TRACKING ---
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double kP = 0.0137;
            double deadband = 0.001;
            double minPower = 0.0313;

            if (Math.abs(tx) > deadband) {
                double motorPower = -(tx-0.8) * kP;
                motorPower += (motorPower > 0) ? minPower : -minPower;
                swivel.setPower(Range.clip(motorPower, -0.45, 0.45));
            } else {
                swivel.setPower(0);
            }
        } else {
            swivel.setPower(0);
        }

        double currentdistance = getDistance();
        if (currentdistance == -1){
            speed = 1400;
        }else {
            speed = 6.7313 * currentdistance + 962.48;
        }
        shooter1.setVelocity(-speed);
        shooter2.setVelocity(speed);

        if(shooter2.getVelocity() >= speed-25) {
            teamlight.setPosition(0.5);
        } else {
            teamlight.setPosition(0);
        }


    }

    public void updatebutauto() {
        // --- INITIALIZATION (Runs only once) ---
        if (!autoInitialized) {
            swivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            swivel.setTargetPosition(0); // Lock it to the starting position
            swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoInitialized = true;
        }

        // --- HOLD POSITION ---
        // You MUST provide power in RUN_TO_POSITION so the motor can fight resistance to hold 0.
        // The motor will only use as much of this power as it needs to stay at exactly 0.
        swivel.setPower(0.45);

        // --- SHOOTER ---
        speed = 1280;
        shooter1.setVelocity(-speed);
        shooter2.setVelocity(speed);

        if (shooter2.getVelocity() >= speed - 25) {
            teamlight.setPosition(0.5);
        }
    }

    // Call this in TeleOp when trigger is held
    public void setShooting(boolean shooting) {
        if (shooting && !isShooting) {
            shootTimer.reset(); // Start timer the moment shooting starts
        }
        this.isShooting = shooting;
    }

    public double getDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;
        double targetOffsetAngle_Vertical = result.getTy();
        double angleToGoalRadians = Math.toRadians(LL_MOUNT_ANGLE + targetOffsetAngle_Vertical);
        return (GOAL_HEIGHT - LL_LENS_HEIGHT) / Math.tan(angleToGoalRadians);
    }
}