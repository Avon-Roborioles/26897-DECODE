package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Vector;
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

@Configurable
public class TurretSubsystem {
    private DcMotorEx swivel;
    private Limelight3A limelight;
    private DcMotor intake;
    private CRServo pass;
    private Servo left, right, kicker;
    private DcMotorEx shooter1, shooter2;
    private Servo teamlight, colorlight;
    com.bylazar.telemetry.TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean isShooting = false;
    static double val = 4.5433;
    public static double drivekP=0.82;

    // Constants
    private final double GOAL_HEIGHT = 29.867;
    private final double LL_MOUNT_ANGLE = 8.0;
    private final double LL_LENS_HEIGHT = 14.9;

    private boolean autoInitialized = false;

    double speed = 0;

    static double GOAL_X = 65.4;
    static double GOAL_Y = 80.0;
    static double TICKS_PER_RADIAN = 653*2/Math.PI;

    private boolean trackingInitialized = false;
    private MecanumDrivetrain drivetrain;

    public static double timeOfFlight = 0.89;


    // Dip Counting Variables
    public static double RPM_DIP_THRESHOLD = 100;    // How much RPM must drop to count as a ball
    public static double RPM_RECOVERED_THRESHOLD = 25; // How close to target RPM to be "recovered"

    private int ballsFired = 0;
    private boolean rpmCurrentlyDipped = false;

    // Kicker State Machine
    private enum KickerState { WAITING, EXTENDING, RETRACTING }
    private KickerState kickerState = KickerState.WAITING;
    private ElapsedTime kickTimer = new ElapsedTime();
    private boolean manualKick = false;


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
        shooter1.setVelocityPIDFCoefficients(24, 0, 23.18, 24.53);
        shooter2.setVelocityPIDFCoefficients(24, 0, 23.18, 24.53);

        teamlight = hardwareMap.get(Servo.class,"teamlight");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void setDrivetrain(MecanumDrivetrain drivetrain){
        this.drivetrain=drivetrain;
    }

//    public void update() {
//        // --- TURRET TRACKING ---
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            double tx = result.getTx();
//            double kP = 0.0137;
//            double deadband = 0.001;
//            double minPower = 0.0313;
//
//            if (Math.abs(tx) > deadband) {
//                double motorPower = -(tx-0.8) * kP;
//                motorPower += (motorPower > 0) ? minPower : -minPower;
//                swivel.setPower(Range.clip(motorPower, -0.45, 0.45));
//            } else {
//                swivel.setPower(0);
//            }
//        } else {
//            swivel.setPower(0);
//        }
//
//        double currentdistance = getDistance();
//        if (currentdistance == -1){
//            speed = 1400;
//        }else {
//            speed = val * currentdistance + 962.48;
//        }
//        shooter1.setVelocity(-speed);
//        shooter2.setVelocity(speed);
//
//        if(shooter2.getVelocity() >= speed-25) {
//            teamlight.setPosition(0.5);
//        } else {
//            teamlight.setPosition(0);
//        }
//
//        telemetryManager.addData("shooter1 speed",shooter1.getVelocity());
//        telemetryManager.addData("shooter2 speed",shooter2.getVelocity());
//        telemetryManager.update();
//
//
//
//    }

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


    public void updateOdometryTracking(com.pedropathing.geometry.Pose robotPose, Vector robotVelocity) {

        // Initialize the tracking using run_to_position
        if (!trackingInitialized) {
            swivel.setTargetPosition(swivel.getCurrentPosition());
            swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackingInitialized = true;
        }

        double virtualGoalX = GOAL_X - (robotVelocity.getXComponent() * timeOfFlight);
        double virtualGoalY = GOAL_Y - (robotVelocity.getYComponent() * timeOfFlight);

        // Calculate deltas based on the VIRTUAL goal, not the actual goal
        double deltaX = virtualGoalX - robotPose.getX();
        double deltaY = virtualGoalY - robotPose.getY();

        double absoluteAngleToGoal = Math.atan2(deltaY, deltaX);

        // Calculate desired relative angle
        double targetRelativeAngle = absoluteAngleToGoal - robotPose.getHeading();

        // Get current turret angle based on encoder ticks
        double currentTurretAngle = swivel.getCurrentPosition() / TICKS_PER_RADIAN;

        // Find the SHORTEST distance between current and target
        double angleError = targetRelativeAngle - currentTurretAngle;
        telemetryManager.addData("Raw Angle Error",angleError);

        // Normalize the ERROR so it never moves more than 180 degrees
        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;
        telemetryManager.addData("Normalized Angle Error",angleError);

//        angleError = Range.clip(angleError,-Math.PI,0);
//        telemetryManager.addData("Clipped Angle Error",angleError);

        // Set new target based on current position + shortest path
        double clip = Range.clip((currentTurretAngle + angleError),Math.toRadians(-110),Math.toRadians(110));
        int targetTicks = (int) (clip * TICKS_PER_RADIAN);

        swivel.setTargetPosition(targetTicks);
        swivel.setPower(1);

        drivetrain.turretRequestTurn(clip*drivekP);


        double currentDistance = Math.hypot(deltaX, deltaY);
        speed = val * currentDistance + 962.48;


        double currentVel = shooter2.getVelocity();
        double error = speed - currentVel;
        if (Math.abs(error) > 200) {
            // FULL SEND: The wheels need 100% power to overcome inertia quickly
            shooter2.setPower(1.0);
        } else if (Math.abs(error) > 75) {
            // APPROACH: Lower power slightly so we don't fly past the target speed
            shooter2.setPower(0.8);
        } else {
            // PRECISION: Let the PIDF stabilize the 10-20 RPM variance
            shooter2.setVelocity(speed);
        }
        shooter1.setPower(-shooter2.getPower());



        if (shooter2.getVelocity() >= speed - 25) {
            teamlight.setPosition(0.5);
        } else {
            teamlight.setPosition(0);
        }

        telemetryManager.addData("Distance", currentDistance);
        telemetryManager.addData("Target Ticks", targetTicks);
        telemetryManager.addData("Current Ticks", swivel.getCurrentPosition());
        telemetryManager.addData("shooter1 speed",shooter1.getVelocity());
        telemetryManager.addData("shooter2 speed",shooter2.getVelocity());
        telemetryManager.update();
    }
    public static double tolerance=16.7;

    public boolean isOnTarget(){
        if (Math.abs(shooter2.getVelocity() - speed)<tolerance){
            return true;
        }
        return false;
    }

    public void updateShootingSequence(boolean isTriggerHeld) {
        // --- MANUAL OVERRIDE (e.g., for the A button) ---
        if (manualKick) {
            kicker.setPosition(0.30);
            return;
        }

        // --- TRIGGER RELEASED: Reset Everything ---
        if (!isTriggerHeld) {
            ballsFired = 0;
            rpmCurrentlyDipped = false;
            kickerState = KickerState.WAITING;
            kicker.setPosition(0.012); // Retracted
            return;
        }

        // 1. Calculate how far we are from our target speed
        double rpmError = speed - shooter2.getVelocity();

        // 2. Detect the Dip (Ball is currently passing through)
        if (rpmError > RPM_DIP_THRESHOLD) {
            rpmCurrentlyDipped = true;
        }

        // 3. Detect the Recovery (Ball has completely left)
        if (rpmCurrentlyDipped && rpmError <= RPM_RECOVERED_THRESHOLD) {
            ballsFired++;
            rpmCurrentlyDipped = false; // Reset flag to look for the next ball
        }

        // 4. Kicker Logic: Only fire if 2 balls have left AND we are back up to speed
        switch (kickerState) {
            case WAITING:
                if (ballsFired >= 2 && rpmError <= RPM_RECOVERED_THRESHOLD) {
                    kicker.setPosition(0.30); // Fire 3rd ball
                    kickTimer.reset();
                    kickerState = KickerState.EXTENDING;
                }
                break;

            case EXTENDING:
                if (kickTimer.milliseconds() > 150) { // Wait for servo to physically push
                    kicker.setPosition(0.012); // Retract
                    kickTimer.reset();
                    kickerState = KickerState.RETRACTING;
                }
                break;

            case RETRACTING:
                if (kickTimer.milliseconds() > 150) {
                    // Done. Sequence stays here until trigger is released.
                }
                break;
        }

        // Optional: Add telemetry to FTC Dashboard to help you tune the thresholds
        telemetryManager.addData("Balls Fired", ballsFired);
        telemetryManager.addData("RPM Error", rpmError);
    }

    // Keep this so your TeleOp can still fire the kicker manually
    public void setManualKick(boolean manual) {
        this.manualKick = manual;
    }
}