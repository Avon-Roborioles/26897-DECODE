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
    static double val = 4.17391;
    static double y_int = 900;
    public double drivekP=1;
    public static double useSpeed = 1;

    // Constants
    private final double GOAL_HEIGHT = 29.867;
    private final double LL_MOUNT_ANGLE = 8.0;
    private final double LL_LENS_HEIGHT = 14.9;

    private boolean autoInitialized = false;

    public static double speed = 50;

    double GOAL_X = 65;
    double GOAL_Y = 75;
    double AIM_GOAL_Y = 75;
    double AIM_GOAL_X = 65;
    double TICKS_PER_RADIAN = 653*2/Math.PI;

    private boolean trackingInitialized = false;
    private MecanumDrivetrain drivetrain;

    public static double timeOfFlight = 0.567;


    // Dip Counting Variables
    public double RPM_DIP_THRESHOLD = -150;    // How much RPM must drop to count as a ball
    public double RPM_RECOVERED_THRESHOLD = 100; // How close to target RPM to be "recovered"

    private int ballsFired = 0;
    private boolean rpmCurrentlyDipped = false;

    // Kicker State Machine
    private enum KickerState { WAITING, EXTENDING, RETRACTING }
    private KickerState kickerState = KickerState.WAITING;
    private ElapsedTime kickTimer = new ElapsedTime();
    private boolean manualKick = false;

    public double kP = 50;
    public double kD = 0;
    public static double kF = 21.3;


    // Blue Side Goal Variables
    static double BLUE_GOAL_X = -65;
    static double BLUE_GOAL_Y = 65;
    static double BLUE_AIM_GOAL_Y = 65;
    static double BLUE_AIM_GOAL_X = -65;

    private double currentDistance = 0;


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
        shooter1.setVelocityPIDFCoefficients(kP,0,kD,kF);
        shooter2.setVelocityPIDFCoefficients(kP,0,kD,kF);

        teamlight = hardwareMap.get(Servo.class,"teamlight");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public double getCurrentDistance() {
        return currentDistance;
    }

    public void setDrivetrain(MecanumDrivetrain drivetrain){
        this.drivetrain=drivetrain;
    }

    // Call this in TeleOp when trigger is held
    public void setShooting(boolean shooting) {
        if (shooting && !isShooting) {
            shootTimer.reset(); // Start timer the moment shooting starts
        }
        this.isShooting = shooting;
    }


    public void updateRed(com.pedropathing.geometry.Pose robotPose, Vector robotVelocity) {
        // Initialize the tracking using run_to_position
        if (!trackingInitialized) {
            swivel.setTargetPosition(swivel.getCurrentPosition());
            swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackingInitialized = true;
        }


        double virtualGoalX = AIM_GOAL_X - (robotVelocity.getXComponent() * timeOfFlight);
        double virtualGoalY = AIM_GOAL_Y - (robotVelocity.getYComponent() * timeOfFlight);


        double distancey = GOAL_Y - (robotVelocity.getYComponent() * timeOfFlight);
        double distancex = GOAL_X - (robotVelocity.getXComponent() * timeOfFlight);

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

        //drivetrain.turretRequestTurn(clip*drivekP);


        currentDistance = Math.hypot(deltaX, deltaY);
        if(useSpeed == 1){speed = -0.0000517848*Math.pow(currentDistance,4)+0.0190175*Math.pow(currentDistance,3)-2.48692*Math.pow(currentDistance,2)+141.46378*currentDistance-1620.32148;}

        double currentError = Math.abs(speed - shooter1.getVelocity());
        double activeKP = kP;

        if (currentError > 50) {
            activeKP = kP * 7.7;
        }

        shooter2.setVelocityPIDFCoefficients(activeKP, 0, kD, kF);
        shooter1.setVelocityPIDFCoefficients(activeKP, 0, kD, kF);


        shooter2.setVelocity(-speed);
        // Shooter 2 is the master, Shooter 1 is the slave it follows all the commands via the master
        shooter1.setVelocity(speed);



        if (shooter1.getVelocity() >= speed - 25) {
            teamlight.setPosition(0.5);
        } else {
            teamlight.setPosition(0);
        }

        telemetryManager.addData("Distance", currentDistance);
        telemetryManager.addData("shooter2 speed",shooter2.getVelocity());
        telemetryManager.addData("Target Speed",speed);
        telemetryManager.addData("X Pos",robotPose.getX());
        telemetryManager.addData("Y Pos",robotPose.getY());
        telemetryManager.update();
    }

    public void updateBlue(com.pedropathing.geometry.Pose robotPose, Vector robotVelocity) {
        // Initialize the tracking using run_to_position
        if (!trackingInitialized) {
            swivel.setTargetPosition(swivel.getCurrentPosition());
            swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackingInitialized = true;
        }

        // Using BLUE aim variables
        double virtualGoalX = BLUE_AIM_GOAL_X - (robotVelocity.getXComponent() * timeOfFlight);
        double virtualGoalY = BLUE_AIM_GOAL_Y - (robotVelocity.getYComponent() * timeOfFlight);

        // Using BLUE distance variables
        double distancey = BLUE_GOAL_Y - (robotVelocity.getYComponent() * timeOfFlight);
        double distancex = BLUE_GOAL_X - (robotVelocity.getXComponent() * timeOfFlight);

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

        //drivetrain.turretRequestTurn(clip*drivekP);


        currentDistance = Math.hypot(deltaX, deltaY);
        if(useSpeed == 1){speed = -0.0000517848*Math.pow(currentDistance,4)+0.0190175*Math.pow(currentDistance,3)-2.48692*Math.pow(currentDistance,2)+141.46378* currentDistance -1620.32148;}

        double currentError = Math.abs(speed - shooter1.getVelocity());
        double activeKP = kP;

        if (currentError > 50) {
            activeKP = kP*7.70; // 670% boost during recovery or during changes while moving
        }

        shooter2.setVelocityPIDFCoefficients(activeKP, 0, kD, kF);
        shooter1.setVelocityPIDFCoefficients(activeKP, 0, kD, kF);


        shooter2.setVelocity(-speed);
        // Shooter 2 is the master, Shooter 1 is the slave it follows all the commands via the master
        shooter1.setVelocity(speed);



        if (shooter1.getVelocity() >= speed - 25) {
            teamlight.setPosition(0.5);
        } else {
            teamlight.setPosition(0);
        }

        telemetryManager.addData("Distance", currentDistance);
        telemetryManager.addData("shooter2 speed",shooter2.getVelocity());
        telemetryManager.addData("Target Speed",speed);
        telemetryManager.addData("X Pos",robotPose.getX());
        telemetryManager.addData("Y Pos",robotPose.getY());
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
        // --- MANUAL OVERRIDE ---
        if (manualKick) {
            kicker.setPosition(0.38);
            return;
        }

        // --- TRIGGER RELEASED: Reset Everything ---
        if (!isTriggerHeld) {
            kickerState = KickerState.WAITING;
            kicker.setPosition(0.012); // Retracted


            kickTimer.reset();
            return;
        }

        // --- TRIGGER HELD: 600ms Timer Sequence ---
        switch (kickerState) {
            case WAITING:
                // Wait 600ms before kicking
                if (kickTimer.milliseconds() > 600) {
                    kicker.setPosition(0.38); // Fire ball
                    kickTimer.reset();
                    kickerState = KickerState.EXTENDING;
                }
                break;

            case EXTENDING:
                // Wait 150ms for the servo to physically push the ball
                if (kickTimer.milliseconds() > 315) {
                    kicker.setPosition(0.012); // Retract
                    kickTimer.reset();
                    kickerState = KickerState.RETRACTING;
                }
                break;

            case RETRACTING:
                // Done. Sequence stays here until the trigger is released.
                break;
        }
    }

    // Keep this so your TeleOp can still fire the kicker manually
    public void setManualKick(boolean manual) {
        this.manualKick = manual;
    }

}