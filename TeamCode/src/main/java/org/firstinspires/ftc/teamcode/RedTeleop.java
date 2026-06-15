package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing Imports
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp(name = "Red Teleop")
@Configurable
public class RedTeleop extends LinearOpMode {
    // Hardware
    private DcMotor intake;
    private CRServo pass, underpass;
    private Follower follower;
    private Servo left, right,kicker;

    private TurretSubsystem turret;
    private MecanumDrivetrain drive;

    public static double kickerDelay = 3000;

    final double GOAL_HEIGHT = 29.867;

    // State Variables
    double speed = 0;

    ElapsedTime kickTimer = new ElapsedTime();
    boolean isTriggerHeld = false;


    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);

        drive = new MecanumDrivetrain(follower,hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        pass = hardwareMap.get(CRServo.class,"pass");
        kicker = hardwareMap.get(Servo.class,"kicker");
        left = hardwareMap.get(Servo.class,"left");
        right = hardwareMap.get(Servo.class,"right");

        turret = new TurretSubsystem(hardwareMap);

        follower.setStartingPose(PoseStorageRed.currentPose);

        waitForStart();
        follower.startTeleopDrive();
        turret.setDrivetrain(drive);

        while (opModeIsActive()) {
            boolean triggerPressed = gamepad1.right_trigger > 0.8;

// 1. Handle Intake/Pass Servos (Feeding Balls 1 & 2)
            if (triggerPressed) {
                left.setPosition(0.79);
                right.setPosition(0.7);
            } else {
                left.setPosition(0.62);
                right.setPosition(0.8);
            }


            follower.update();
            Vector currentVelocity = follower.getVelocity();
            // Pass the current pose from the follower to the new tracking method
            turret.updateRed(follower.getPose(),currentVelocity);

            turret.updateShootingSequence(triggerPressed);

            turret.setManualKick(gamepad1.a);


            drive.setDriveInputs(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            drive.updateDriveInputs();

            if(gamepad1.right_bumper) {
                drive.move = true;
            } else {
                drive.move = false;
            }

            if(gamepad1.yWasPressed()) {
                follower.setPose(new Pose(49.80,60.52,Math.toRadians(45)));
            }


            // 2. Manual Intake Control (Left Trigger)
            // We only let the manual intake run if the auto-shooter ISN'T running
            double intakepower = gamepad1.left_trigger;

            if(turret.getCurrentDistance() > 100 && gamepad1.right_trigger > 0.3) {
                intakepower = Range.clip(intakepower,0,0.5);
                if(gamepad1.x) {
                    intake.setPower(intakepower);
                } else {
                    intake.setPower(-intakepower);
                }
            } else {
                if (gamepad1.x) {
                    intake.setPower(intakepower);
                } else {
                    intake.setPower(-intakepower);
                }
            }


//            if (gamepad1.y){
//                if (turret.isOnTarget()) {
//                    intake.setPower(-1);
//                } else {
//                    intake.setPower(0);
//                }
//            }
            PoseStorageRed.currentPose = follower.getPose();
            telemetry.update();
        }
    }
}