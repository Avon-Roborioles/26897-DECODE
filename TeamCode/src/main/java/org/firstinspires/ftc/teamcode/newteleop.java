package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.Timing;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Pedro Pathing Imports
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "New Teleop")
public class newteleop extends LinearOpMode {
    // Hardware
    private DcMotor intake;
    private CRServo pass, underpass;
    private Follower follower;
    private Servo left, right,kicker;

    private TurretSubsystem turret;



    final double GOAL_HEIGHT = 29.867;

    // State Variables
    double speed = 0;

    ElapsedTime kickTimer = new ElapsedTime();
    boolean isTriggerHeld = false;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        pass = hardwareMap.get(CRServo.class,"pass");
        kicker = hardwareMap.get(Servo.class,"kicker");
        left = hardwareMap.get(Servo.class,"left");
        right = hardwareMap.get(Servo.class,"right");

        turret = new TurretSubsystem(hardwareMap);

        follower.setStartingPose(new Pose(0, 0, Math.PI/2));

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            boolean triggerPressed = gamepad1.right_trigger > 0.8;

            follower.update();

            // Pass the current pose from the follower to the new tracking method
            turret.updateOdometryTracking(follower.getPose());


            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            if (gamepad1.right_trigger > 0.8) {

                pass.setPower(-1);

                left.setPosition(0.79);

                right.setPosition(0.7);

                if (!isTriggerHeld) {

                    kickTimer.reset();

                    isTriggerHeld = true;

                }

            } else {

                pass.setPower(0);

                left.setPosition(0.62);

                right.setPosition(0.8);
                isTriggerHeld = false;

            }

            if (gamepad1.a || (isTriggerHeld && kickTimer.milliseconds() >= 999)) {

                kicker.setPosition(0.30);

            } else {

                kicker.setPosition(0.012);

            }


            // 2. Manual Intake Control (Left Trigger)
            // We only let the manual intake run if the auto-shooter ISN'T running
            if(gamepad1.x) {
                intake.setPower(gamepad1.left_trigger);
            } else {
                intake.setPower(-gamepad1.left_trigger);
            }

            telemetry.update();
        }
    }
}