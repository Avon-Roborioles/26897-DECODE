package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@Autonomous(name = "Hopefully Goated Red")
public class RedCloseAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private TurretSubsystem turret;

    private DcMotor intake;
    private Servo left, right, kicker;

    private int pathState;

    private final Pose startPose = new Pose(38.3074518, 60.214118, Math.toRadians(90));
    private final Pose scorePose = new Pose(13.307, 25.2, Math.toRadians(45));
    private final Pose pickup1Pose = new Pose(26.3, 1.6, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(63.307, 1.6, Math.toRadians(0));
    private final Pose pickup2StartPose = new Pose(26.3, 27.2, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(60.307, 27.2, Math.toRadians(0));
    private final Pose gatePose = new Pose(63.307, 8.6, Math.toRadians(31));
    private final Pose end = new Pose(26.3, 15.2, Math.toRadians(45));

    private Path scorePreload;
    private PathChain grabCycle1, grabCycle2, grabCycle3_ToGate, grabCycle3_Return, parkPath;

    public void buildPaths() {
        // Preload
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Cycle 1: Score -> Pickup 1 -> Score (3 segments, fluid motion)
        grabCycle1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose)) // Go to start of intake
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose)) // Sweep/Intake
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .addPath(new BezierLine(pickup1EndPose, scorePose)) // Return to score
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

        // Cycle 2: Score -> Pickup 2 -> Score (3 segments, fluid motion)
        grabCycle2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2StartPose.getHeading())
                .addPath(new BezierLine(pickup2StartPose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(), pickup2EndPose.getHeading())
                .addPath(new BezierLine(pickup2EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePose.getHeading())
                .build();



        grabCycle3_ToGate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, gatePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), gatePose.getHeading())
                .build();

        grabCycle3_Return = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Park
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, end))
                .setLinearHeadingInterpolation(scorePose.getHeading(), end.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to Preload Score
                follower.followPath(scorePreload);
                setPathState(10);
                break;

            // All scoring states run this shooting sequence
            case 10:
            case 20:
            case 30:
            case 40:
                if (!follower.isBusy()) {
                    /* --- START OF SCORING SEQUENCE --- */
                    if (pathTimer.getElapsedTime() > 150) {
                        left.setPosition(0.75);
                        right.setPosition(0.7);
                        intake.setPower(-1);
                    }
                    if (pathTimer.getElapsedTime() > 800) {
                        kicker.setPosition(0.38);
                    }
                    if (pathTimer.getElapsedTime() > 1200) {
                        left.setPosition(0.65);
                        right.setPosition(0.8);
                        intake.setPower(0);
                        kicker.setPosition(0.015);
                        /* --- END OF SCORING SEQUENCE --- */

                        // "Exit" the scoring state and move to the next path
                        if (pathState == 10) {
                            follower.followPath(grabCycle1);
                            setPathState(11);
                        } else if (pathState == 20) {
                            follower.followPath(grabCycle2);
                            setPathState(21);
                        } else if (pathState == 30) {
                            follower.followPath(grabCycle3_ToGate);
                            setPathState(31);
                        } else if (pathState == 40) {
                            follower.followPath(parkPath);
                            setPathState(50);
                        }
                    }
                }
                break;

            case 11: //Cycle 1
                if (follower.getCurrentPathNumber() == 1) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    setPathState(20); // Return to scoring for Cycle 1
                }
                break;

            case 21: //Cycle 2
                if (follower.getCurrentPathNumber() == 1) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    setPathState(30); // Return to scoring for Cycle 2
                }
                break;

            case 31: // Cycle 3
                if (!follower.isBusy()) setPathState(32);
                break;

            case 32:
                intake.setPower(-1.0);
                follower.holdPoint(gatePose); // Brake and hold position
                if (pathTimer.getElapsedTime() > 4000) {
                    intake.setPower(0);
                    follower.followPath(grabCycle3_Return);
                    setPathState(33);
                }
                break;

            case 33: // Cycle 3 - Return to Score
                if (!follower.isBusy()) setPathState(40); // Final shooting before park
                break;

            case 50: // Final Parking
                if (!follower.isBusy()) {
                    setPathState(-1); // Finished
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        turret.updateRed(follower.getPose(), follower.getVelocity());
        PoseStorageRed.currentPose = follower.getPose();

        telemetry.addData("Path State", pathState);
        telemetry.addData("T-Value", follower.getCurrentTValue());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        intake = hardwareMap.get(DcMotor.class, "intake");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        kicker = hardwareMap.get(Servo.class, "kicker");
        turret = new TurretSubsystem(hardwareMap);

        left.setPosition(0.65);
        right.setPosition(0.8);
        kicker.setPosition(0.015);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        PoseStorageRed.currentPose = follower.getPose();
    }
}