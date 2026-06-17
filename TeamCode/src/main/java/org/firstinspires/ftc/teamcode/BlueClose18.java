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

@Autonomous(name = "Blue Close 18")
public class BlueClose18 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private TurretSubsystem turret;

    private DcMotor intake;
    private Servo left, right, kicker;

    private int pathState;

    /* Blue Side Coordinates */
    private final Pose startPose = new Pose(-39.502575, 58.250357, Math.toRadians(90));
    private final Pose scorePose = new Pose(-15.502575, 12.75, Math.toRadians(136.5));
    private final Pose pickup1Pose = new Pose(-14.502, -23.35, Math.toRadians(180));
    private final Pose pickup1EndPose = new Pose(-46.733, -23.35, Math.toRadians(180));
    private final Pose pickup2StartPose = new Pose(-14.502, 1.25, Math.toRadians(180));
    private final Pose pickup2EndPose = new Pose(-40.503, 1.25, Math.toRadians(180));
    private final Pose pickup3StartPose = new Pose(-14.502, -47.45, Math.toRadians(180));
    private final Pose pickup3EndPose = new Pose(-46.733, -47.45, Math.toRadians(180));
    private final Pose gatePose = new Pose(-56.446, -18.866, Math.toRadians(158));
    private final Pose end = new Pose(-51.503, 0.25, Math.toRadians(143));

    // Your specific hold pose for gate cycles
    private final Pose gateHoldPose = new Pose(-56.146, -25.266, Math.toRadians(158));

    private Path scorePreload;
    private PathChain grabCycle1, grabCycle2, grabCycle3_Ground, parkPath;
    private PathChain grabCycle3_ToGate, grabCycle3_Return;
    private PathChain grabCycle4_ToGate, grabCycle4_Return;

    Pose cycle1Control1 = new Pose(-14.5, 0.0, Math.toRadians(180));
    Pose cycle1Control2 = new Pose(-4.5, -25.35, Math.toRadians(180));

    Pose cycle2Control1 = new Pose(-14.5, 25.0, Math.toRadians(180));
    Pose cycle2Control2 = new Pose(-4.5, 11.25, Math.toRadians(180));

    // FIXED: Corrected control points matching Blue Side ground pickup 3
    Pose cycle3Control1 = new Pose(-14.5, -30.0, Math.toRadians(180));
    Pose cycle3Control2 = new Pose(-4.5, -47.45, Math.toRadians(180));

    Pose gate3ToControl1 = new Pose(-14.5, 0.0, Math.toRadians(180));
    Pose gate3ToControl2 = new Pose(-14.5, -25.35, Math.toRadians(180));

    Pose gate3ReturnControl1 = new Pose(-14.5, -25.35, Math.toRadians(180));
    Pose gate3ReturnControl2 = new Pose(-14.5, 0.0, Math.toRadians(180));

    public void buildPaths() {
        // Preload Path
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Cycle 1: Score -> Pickup 1 -> Score
        grabCycle1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cycle1Control1, cycle1Control2, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .addPath(new BezierCurve(pickup1EndPose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

        // Cycle 2: Score -> Pickup 2 -> Score
        grabCycle2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cycle2Control1, cycle2Control2, pickup2StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2StartPose.getHeading())
                .addPath(new BezierLine(pickup2StartPose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(), pickup2EndPose.getHeading())
                .addPath(new BezierLine(pickup2EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePose.getHeading())
                .build();

        // Gate Cycle 1
        grabCycle3_ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, gate3ToControl1, gate3ToControl2, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        grabCycle3_Return = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Gate Cycle 2
        grabCycle4_ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, gate3ToControl1, gate3ToControl2, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        grabCycle4_Return = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Cycle 3 off the ground
        grabCycle3_Ground = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cycle3Control1, cycle3Control2, pickup3StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3StartPose.getHeading())
                .addPath(new BezierLine(pickup3StartPose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3StartPose.getHeading(), pickup3EndPose.getHeading())
                .addPath(new BezierCurve(pickup3EndPose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePose.getHeading())
                .build();

        // Park Path
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, end))
                .setLinearHeadingInterpolation(scorePose.getHeading(), end.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to Preload Score
                if (!follower.isBusy() && pathTimer.getElapsedTime() < 100) {
                    follower.setMaxPower(0.67);
                    follower.followPath(scorePreload);
                }

                if (pathTimer.getElapsedTime() > 1200) {
                    left.setPosition(0.75);
                    right.setPosition(0.7);
                    intake.setPower(-1);
                }
                if (pathTimer.getElapsedTime() > 1867) {
                    kicker.setPosition(0.38);
                }
                if (pathTimer.getElapsedTime() > 1967) {
                    left.setPosition(0.65);
                    right.setPosition(0.8);
                    intake.setPower(0);
                    kicker.setPosition(0.015);
                }

                if (pathTimer.getElapsedTime() > 1667 && !follower.isBusy()) {
                    follower.setMaxPower(1);
                    setPathState(10);
                }
                break;

            // All scoring states run this shooting sequence
            case 10:
            case 20:
            case 30:
            case 34: // Scoring after Gate Cycle 1
            case 40: // Scoring after Gate Cycle 2
            case 44: // Scoring after Ground Cycle 3
                if (!follower.isBusy()) {
                    /* --- START OF SCORING SEQUENCE --- */
                    if (pathTimer.getElapsedTime() > 50) {
                        left.setPosition(0.75);
                        right.setPosition(0.7);
                        intake.setPower(-1);
                    }
                    if (pathTimer.getElapsedTime() > 650) {
                        kicker.setPosition(0.38);
                    }
                    if (pathTimer.getElapsedTime() > 817) {
                        left.setPosition(0.65);
                        right.setPosition(0.8);
                        intake.setPower(0);
                        kicker.setPosition(0.015);
                        /* --- END OF SCORING SEQUENCE --- */

                        // Routing logic based on previous completed step
                        if (pathState == 10) {
                            follower.setMaxPower(1);
                            follower.followPath(grabCycle1);
                            setPathState(11);
                        } else if (pathState == 30) {
                            follower.followPath(grabCycle3_ToGate);
                            setPathState(31);
                        } else if (pathState == 34) {
                            follower.followPath(grabCycle4_ToGate);
                            setPathState(35);
                        } else if (pathState == 40) {
                            // After Gate Cycle 2 is completed and scored, transition to Ground Cycle 2
                            follower.followPath(grabCycle2);
                            setPathState(21);
                        } else if (pathState == 20) {
                            // After Ground Cycle 2 is completed and scored, run Ground Cycle 3
                            follower.followPath(grabCycle3_Ground);
                            setPathState(41);
                        } else if (pathState == 44) {
                            // Final Score Complete -> Move to Park
                            follower.followPath(parkPath);
                            setPathState(50);
                        }
                    }
                }
                break;

            case 11: // grabCycle1 Execution
                if (follower.getCurrentPathNumber() == 1) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    setPathState(30);
                }
                break;

            /* --- GATE CYCLE 1 --- */
            case 31:
                if (!follower.isBusy()) setPathState(32);
                break;

            case 32:
                intake.setPower(-1.0);
                follower.holdPoint(gateHoldPose); // Updated with corrected hold point coordinates
                if (pathTimer.getElapsedTime() > 1650) {
                    intake.setPower(0);
                    follower.followPath(grabCycle3_Return);
                    setPathState(33);
                }
                break;

            case 33:
                if (!follower.isBusy()) setPathState(34);
                break;

            /* --- GATE CYCLE 2 --- */
            case 35:
                if (!follower.isBusy()) setPathState(36);
                break;

            case 36:
                intake.setPower(-1.0);
                follower.holdPoint(gateHoldPose); // Updated with corrected hold point coordinates
                if (pathTimer.getElapsedTime() > 1650) {
                    intake.setPower(0);
                    follower.followPath(grabCycle4_Return);
                    setPathState(37);
                }
                break;

            case 37:
                if (!follower.isBusy()) setPathState(40);
                break;

            case 21: // grabCycle2 Execution
                if (follower.getCurrentPathNumber() == 1) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;

            case 41: // grabCycle3_Ground Execution
                if (follower.getCurrentPathNumber() == 1) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    setPathState(44);
                }
                break;

            case 50: // Final Parking
                if (!follower.isBusy()) {
                    setPathState(-1);
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
        turret.updateBlue(follower.getPose(), follower.getVelocity());
        PoseStorageBlue.currentPose = follower.getPose();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Current Path Segment", follower.getCurrentPathNumber());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
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
        PoseStorageBlue.currentPose = follower.getPose();
    }
}