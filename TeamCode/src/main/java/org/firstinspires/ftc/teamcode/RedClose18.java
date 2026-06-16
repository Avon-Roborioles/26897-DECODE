package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
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

@Autonomous(name = "Red Close 18")
@Configurable
public class RedClose18 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private TurretSubsystem turret;

    private DcMotor intake;
    private Servo left, right, kicker;

    private int pathState;

    /* Red Side Coordinates */
    private static Pose startPose = new Pose(38.3074518, 60.214118, Math.toRadians(90));
    private static Pose scorePose = new Pose(16.307, 20.2, Math.toRadians(45));
    private static Pose pickup1Pose = new Pose(26.3, -1.6, Math.toRadians(0));
    private static Pose pickup1EndPose = new Pose(63.307, -1.6, Math.toRadians(0));
    private static Pose pickup2StartPose = new Pose(26.3, 19.2, Math.toRadians(0));
    private static Pose pickup2EndPose = new Pose(60.307, 19.2, Math.toRadians(0));
    private static Pose gatePose = new Pose(59.8, -3.85, Math.toRadians(22));
    private static Pose pickup3StartPose = new Pose(26.3, -22.4, Math.toRadians(0));
    private static Pose pickup3EndPose = new Pose(63.307, -22.4, Math.toRadians(0));
    private static Pose end = new Pose(26.3, 15.2, Math.toRadians(45));

    private Path scorePreload;
    private PathChain grabCycle1, grabCycle2, grabCycle3_Ground, parkPath;

    private PathChain grabCycle3_ToGate, grabCycle3_Return;
    private PathChain grabCycle4_ToGate, grabCycle4_Return;

    /* High-Speed Control Points adapted for Red Coordinates */
    // Cycle 1: Control 2 lines up perfectly on Y = -12.6 to flatten out into the intake stretch
    Pose cycle1Control1 = new Pose(26.3, 0.0, Math.toRadians(0));
    Pose cycle1Control2 = new Pose(16.3, -12.6, Math.toRadians(0));

    // Cycle 2: Control 2 lines up perfectly on Y = 19.2 to flatten out into the intake stretch
    Pose cycle2Control1 = new Pose(26.3, 30.0, Math.toRadians(0));
    Pose cycle2Control2 = new Pose(16.3, 19.2, Math.toRadians(0));

    // Cycle 3 (Ground): Control 2 lines up perfectly on Y = -22.4 to flatten out into the intake stretch
    Pose cycle3Control1 = new Pose(26.3, -10.0, Math.toRadians(0));
    Pose cycle3Control2 = new Pose(16.3, -22.4, Math.toRadians(0));

    // Gate Control Points: Reuses pickup1Pose to safely pull the path wide around center
    Pose gate3ToControl1 = new Pose(26.3, 0.0, Math.toRadians(0));
    Pose gate3ToControl2 = new Pose(26.3, -12.6, Math.toRadians(0));

    Pose gate3ReturnControl1 = new Pose(26.3, -12.6, Math.toRadians(0));
    Pose gate3ReturnControl2 = new Pose(26.3, 0.0, Math.toRadians(0));

    public void buildPaths() {
        // Preload
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

        // ==========================================
        //          GATE CYCLE 1 (First Gate Run)
        // ==========================================
        grabCycle3_ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, gate3ToControl1, gate3ToControl2, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        grabCycle3_Return = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // ==========================================
        //          GATE CYCLE 2 (Second Gate Run)
        // ==========================================
        grabCycle4_ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, gate3ToControl1, gate3ToControl2, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        grabCycle4_Return = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // ==========================================
        //          CYCLE 3 (Ground Pickup 3)
        // ==========================================
        grabCycle3_Ground = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cycle3Control1, cycle3Control2, pickup3StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3StartPose.getHeading())
                .addPath(new BezierLine(pickup3StartPose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3StartPose.getHeading(), pickup3EndPose.getHeading())
                .addPath(new BezierCurve(pickup3EndPose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePose.getHeading())
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
                // 1. Start the path (this happens instantly and keeps running in the background)
                if (!follower.isBusy() && pathTimer.getElapsedTime() < 100) {
                    follower.setMaxPower(0.67);
                    follower.followPath(scorePreload);
                }

                // 2. The Synchronous Shooting Sequence (runs WHILE driving)
                if (pathTimer.getElapsedTime() > 1200) {
                    left.setPosition(0.75);
                    right.setPosition(0.7);
                    intake.setPower(-1);
                }
                if (pathTimer.getElapsedTime() > 1800) {
                    kicker.setPosition(0.38);
                }
                if (pathTimer.getElapsedTime() > 1967) {
                    left.setPosition(0.65);
                    right.setPosition(0.8);
                    intake.setPower(0);
                    kicker.setPosition(0.015);
                }

                // 3. ONLY progress to the next state if BOTH the shooting is done AND the robot arrived
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

                        // "Exit" the scoring state and move to the next path
                        if (pathState == 10) {
                            follower.followPath(grabCycle1);
                            setPathState(11);
                        } else if (pathState == 30) {
                            follower.followPath(grabCycle3_ToGate);
                            setPathState(31);
                        } else if (pathState == 34) {
                            follower.followPath(grabCycle4_ToGate);
                            setPathState(35);
                        } else if (pathState == 40) {
                            // After Gate Cycle 2, run grabCycle2
                            follower.followPath(grabCycle2);
                            setPathState(21);
                        } else if (pathState == 20) {
                            // After grabCycle2, run the new 3rd ground cycle
                            follower.followPath(grabCycle3_Ground);
                            setPathState(41);
                        } else if (pathState == 44) {
                            // After the 3rd ground cycle is scored, head to park
                            follower.followPath(parkPath);
                            setPathState(50);
                        }
                    }
                }
                break;

            case 11: // grabCycle1 Execution
                if (follower.getCurrentPathNumber() == 1) {
                    intake.setPower(-1.0); // Active intaking during segment 1 (the BezierLine)
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    setPathState(30); // Go to score and start Gate Cycles
                }
                break;

            /* --- GATE CYCLE 1 --- */
            case 31:
                if (!follower.isBusy()) setPathState(32);
                break;

            case 32:
                intake.setPower(-1.0);
                follower.holdPoint(new Pose(59.5, -6.85, Math.toRadians(22)));
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
                follower.holdPoint(new Pose(59.5, -6.85, Math.toRadians(22)));
                if (pathTimer.getElapsedTime() > 1650) {
                    intake.setPower(0);
                    follower.followPath(grabCycle4_Return);
                    setPathState(37);
                }
                break;

            case 37:
                if (!follower.isBusy()) setPathState(40); // Move to score Gate Cycle 2
                break;

            case 21: // grabCycle2 Execution
                if (follower.getCurrentPathNumber() == 1) {
                    intake.setPower(-1.0); // Active intaking during segment 1 (the BezierLine)
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    setPathState(20); // Return to scoring for grabCycle2
                }
                break;

            case 41: // grabCycle3_Ground Execution
                if (follower.getCurrentPathNumber() == 1) {
                    intake.setPower(-1.0); // Active intaking during segment 1 (the BezierLine)
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    setPathState(44); // Return to scoring for the final ground cycle
                }
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