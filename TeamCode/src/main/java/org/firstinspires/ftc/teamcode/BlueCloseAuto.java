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

@Autonomous(name = "Hopefully Blue works or imma crash out")
public class BlueCloseAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private TurretSubsystem turret;

    private DcMotor intake;
    private Servo left, right, kicker;

    private int pathState;

    /* Blue Side Coordinates */
    private final Pose startPose = new Pose(-39.502575, 58.250357, Math.toRadians(90));
    private final Pose scorePose = new Pose(-15.502575, 12.75, Math.toRadians(136.5));
    private final Pose pickup1Pose = new Pose(-14.502, -25.35, Math.toRadians(180));
    private final Pose pickup1EndPose = new Pose(-46.733, -25.35, Math.toRadians(180));
    private final Pose pickup2StartPose = new Pose(-14.502, 5.25, Math.toRadians(180));
    private final Pose pickup2EndPose = new Pose(-40.503, 5.25, Math.toRadians(180));

    private final Pose gatePose = new Pose(-57.746, -14.866, Math.toRadians(158));
    private final Pose end = new Pose(-6.503, -30.25, Math.toRadians(143));

    private Path scorePreload;
    private PathChain grabCycle1, grabCycle2, parkPath;
    private PathChain grabCycle3_ToGate, grabCycle3_Return;
    private PathChain grabCycle4_ToGate, grabCycle4_Return;
    private PathChain grabCycle5_ToGate, grabCycle5_Return;

    Pose cycle1Control1 = new Pose(-14.5, 0.0, Math.toRadians(180));

    // Control 2 is placed 10 inches BEFORE pickup1Pose on the exact same Y-axis (-25.35).
    Pose cycle1Control2 = new Pose(-4.5, -25.35, Math.toRadians(180));

    Pose cycle2Control1 = new Pose(-14.5, 25.0, Math.toRadians(180));

    // Control 2 is placed 10 inches BEFORE pickup2StartPose on the exact same Y-axis (11.25).
    Pose cycle2Control2 = new Pose(-4.5, 11.25, Math.toRadians(180));

    // Control points to safely guide the robot out of score and around the field center
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
                // 1. Arc smoothly across the field and straighten out perfectly upon arrival at pickup1Pose
                .addPath(new BezierCurve(scorePose, cycle1Control1, cycle1Control2, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())

                // 2. The critical intake stretch! A perfectly rigid, straight line for swallowing the balls
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())

                // 3. Straight shot back to score (or use another curve here if clear)
                .addPath(new BezierLine(pickup1EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

        // Cycle 2: Score -> Pickup 2 -> Score
        grabCycle2 = follower.pathBuilder()
                // 1. Arc smoothly across the field and flatten out perfectly right as you hit the start of the intake run
                .addPath(new BezierCurve(scorePose, cycle2Control1, cycle2Control2, pickup2StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2StartPose.getHeading())

                // 2. The critical intake stretch! A perfectly straight line for swallowing the balls
                .addPath(new BezierLine(pickup2StartPose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(), pickup2EndPose.getHeading())

                // 3. Straight shot back to the scoring pose
                .addPath(new BezierLine(pickup2EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePose.getHeading())
                .build();

        // Cycle 3: Score -> gatePose with Hold
        grabCycle3_ToGate = follower.pathBuilder()
                // Arc beautifully from score, around the center, directly into the exact gate spot
                .addPath(new BezierCurve(scorePose, gate3ToControl1, gate3ToControl2, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        grabCycle3_Return = follower.pathBuilder()
                // Arc beautifully back from the gate directly to the score pose
                .addPath(new BezierCurve(gatePose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();
        // Gate Cycle 2
        grabCycle4_ToGate = follower.pathBuilder()
                // Arc beautifully from score, around the center, directly into the exact gate spot
                .addPath(new BezierCurve(scorePose, gate3ToControl1, gate3ToControl2, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        grabCycle4_Return = follower.pathBuilder()
                // Arc beautifully back from the gate directly to the score pose
                .addPath(new BezierCurve(gatePose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Gate Cycle 3
        grabCycle5_ToGate = follower.pathBuilder()
                // Arc beautifully from score, around the center, directly into the exact gate spot
                .addPath(new BezierCurve(scorePose, gate3ToControl1, gate3ToControl2, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        grabCycle5_Return = follower.pathBuilder()
                // Arc beautifully back from the gate directly to the score pose
                .addPath(new BezierCurve(gatePose, gate3ReturnControl1, gate3ReturnControl2, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
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
                follower.followPath(scorePreload);
                setPathState(10);
                break;

            // All scoring states run this shooting sequence
            case 10:
            case 20:
            case 30:
            case 34: // Scoring after Gate Cycle 1
            case 40: // Scoring after Gate Cycle 2
            case 44: // Scoring after Gate Cycle 3
                if (!follower.isBusy()) {
                    /* --- START OF SCORING SEQUENCE --- */
                    if (pathTimer.getElapsedTime() > 50) {
                        left.setPosition(0.75);
                        right.setPosition(0.7);
                        intake.setPower(-1);
                    }
                    if (pathTimer.getElapsedTime() > 520) {
                        kicker.setPosition(0.38);
                    }
                    if (pathTimer.getElapsedTime() > 620) {
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
                            follower.followPath(grabCycle5_ToGate);
                            setPathState(41);
                        } else if (pathState == 44) {
                            // After Gate Cycle 3, run grabCycle2
                            follower.followPath(grabCycle2);
                            setPathState(21);
                        } else if (pathState == 20) {
                            // After grabCycle2 is scored, head to park
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
                follower.holdPoint(gatePose);
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
                follower.holdPoint(gatePose);
                if (pathTimer.getElapsedTime() > 1650) {
                    intake.setPower(0);
                    follower.followPath(grabCycle4_Return);
                    setPathState(37);
                }
                break;

            case 37:
                if (!follower.isBusy()) setPathState(40);
                break;

            /* --- GATE CYCLE 3 --- */
            case 41:
                if (!follower.isBusy()) setPathState(42);
                break;

            case 42:
                intake.setPower(-1.0);
                follower.holdPoint(gatePose);
                if (pathTimer.getElapsedTime() > 1650) {
                    intake.setPower(0);
                    follower.followPath(grabCycle5_Return);
                    setPathState(43);
                }
                break;

            case 43:
                if (!follower.isBusy()) setPathState(44);
                break;

            case 21: // grabCycle2 Execution (Moved to end)
                if (follower.getCurrentPathNumber() == 1) {
                    intake.setPower(-1.0); // Active intaking during segment 1 (the BezierLine)
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    setPathState(20); // Return to scoring for grabCycle2
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