package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@Autonomous(name = "Red 12 Ball Auto", group = "12")
public class Red12BallAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Timer kicktimer = new Timer();
    private TurretSubsystem turret;

    private DcMotor intake;
    private Servo left, right, kicker;

    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(-25, -35, Math.toRadians(54)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(-12, -58.6, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1EndPose = new Pose(24, -58.6, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2StartPose = new Pose(-12, -33, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup2EndPose = new Pose(20, -33, Math.toRadians(0));
    private final Pose pickup3StartPose = new Pose(-12, -82, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(25, -82, Math.toRadians(0));
    private final Pose end = new Pose(-25,-10,Math.toRadians(34));

    private Path scorePreload;
    private PathChain grab1,grabend1,scorepre1,score1,grab2,grabend2,scorepre2,score2,grab3,grabend3,scorepre3,endpre,endfr;

//    public Command shoot() {
//        return new Command() {
//            @Override
//            public void start() {
//                kickTimer.reset();
//            }
//
//            @Override
//            public void update() {
//                time = kickTimer.milliseconds();
//                intake.setPower(-1);
//                left.setPosition(0.75);
//                right.setPosition(0.7);
//
//                if(time > 999) {
//                    kicker.setPosition(0.38);
//                }
//            }
//            @Override
//            public boolean isDone() {
//                if(time > 2000) {
//                 left.setPosition(0.65);
//                 right.setPosition(0.8);
//                 intake.setPower(0);
//                 kicker.setPosition(0);
//                 return true;
//                }
//                else {
//                    return false;
//                }
//            }
//        };
//    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabend1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .build();

        scorepre1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2StartPose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabend2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2StartPose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(), pickup2EndPose.getHeading())
                .build();

        scorepre2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, pickup2StartPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2StartPose, scorePose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePose.getHeading())
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3StartPose.getHeading())
                .build();

        grabend3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3StartPose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3StartPose.getHeading(), pickup3EndPose.getHeading())
                .build();

        scorepre3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, pickup3StartPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .build();

        endpre = follower.pathBuilder()
                .addPath(new BezierLine(pickup3StartPose, end))
                .setLinearHeadingInterpolation(pickup3StartPose.getHeading(), end.getHeading())
                .build();

        endfr = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, end))
                .setLinearHeadingInterpolation(scorePose.getHeading(), end.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(101);
                break;
            case 101: // Waiting to reach scoring position
                if (!follower.isBusy()) {
                    setPathState(102); // Start the shooting sequence
                }
                break;

            case 102: // The Shooting Sequence

                // Use pathTimer (which resets automatically in setPathState)
                if (pathTimer.getElapsedTime() > 1000) {
                    left.setPosition(0.75);
                    right.setPosition(0.7);
                    intake.setPower(-1);
                }

                if(pathTimer.getElapsedTime() > 1999) {
                    kicker.setPosition(0.38);
                }

                // After 2 seconds total, reset and move to next path
                if (pathTimer.getElapsedTime() > 3000) {
                    left.setPosition(0.65);
                    right.setPosition(0.8);
                    intake.setPower(0);
                    kicker.setPosition(0.015);
                    setPathState(1);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grab1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabend1,true);
                    setPathState(3);
                }
                break;
            case 3:
                // While moving, run intake
                if (follower.isBusy()) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0.0);
                    // Start following the path to the goal
                    follower.followPath(scorepre1, true);
                    setPathState(301); // Move to a "wait for score" state
                }
                break;

            case 301:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score1,true);
                    setPathState(31);
                }
                break;

            case 31: // Waiting to reach scoring position
                if (!follower.isBusy()) {
                    setPathState(32); // Start the shooting sequence
                }
                break;

            case 32: // The Shooting Sequence
                if (pathTimer.getElapsedTime() > 500) {
                    left.setPosition(0.75);
                    right.setPosition(0.7);
                    intake.setPower(-1);
                }

                if(pathTimer.getElapsedTime() > 1499) {
                    kicker.setPosition(0.38);
                }

                // After 2 seconds total, reset and move to next path
                if (pathTimer.getElapsedTime() > 2500) {
                    left.setPosition(0.65);
                    right.setPosition(0.8);
                    intake.setPower(0);
                    kicker.setPosition(0.015);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grab2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabend2,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (follower.isBusy()) {
                    intake.setPower(-1.0);
                } else {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intake.setPower(0.0);
                    follower.followPath(scorepre2,true);
                    setPathState(601);
                }
                break;
            case 601:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2,true);
                    setPathState(61);
                }
                break;
            case 61: // Waiting to reach scoring position
                if (!follower.isBusy()) {
                    setPathState(62); // Start the shooting sequence
                }
                break;

            case 62: // The Shooting Sequence
                if (pathTimer.getElapsedTime() > 500) {
                    left.setPosition(0.75);
                    right.setPosition(0.7);
                    intake.setPower(-1);
                }

                if(pathTimer.getElapsedTime() > 1499) {
                    kicker.setPosition(0.38);
                }

                // After 2 seconds total, reset and move to next path
                if (pathTimer.getElapsedTime() > 2500) {
                    left.setPosition(0.65);
                    right.setPosition(0.8);
                    intake.setPower(0);
                    kicker.setPosition(0.015);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grab3,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabend3,true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (follower.isBusy()) {
                    intake.setPower(-1.0);
                } else {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intake.setPower(0.0);
                    follower.followPath(scorepre3,true);
                    setPathState(901);
                }
                break;

            case 901:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(endpre,true);
                    setPathState(91);
                }
                break;
            case 91: // Waiting to reach scoring position
                if (!follower.isBusy()) {
                    setPathState(92); // Start the shooting sequence
                }
                break;

            case 92: // The Shooting Sequence
                if (pathTimer.getElapsedTime() > 500) {
                    left.setPosition(0.75);
                    right.setPosition(0.7);
                    intake.setPower(-1);
                }

                if(pathTimer.getElapsedTime() > 1499) {
                    kicker.setPosition(0.38);
                }

                // After 2 seconds total, reset and move to next path
                if (pathTimer.getElapsedTime() > 2500) {
                    left.setPosition(0.65);
                    right.setPosition(0.8);
                    intake.setPower(0);
                    kicker.setPosition(0.015);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        turret.updatebutauto();


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        intake = hardwareMap.get(DcMotor.class, "intake");
        left = hardwareMap.get(Servo.class,"left");
        right = hardwareMap.get(Servo.class,"right");
        kicker = hardwareMap.get(Servo.class,"kicker");
        turret = new TurretSubsystem(hardwareMap);

        left.setPosition(0.65);
        right.setPosition(0.8);
        kicker.setPosition(0.015);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        telemetry.update();

    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        kicktimer.resetTimer();
        setPathState(0);
    }
}