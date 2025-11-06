package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor; // NEW IMPORT for direct motor control

// --- PEDROPATHING IMPORTS (Confirmed by your example code) ---
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.util.Timer;
// Custom Constants file for initialization
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
// -----------------------------------------------------------

/**
 * This OpMode performs the movement sequence (forward 2, left 3)
 * and executes a rapid strafing "tweak out" motion.
 * * FIX: This version uses standard FTC DcMotor objects for manual control,
 * completely bypassing all problematic Follower motor access methods.
 */
@Autonomous(name = "Pedro Somewhat then tweak tf out", group = "Pedro")
public class pedrofirsttest extends OpMode {

    // --- VARIABLES ---
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState; // Tracks the robot's progression

    // CRITICAL: Variables for direct motor control
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Poses (Defined as final constants for clarity)
    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose target1 = new Pose(0, 2, 0);       // 2 inches Forward
    private final Pose target2 = new Pose(-3, 2, 0);      // 3 inches Left

    // Paths
    private PathChain moveChain;

    // Tweak Out Constants
    private final double TWEAK_POWER = 0.6;
    private final long TWEAK_DURATION_MS = 3000;
    private final long TWEAK_INTERVAL_MS = 50;

    // --- SETUP METHODS (Called in Init) ---

    public void buildPaths() {
        // Path 1: Forward 2 inches
        Path moveForward = new Path(new BezierLine(startPose, target1));
        moveForward.setLinearHeadingInterpolation(startPose.getHeading(), target1.getHeading());

        // Path 2: Strafe Left 3 inches
        Path strafeLeft = new Path(new BezierLine(target1, target2));
        strafeLeft.setLinearHeadingInterpolation(target1.getHeading(), target2.getHeading());

        // Combine paths into a chain for sequential execution using the pathBuilder() pattern
        moveChain = follower.pathBuilder()
                .addPath(moveForward)
                .addPath(strafeLeft)
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) {
            pathTimer.resetTimer();
        }
    }

    // --- MAIN FSM (Called in Loop) ---

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // START: Begin the movement chain
                telemetry.addLine("State 0: Starting Movement Chain");
                follower.followPath(moveChain);
                setPathState(1);
                break;

            case 1: // FOLLOW PATH: Wait until the path chain is complete
                telemetry.addLine("State 1: Following Path Chain...");
                if (!follower.isBusy()) {
                    // Path is complete, transition to Tweak Out
                    setPathState(2);
                }
                break;

            case 2: // TWEAK OUT START: Initialize the Tweak Out timer
                telemetry.addLine("State 2: Initializing Tweak Out");
                opmodeTimer.resetTimer(); // Start the 3-second timer
                setPathState(3);
                break;

            case 3: // TWEAK OUT EXECUTION: Rapid Strafe motion
                if (opmodeTimer.getElapsedTime() < TWEAK_DURATION_MS) {
                    long currentTime = opmodeTimer.getElapsedTime();

                    // Define motor powers for strafing.
                    double fl, fr, bl, br;

                    if ((currentTime / TWEAK_INTERVAL_MS) % 2 == 0) {
                        // Strafe Right: (FrontLeft, FrontRight, BackLeft, BackRight) -> (P, P, -P, -P)
                        fl = TWEAK_POWER; fr = TWEAK_POWER; bl = -TWEAK_POWER; br = -TWEAK_POWER;
                    } else {
                        // Strafe Left: (FrontLeft, FrontRight, BackLeft, BackRight) -> (-P, -P, P, P)
                        fl = -TWEAK_POWER; fr = -TWEAK_POWER; bl = TWEAK_POWER; br = TWEAK_POWER;
                    }

                    // CRITICAL: Use direct motor control
                    frontLeft.setPower(fl);
                    frontRight.setPower(fr);
                    backLeft.setPower(bl);
                    backRight.setPower(br);

                } else {
                    // Tweak Out time is up, stop motors and finish
                    // CRITICAL: Stop motors directly
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    setPathState(4); // Finished state
                }
                break;

            case 4: // FINISHED: Autonomous complete
                telemetry.addLine("State 4: Autonomous Complete! ✅");
                break;
        }
    }

    // --- OPMODE LIFECYCLE METHODS ---

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        try {
            telemetry.addData("Status", "Initializing Follower...");
            // Initialize the Follower for path following
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Follower failed to initialize. Check your Constants setup!");
            telemetry.addData("Error Detail", e.getMessage());
            telemetry.update();
            return;
        }

        // --- CRITICAL: Initialize Motors for direct Tweak Out control ---
        // !!! YOU MUST CHANGE THE STRING NAMES BELOW TO MATCH YOUR ROBOT'S CONFIGURATION !!!
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");
        // --------------------------------------------------------

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialization Complete. Ready to Start.");
        telemetry.addData("Start Pose", follower.getPose().toString());
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // CRITICAL: Use direct motor control to ensure they stop
        if (frontLeft != null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    @Override
    public void init_loop() {}
}