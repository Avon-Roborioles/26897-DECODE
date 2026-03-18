//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import dev.*;
//import dev.nextftc.ftc.NextFTCOpMode;
//
//import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
//
//@TeleOp(name = "Odo Turret teleop")
//public class newtestteleop extends NextFTCOpMode {
//
//    // Declare the Subsystem and the Follower
//    private final TurretSubsystem turret = new TurretSubsystem();
//    private DcMotorEx shooter1, shooter2;
//    private DcMotor intake;
//    private CRServo pass, underpass;
//    private Limelight3A limelight;
//    private Follower follower;
//
//    // Field coordinates for goals
//    private final Pose BLUE_GOAL = new Pose(-5, 172.5);
//    private final Pose RED_GOAL = new Pose(139, 172.5);
//    private boolean trackingRed = true;
//
//    double speed = 0;
//    final double GOAL_HEIGHT = 29.867;
//
//    @Override
//    public void onInit() {
//        // Initialize Pedro Pathing
//        follower = Constants.createFollower(hardwareMap);
//
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        pass = hardwareMap.get(CRServo.class,"pass");
//        underpass = hardwareMap.get(CRServo.class,"underpass");
//        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
//        shooter2 = hardwareMap.get(DcMotorEx.class,"shooter2");
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//        limelight.start();
//        shooter1.setVelocityPIDFCoefficients(50, 0, 17.8, 20.21);
//        shooter2.setVelocityPIDFCoefficients(50, 0, 17.8, 20.21);
//
//        // Starting Position: Facing 90 degrees (Forward)
//        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));
//        follower.startTeleopDrive();
//
//        // Register the subsystem so NextFTC knows to run its onOutput()
//        // In NextFTC, adding the subsystem to the OpMode handles the lifecycle
//        turret.initialize();
//    }
//
//    @Override
//    public void onUpdate() {
//        // 1. Update Pedro Pathing
//        follower.update();
//
//        // 2. Target Calculation
//        Pose currentPose = follower.getPose();
//        Pose targetGoal = trackingRed ? RED_GOAL : BLUE_GOAL;
//
//        double dx = targetGoal.getX() - currentPose.getX();
//        double dy = targetGoal.getY() - currentPose.getY();
//
//        // Absolute field angle to the goal
//        double angleToGoalRad = Math.atan2(dy, dx);
//
//        // Relative angle = (Goal Angle) - (Robot Heading)
//        double relativeTargetDeg = Math.toDegrees(angleToGoalRad - currentPose.getHeading());
//
//        // 3. Update the Turret Subsystem variables
//        turret.targetAngle = relativeTargetDeg;
//        turret.isTracking = true;
//
//        // 2. Manually run the turret's periodic loop!
//        turret.periodic();
//
//        // 4. Drive Control
//        // (Forward, Strafe, Turn, FieldCentric)
//        follower.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                true
//        );
//
//        // 5. Goal Toggle Logic
//        // In NextFTC, we can use the declarative style for buttons
//        if (gamepad1.yWasPressed()) {
//            trackingRed = !trackingRed;
//        }
//
//        if (gamepad1.right_trigger > 0.8) {
//            pass.setPower(-1);
//        } else {
//            pass.setPower(0);
//        }
//
//        intake.setPower(-gamepad1.left_trigger);
//        underpass.setPower(-gamepad1.left_trigger);
//        //intake.setPower(-gamepad1.right_trigger);
//
//        // Manual Shooter Control
//        if (gamepad1.right_bumper) speed += 25;
//        if (gamepad1.left_bumper) speed -= 25;
//
//        double currentdistance = getDistance();
//        //speed = 6.6533 * currentdistance + 981.48;
//        shooter1.setVelocity(-speed);
//        shooter2.setVelocity(speed);
//
//        // 6. Telemetry
//        telemetry.addData("Tracking", trackingRed ? "RED GOAL" : "BLUE GOAL");
//        telemetry.addData("Turret Target", "%.2f deg", turret.targetAngle);
//        telemetry.addData("Robot Pose", currentPose.toString());
//        telemetry.update();
//    }
//
//    public double getDistance() {
//        LLResult result = limelight.getLatestResult();
//        if (result == null || !result.isValid()) return -1;
//        double targetOffsetAngle_Vertical = result.getTy();
//        double limelightMountAngleDegrees = 8;
//        double limelightLensHeightInches = 14.9;
//        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);
//        if (angleToGoalRadians == 0) return -1;
//        return (GOAL_HEIGHT - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
//
//    }
//}