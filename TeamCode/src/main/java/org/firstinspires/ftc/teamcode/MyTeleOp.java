//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//<<<<<<< Updated upstream
//import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
//
//=======
//import Subsystems.MecanumDrivetrain;
//>>>>>>> Stashed changes
//@TeleOp(name = "Drive For Now")
//public class MyTeleOp extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//
//        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
//        GamepadEx driverOp = new GamepadEx(gamepad1);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            driverOp.readButtons();
//            double x = driverOp.getLeftX();
//            double y = -driverOp.getLeftY();
//            double turn = driverOp.getRightX();
//            drivetrain.drive(x, y, turn);
//
//            telemetry.addData("Drive", "%.2f, %.2f, %.2f", x, y, turn);
//            telemetry.update();
//
//            idle();
//        }
//        drivetrain.drive(0, 0, 0);
//    }
//}