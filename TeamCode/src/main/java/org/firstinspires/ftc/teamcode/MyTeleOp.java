package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive For Now")
public class MyTeleOp extends LinearOpMode {
    private Motor frontLeft;
    private Motor backLeft;
    private Motor frontRight;
    private Motor backRight;

    @Override
    public void runOpMode() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        MecanumDrive drivetrain = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
        GamepadEx driverOp = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.drive(
                    driverOp.getLeftX(),
                    -driverOp.getLeftY(),
                    driverOp.getRightX()
            );
        }
    }
}