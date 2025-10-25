package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;

@TeleOp(name = "Drive For Now")
public class MyTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
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