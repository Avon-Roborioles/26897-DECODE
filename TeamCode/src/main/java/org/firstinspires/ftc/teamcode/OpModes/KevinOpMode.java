package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.KevinCommand;
import org.firstinspires.ftc.teamcode.Subsystems.KevinSubsystem;

@TeleOp
public class KevinOpMode extends CommandOpMode {
    private KevinSubsystem kevinSubsystem;
    private GamepadEx driver, operator;
    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        kevinSubsystem = new KevinSubsystem(hardwareMap.get(DcMotor.class, "motor"), hardwareMap.get(Rev2mDistanceSensor.class, "distance sensor"));
        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new KevinCommand(kevinSubsystem));
    }
}
