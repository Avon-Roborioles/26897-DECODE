package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@TeleOp
public class IntakeOpMode extends CommandOpMode {
    private IntakeSubsystem intakeSubsystem;
    private GamepadEx driver, operator;
@Override
    public void initialize() {
    intakeSubsystem = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "intakemotor"));
    driver = new GamepadEx(gamepad1);
    operator = new GamepadEx(gamepad2);
    operator.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new IntakeCommand(intakeSubsystem));
    }
}
