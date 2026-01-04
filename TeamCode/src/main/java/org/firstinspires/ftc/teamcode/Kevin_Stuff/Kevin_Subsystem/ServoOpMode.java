package org.firstinspires.ftc.teamcode.Kevin_Stuff.Kevin_Subsystem;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.ServoCommand;
import org.firstinspires.ftc.teamcode.Commands.ServoCommand2;

public class ServoOpMode extends CommandOpMode {
    private KevinServo kevinServo;
    private GamepadEx driver, operator;
    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        kevinServo = new KevinServo(hardwareMap.get(Servo.class, "down servo"));
        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new ServoCommand(kevinServo))
                .whenReleased(new ServoCommand2(kevinServo));
    }
}
