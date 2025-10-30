package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.KevinServo;

public class ServoCommand2 extends CommandBase {
    private KevinServo kevinServo;

    public ServoCommand2 (KevinServo kevinServo) {
        this.kevinServo = kevinServo;
    }
    @Override
    public void execute() {
        kevinServo.upPosition();
    }
    public boolean isFinished() {
        return true;
    }
}
