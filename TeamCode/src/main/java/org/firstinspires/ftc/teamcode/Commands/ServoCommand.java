package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.KevinServo;

public class ServoCommand extends CommandBase {
    private KevinServo kevinServo;

    public ServoCommand(KevinServo kevinServo) {
        this.kevinServo = kevinServo;
        addRequirements(kevinServo);
    }
    @Override
    public void execute() {
        kevinServo.downPosition();
    }
    public boolean isFinished() {
        return true;
    }
}
