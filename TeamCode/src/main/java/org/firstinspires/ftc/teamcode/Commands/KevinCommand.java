package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Kevin_Stuff.Kevin_Subsystem.KevinSubsystem;

public class KevinCommand extends CommandBase {
    private KevinSubsystem subsystem;

    public KevinCommand(KevinSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void execute() {
        subsystem.runMotor();
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();
    }
    @Override
    public boolean isFinished() {
        return subsystem.hasObject();
    }
}
