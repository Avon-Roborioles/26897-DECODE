package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public IntakeCommand (IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        intakeSubsystem.runMotor();
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotor();
    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
