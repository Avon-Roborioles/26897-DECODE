package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class IntakeStopCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public IntakeStopCommand (IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }
    @Override
    public void execute() {
        intakeSubsystem.stopMotor();
    }
}
