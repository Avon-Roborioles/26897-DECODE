package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class Intaketestcommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final boolean extend;
    public Intaketestcommand(IntakeSubsystem subsystem, boolean extend) {
        this.intakeSubsystem = subsystem;
        this.extend = extend;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        if (extend) {
            intakeSubsystem.intake();
        } else {
            intakeSubsystem.retract();
        }
    }
    @Override
    public boolean isFinished() {
        return !intakeSubsystem.isBusy();
    }
}

