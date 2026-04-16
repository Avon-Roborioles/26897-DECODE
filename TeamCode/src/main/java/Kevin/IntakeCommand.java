package Kevin;

import com.arcrobotics.ftclib.command.CommandBase;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public IntakeCommand (IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }


}
