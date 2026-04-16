package Kevin;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor = null;

    public IntakeSubsystem (DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public void runMotor() {

        intakeMotor.setPower(1);
    }
    public void stopMotor() {

        intakeMotor.setPower(0);
    }
}
