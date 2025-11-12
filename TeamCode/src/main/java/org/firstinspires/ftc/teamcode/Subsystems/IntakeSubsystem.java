package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor = null;



    public IntakeSubsystem(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }
    public void runMotor() {
        intakeMotor.setPower(1.0);
    }


    public void stopMotor() {
        intakeMotor.setPower(0.0);
    }
}
