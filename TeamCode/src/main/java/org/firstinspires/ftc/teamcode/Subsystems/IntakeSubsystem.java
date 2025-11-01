package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor = null;
    private CRServo intakeServo1;
    private CRServo intakeServo2;


    public IntakeSubsystem(DcMotor motor, CRServo intakeServo1, CRServo intakeServo2) {
        this.intakeMotor = intakeMotor;
        this.intakeServo1 = intakeServo1;
        this.intakeServo2 = intakeServo2;
    }
    public void runMotor() {
        intakeMotor.setPower(1.0);
    }

    public void runIntake1() {
        intakeServo1.setPower(0.8);
    }

    public void runIntake2() {
        intakeServo2.setPower(0.8);
    }

    public void stopIntakeServo1() {
        intakeServo1.setPower(0);
    }

    public void stopIntakeServo2() {
        intakeServo2.setPower(0);
    }


    public void stopMotor() {
        intakeMotor.setPower(0.0);
    }
}
