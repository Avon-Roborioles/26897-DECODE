package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;
    private final CRServo intakeServo;
    private final CRServo intakeServo1;
    private final CRServo intakeServo2;

    public static final int INTAKE_POSITION = 490;
    public static final int RETRACT_POSITION = 0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakemotor");
        intakeServo = hardwareMap.get(CRServo.class, "intakeservo");
        intakeServo1 = hardwareMap.get(CRServo.class, "intakeservo1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intakeservo2");

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake() {
        intakeMotor.setTargetPosition(INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(0.5);
    }

    public void retract() {
        intakeMotor.setTargetPosition(RETRACT_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(0.5);
    }

    public void runIntakeServos() {
        intakeServo.setPower(1);
        intakeServo1.setPower(-1);
        intakeServo2.setPower(-1);
    }

    public void stopIntakeServos() {
        intakeServo.setPower(0);
        intakeServo1.setPower(0);
        intakeServo2.setPower(0);
    }

    public boolean isBusy() {
        return intakeMotor.isBusy();
    }
}
