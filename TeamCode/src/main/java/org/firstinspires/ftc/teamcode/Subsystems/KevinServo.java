package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class KevinServo extends SubsystemBase {
    private Servo servo;

    public KevinServo(Servo servo) {
        this.servo = servo;
    }
    public void downPosition() {
        servo.setPosition(0.3);
    }
    public void midPosition() {
        servo.setPosition(0.5);
    }

    public void upPosition() {
        servo.setPosition(1.0);
    }
}
