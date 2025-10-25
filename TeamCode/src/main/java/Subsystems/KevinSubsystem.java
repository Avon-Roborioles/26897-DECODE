package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class KevinSubsystem extends SubsystemBase {
    private DcMotor motor = null;
    private Rev2mDistanceSensor distance;
    public KevinSubsystem(DcMotor motor, Rev2mDistanceSensor distance) {
        this.motor = motor;
        this.distance = distance;
    }

    public void runMotor() {
        motor.setPower(0.5);
    }
    public void stopMotor() {
        motor.setPower(0);
    }
    public boolean hasObject() {
        return (distance.getDistance(DistanceUnit.INCH) <= 3);
    }
}
