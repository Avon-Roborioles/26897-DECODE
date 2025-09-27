package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
public class MecanumDrive {
    private MecanumDrive drive = null;
    private final Motor frontLeft, backLeft, frontRight, backRight;
    public MecanumDrive(Motor frontLeft, Motor backLeft, Motor frontRight, Motor backRight) {
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    }
    public void drive(double strafe, double forward, double turn) {
        drive.drive(strafe, forward, turn);
    }
}