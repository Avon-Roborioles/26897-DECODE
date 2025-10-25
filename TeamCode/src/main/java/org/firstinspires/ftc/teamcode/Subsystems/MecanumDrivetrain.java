package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrivetrain {
    private final MecanumDrive drive;
    private final Motor frontLeft, backLeft, frontRight, backRight;
    public MecanumDrivetrain(HardwareMap hardwareMap) {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        backLeft = new Motor(hardwareMap, "backLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backRight = new Motor(hardwareMap, "backRight");

        frontLeft.setInverted(true);
        backRight.setInverted(true);

        drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    }
    public void drive(double strafe, double forward, double turn) {
        drive.driveRobotCentric(strafe, forward, turn);
    }

}