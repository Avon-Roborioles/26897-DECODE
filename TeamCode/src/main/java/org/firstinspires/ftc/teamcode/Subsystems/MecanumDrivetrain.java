package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrivetrain {
    private final MecanumDrive drive;
    private final Motor frontLeft, backLeft, frontRight, backRight;

    private static final boolean FRONT_LEFT_DEFAULT_INVERTED = true;

    public MecanumDrivetrain(HardwareMap hardwareMap) {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        backLeft = new Motor(hardwareMap, "backLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backRight = new Motor(hardwareMap, "backRight");

        frontLeft.setInverted(FRONT_LEFT_DEFAULT_INVERTED);
        backRight.setInverted(true);

        drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    }

    public void drive(double strafe, double forward, double turn) {
        final double TURN_THRESHOLD = 0.05;

        if (Math.abs(turn) > TURN_THRESHOLD) {
            frontLeft.setInverted(false);
        } else {
            frontLeft.setInverted(FRONT_LEFT_DEFAULT_INVERTED);
        }
        drive.driveRobotCentric(strafe, forward, -turn);
    }
}