package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldDrivetrain {
    private final MecanumDrive drive;
    private final Motor fL, fR, bL, bR;
    private final IMU imu;

    public FieldDrivetrain(HardwareMap hardwareMap) {
        // 1. Initialize Motors
        fL = new Motor(hardwareMap, "frontLeft");
        fR = new Motor(hardwareMap, "frontRight");
        bL = new Motor(hardwareMap, "backLeft");
        bR = new Motor(hardwareMap, "backRight");

        // Keep your existing inversion logic
        fL.setInverted(true);
        bL.setInverted(true);
        bR.setInverted(true);
        fR.setInverted(true);

        drive = new MecanumDrive(fL, fR, bL, bR);

        // 2. Initialize the IMU (Gyro)
        // Check your Hub's mounting position!
        // If your Hub is flat with the logo up and USB forward, use UP and FORWARD.
        // If mounted differently, change these directions below.
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
    }

    /**
     * @param strafe  The x speed (horizontal)
     * @param forward The y speed (vertical)
     * @param turn    The rotation speed
     */
    public void drive(double strafe, double forward, double turn) {
        // 3. Get the robot's heading in degrees
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // 4. Pass the heading into FTCLib's driveFieldCentric method
        drive.driveFieldCentric(strafe, forward, turn, heading);
    }

    /**
     * Call this to reset "Forward" to the robot's current facing.
     */
    public void resetHeading() {
        imu.resetYaw();
    }
}