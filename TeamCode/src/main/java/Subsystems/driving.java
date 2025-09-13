package Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

public class driving extends LinearOpMode {
    static final boolean FIELD_CENTRIC = false;


    @Override
    public void runOpMode() throws InterruptedException {
        Motor frontLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_312);
        // init mecanum drive
        MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
        GamepadEx driverOp = new GamepadEx(gamepad1);

        // defines reference variables
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection;
        logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // instantiates and initializes IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //inverts the motors
        backRight.setInverted(true);
        frontLeft.setInverted(true);

        waitForStart();

        while(!isStopRequested()) {
            drive.driveRobotCentric(
                    driverOp.getLeftY(),
                    driverOp.getLeftX(),
                    driverOp.getRightX()
            );
        }
    }
}
