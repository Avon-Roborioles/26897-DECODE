package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrivetrain {
    private final MecanumDrive drive;
    private final Motor fL, fR, bL, bR;
    private Follower follower;
    public double strafe =0;
    public double forward=0;
    public double turn = 0;
    public double requestedTurn = 0;
    public boolean move = false;

    public boolean rotate = false;
    public MecanumDrivetrain(HardwareMap hardwareMap) {
        fL = new Motor(hardwareMap, "frontLeft");
        fR = new Motor(hardwareMap, "frontRight");
        bL = new Motor(hardwareMap, "backLeft");
        bR = new Motor(hardwareMap, "backRight");

        // Usually, the left side needs to be inverted for forward to be positive
        fL.setInverted(true);
        bL.setInverted(true);
        bR.setInverted(true);
        fR.setInverted(true);

        drive = new MecanumDrive(fL, fR, bL, bR);
    }
    public MecanumDrivetrain(Follower follower,HardwareMap hardwareMap){
        fL = new Motor(hardwareMap, "frontLeft");
        fR = new Motor(hardwareMap, "frontRight");
        bL = new Motor(hardwareMap, "backLeft");
        bR = new Motor(hardwareMap, "backRight");

        // Usually, the left side needs to be inverted for forward to be positive
        fL.setInverted(true);
        bL.setInverted(true);
        bR.setInverted(true);
        fR.setInverted(true);

        drive = new MecanumDrive(fL, fR, bL, bR);
        this.follower=follower;

    }
    /**
     * @param strafe  The x speed (horizontal)
     * @param forward The y speed (vertical)
     * @param turn    The rotation speed
     */
    public void drive(double strafe, double forward, double turn) {
        // FTCLib's driveRobotCentric handles the normalization for you
        drive.driveRobotCentric(strafe, forward, turn);
    }
    public void setDriveInputs(double strafe, double forward, double turn){
        this.strafe=strafe;
        this.forward=forward;
        this.turn=turn;
    }

    public void robotTurn() {
        if(!rotate) {
            rotate = true;
        } else {
            rotate = false;
        }
    }

    public void updateDriveInputs() {
        // Start with the default joystick inputs
        double currentForward = forward;
        double currentStrafe = strafe;
        double currentTurn = turn;

        // 1. Apply slow mode if 'move' (right trigger) is active
        if (move) {
            currentForward = Range.clip(forward, -0.5, 0.5);
            currentStrafe = Range.clip(strafe, -0.5, 0.5);
        }

        // 2. Override the turn value if turret auto-rotation is active
        if (rotate) {
            currentTurn = requestedTurn;
        }

        // 3. Send the final calculated values to PedroPathing ONCE
        follower.setTeleOpDrive(currentForward, currentStrafe, currentTurn, true);
    }

    public void setDriveInputsHalfPower(double strafe, double forward, double turn) {
        this.strafe = strafe * 0.5;
        this.forward = forward * 0.5;
        this.turn = turn * 0.5;
    }

    public void turretRequestTurn(double turn){
        requestedTurn=turn;
    }
}