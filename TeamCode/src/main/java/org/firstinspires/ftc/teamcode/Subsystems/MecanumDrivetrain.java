package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrivetrain {
    private final MecanumDrive drive;
    private final Motor fL, fR, bL, bR;
    private Follower follower;
    public double strafe =0;
    public double forward=0;
    public double turn = 0;
    public double requestedTurn = 0;

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

    public void udpateDriveInputs(){
        if (!rotate) {
            follower.setTeleOpDrive(forward, strafe, turn, true);
        } else {
            follower.setTeleOpDrive(forward,strafe,requestedTurn,true);
        }
    }
    public void turretRequestTurn(double turn){
        requestedTurn=turn;
    }
}