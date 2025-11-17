package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class limelightsubsystem extends SubsystemBase {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private LLResult result;
    private final int Pipeline = 4;

    public limelightsubsystem(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = limelight;
        limelight.pipelineSwitch(Pipeline);
        limelight.start();
        this.telemetry = telemetry;
    }

    public LLResult readAprilTag() {
        getResult();
        result = limelight.getLatestResult();

        return result;

    }
    public void start(){
        limelight.start();
    }

    public void getLimelightTelemetry() {
        readAprilTag();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("tags", result.getBotposeTagCount());
                telemetry.addData("LL Status", limelight.getStatus());

            }
        }
    }

    public boolean seeTag() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == 21) {
                return true;
            } else if (fr.getFiducialId() == 22) {
                return true;
            } else if (fr.getFiducialId() == 23) {
                return true;
            }
        }

        return false; // if none matched
    }


    public LLResult getResult(){
        result = limelight.getLatestResult();
        return limelight.getLatestResult();


    }
    public void setPipeline(int pipeline){
        limelight.stop();
        limelight.pipelineSwitch(pipeline);
        limelight.start();

    }
}