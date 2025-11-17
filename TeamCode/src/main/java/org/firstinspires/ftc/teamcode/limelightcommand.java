package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.limelightsubsystem;

public class limelightcommand extends CommandBase {
    private limelightsubsystem limelightSubsystem;
    private LLResult lastResult;

    public limelightcommand(limelightsubsystem limelightSubsystem, LLResult lastResult){
        this.limelightSubsystem = limelightSubsystem;
        this.lastResult = lastResult;
        addRequirements(limelightSubsystem);
    }
    @Override
    public void execute(){

        limelightSubsystem.getLimelightTelemetry();
        limelightSubsystem.setPipeline(4);
    }

    public void end(boolean interrupted){
        limelightSubsystem.setPipeline(1);
    }
    public boolean isFinished(){
        return limelightSubsystem.seeTag();
    }

}