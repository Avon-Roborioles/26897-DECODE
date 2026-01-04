package org.firstinspires.ftc.teamcode.Kevin_Stuff.Kevin_Subsystem;

import static android.graphics.Color.blue;

import static androidx.core.graphics.ColorKt.getGreen;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class KevinSensor extends SubsystemBase {
    private RevColorSensorV3 colorSensor;
    private Rev2mDistanceSensor distanceSensor;
    public KevinSensor (RevColorSensorV3 colorSensor, Rev2mDistanceSensor distanceSensor) {
        this.colorSensor = colorSensor;
        this.distanceSensor = distanceSensor;
    }

    public double getGreen(){
        return colorSensor.green();
    }

    public double getBlue(){
        return colorSensor.blue();
    }

    public double getRed(){
        return colorSensor.red();
    }

    public boolean hasGreen(){
        return getGreen() > 300;
    }

    public boolean hasPurple() {
        return (getRed()> getGreen()) && (getBlue() > getGreen());
    }
    //Distance sensor is the same, just replace colorSensor with distance sensor name
    public double getDistance(){
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean hasArtifact() {
        return getDistance() < 3.0;
    }


}
