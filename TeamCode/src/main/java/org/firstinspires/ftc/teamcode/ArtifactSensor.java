package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryData;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryItem;

public class ArtifactSensor {

    ColorSensor sensor;
    //DistanceSensor dSensor;

    double sum;
    double nRed,nGreen, nBlue;
    double normalDifference;
    double purpleDifference;
    double greenDifference;
    public ArtifactSensor(HardwareMap hMap){
        sensor=hMap.get(ColorSensor.class,"colorSensor");
        //dSensor = hMap.get(DistanceSensor.class,"distanceSensor");

        new TelemetryData("Red",()->1.*nRed);
        new TelemetryData("Green",()->1.*nGreen);
        new TelemetryData("Blue",()->1.*nBlue);
        new TelemetryItem(()->"Color"+this.read().toString());
        //new TelemetryData("Inches Away",()->dSensor.getDistance(DistanceUnit.INCH));
        new TelemetryData("Normal Difference",()->normalDifference);
        new TelemetryData("Purple Difference",()->purpleDifference);
        new TelemetryData("Green Difference",()->greenDifference);
    }


    private double vectorDifference(double[] v1,double[]v2){
        double sum=0;
        for (int i =0;i<3;i++){
            sum+= (v1[i]-v2[i])*(v1[i]-v2[i]);
        }

        return sum;
    }

    public ArtifactColor read() {
        ArtifactColor color;
        double[] normal = {.244,.424,.346};
        double[] purpleTemplate = {.232,.344,.427};
        double[] greenTemplate = {.14,.49,.37};



        double red = sensor.red();
        double green = sensor.green();
        double blue = sensor.blue();
        sum = red + green + blue;
        nRed = red / sum;
        nGreen = green / sum;
        nBlue = blue / sum;
        double[] colors = new double[]{nRed,nGreen,nBlue};

        normalDifference = vectorDifference(normal,colors);
        purpleDifference = vectorDifference(colors,purpleTemplate);
        greenDifference = vectorDifference(colors, greenTemplate);

        if (normalDifference<purpleDifference&&normalDifference<greenDifference){
            color = ArtifactColor.NOTHING;
        } else if (purpleDifference<normalDifference&&purpleDifference<greenDifference){
            color = ArtifactColor.PURPLE;
        } else if (greenDifference<purpleDifference&&greenDifference<normalDifference) {
            color = ArtifactColor.GREEN;
        }  else {
            color = ArtifactColor.NOTHING;
        }
//        if (nGreen > .46) {
//            color = ArtifactColor.GREEN;
//        } else if (nBlue + nRed > .58) {
//            color = ArtifactColor.GREEN;
//        } else {
//            color = ArtifactColor.NOTHING;
//        }
//        if (dSensor.getDistance(DistanceUnit.INCH)>4.+.8){
//            color=ArtifactColor.NOTHING;
//        }
        return  color;
    }
}