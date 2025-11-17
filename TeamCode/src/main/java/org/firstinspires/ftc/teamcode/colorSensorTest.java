package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.LightBlinker;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Telemetry.TelemetryData;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryItem;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryManager;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp
public class colorSensorTest extends OpMode {
    ArtifactSensor sensor;

    @Override
    public void init() {
        sensor=new ArtifactSensor(hardwareMap);

    }

    @Override
    public void loop() {


        telemetry.addData("Artifact Color: ",sensor.read());

        TelemetryManager.getInstance().print(telemetry);

    }
}