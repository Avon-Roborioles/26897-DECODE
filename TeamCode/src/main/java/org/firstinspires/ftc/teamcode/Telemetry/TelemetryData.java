package org.firstinspires.ftc.teamcode.Telemetry;

import java.util.function.Supplier;

public class TelemetryData extends TelemetryItem{
    String caption;
    Supplier<Double> data;

    public TelemetryData(String caption, Supplier<Double> data){
        super(()->caption+data.get());
        this.caption = caption;
        this.data=data;
    }

    @Override
    public String getItem(){
        return caption+": "+data.get();
    }

}