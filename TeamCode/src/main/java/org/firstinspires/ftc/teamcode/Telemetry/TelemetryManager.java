package org.firstinspires.ftc.teamcode.Telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TelemetryManager {

    private static TelemetryManager telemetryManager;
    private ArrayList<TelemetryItem> telemetryItems;
    private ArrayList<String> tagsToExclude;
    private ArrayList<String> tagsToInclude;

    public synchronized static TelemetryManager getInstance(){
        if (TelemetryManager.telemetryManager==null){
            TelemetryManager.telemetryManager=new TelemetryManager();
        }
        return telemetryManager;
    }
    private TelemetryManager(){
        telemetryItems=new ArrayList<TelemetryItem>();
        tagsToExclude=new ArrayList<String>();
    }
    public void addTelemetryItem(TelemetryItem item){
        if (!telemetryItems.contains(item)){
            telemetryItems.add(item);
        }
    }
    public void removeTelemetryItem(TelemetryItem item){
        if (telemetryItems.contains(item)){
            telemetryItems.remove(item);
        }
    }
    public void addExcludedTag(String tag){
        if (!tagsToExclude.contains(tag)) {
            tagsToExclude.add(tag);
        }
    }
    public void removeExcludedTag(String tag){
        if (tagsToExclude.contains(tag)) {
            tagsToExclude.remove(tag);
        }
    }
    public void addIncludedTag(String tag){
        if (!tagsToInclude.contains(tag)) {
            tagsToInclude.add(tag);
        }
    }
    public void removeIncludedTag(String tag){
        if (tagsToInclude.contains(tag)) {
            tagsToInclude.remove(tag);
        }
    }

    public void print(Telemetry telemetry){
        for (int i=0;i<telemetryItems.size();i++){
            TelemetryItem item = telemetryItems.get(i);
            boolean isValid =true;
            //exclude
            for (int j=0;j<item.getTags().size();i++){
                if (tagsToExclude.contains(item.getTags().get(j))==false){
                    isValid=false;
                }
            }
            //include
            for (int j=0;j<item.getTags().size();i++){
                if (tagsToInclude.contains(item.getTags().get(j))==true){
                    isValid=true;
                }
            }
            if (isValid) {
                telemetry.addLine(telemetryItems.get(i).getItem());
            }

        }
        telemetry.update();
    }
    public void reset(){
        TelemetryManager.telemetryManager=null;
    }
}