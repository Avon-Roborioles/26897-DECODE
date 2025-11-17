package org.firstinspires.ftc.teamcode;

public enum ArtifactColor {
    GREEN("green"),
    PURPLE("purple"),
    NOTHING("nothing");
    private String color;
    ArtifactColor(String color){
        this.color=color;
    }

    public boolean equals(ArtifactColor other){
        if (other.ordinal()==this.ordinal()){
            return true;
        } else return false;
    }

    @Override
    public String toString(){
        return color;
    }
}