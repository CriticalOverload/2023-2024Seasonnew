package org.firstinspires.ftc.teamcode;

public class PositionData {
    double x,y;
    double heading;
    //update from ticks from encoders
    public PositionData(){
        x=0;
        y=0;
        heading=0;
    }
    //todo make constructors for different field starting positions
    //todo figure out how to use ticks from each to determine update location data
    public double getX(){ return x;}
    public double getY(){ return y;}
    public double getHeading(){ return heading;}
}
