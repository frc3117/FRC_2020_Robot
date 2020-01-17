package frc.robot.Component;

import edu.wpi.first.networktables.*;
import frc.robot.Component.Data.LimeLightData;

public class LimeLight {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public LimeLightData GetCurrent()
    {        
        NetworkTableEntry tx = table.getEntry("tx"); //X Angle (degree)
        NetworkTableEntry ty = table.getEntry("ty"); //Y Angle (degree)
        NetworkTableEntry ta = table.getEntry("ta"); //Screen Space (Percent)
        NetworkTableEntry tv = table.getEntry("tv"); //Is Target (1 or 0)

        return new LimeLightData(tx.getDouble(0), ty.getDouble(0), ta.getDouble(0), tv.getDouble(0));
    }

    public void SetDriveMode()
    {
        table.getEntry("camMode").setNumber(1);

        TurnOffLight();
    }
    public void SetRecognitionMode()
    {
        table.getEntry("camMode").setNumber(0);

        TurnOnLight();
    }

    public void Zoom()
    {
        //haven't found how yet
    }
    public void UnZoom()
    {
        //haven't found how yet
    }

    // LedMode
    //
    // (0) State From Current Pipeline
    // (1) Turn Off
    // (2) Force Blink
    // (3) Turn On

    public void TurnOnLight()
    {
        table.getEntry("ledMode").setNumber(3);
    }
    public void TurnOffLight()
    {
        table.getEntry("ledMode").setNumber(1);
    }

    public void SetPipeline(int id)
    {
        table.getEntry("pipeline").setNumber(id);
    }
}
