package frc.robot.Component;

import edu.wpi.first.networktables.*;
import frc.robot.Component.Data.LimeLightData;

public class LimeLight {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public LimeLightData GetCurrent()
    {        
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

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
