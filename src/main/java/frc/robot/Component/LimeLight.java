package frc.robot.Component;

import edu.wpi.first.networktables.*;
import frc.robot.Component.Data.LimeLightData;

/**
 * The limelight vision class
 */
public class LimeLight {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    /**
     * Get the current data from the limelight
     * @return
     */
    public LimeLightData GetCurrent()
    {        
        NetworkTableEntry tx = table.getEntry("tx"); //X Angle (degree)
        NetworkTableEntry ty = table.getEntry("ty"); //Y Angle (degree)
        NetworkTableEntry ta = table.getEntry("ta"); //Screen Space (Percent)
        NetworkTableEntry tv = table.getEntry("tv"); //Is Target (1 or 0)

        return new LimeLightData(tx.getDouble(0), ty.getDouble(0), ta.getDouble(0), tv.getDouble(0));
    }

    /**
     * Set the limelight in drive mode
     */
    public void SetDriveMode()
    {
        table.getEntry("camMode").setNumber(1);

        TurnOffLight();
    }
    /**
     * Set the limelight in recognition mode
     */
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

    /**
     * Make the light on the limelight bink
     */
    public void BlinkLight()
    {
        table.getEntry("ledMode").setNumber(2);
    }
    /**
     * Turn on the light on the limelight
     */
    public void TurnOnLight()
    {
        table.getEntry("ledMode").setNumber(3);
    }
    /**
     * Turn off the light on the limelight
     */
    public void TurnOffLight()
    {
        table.getEntry("ledMode").setNumber(1);
    }
    /**
     * Make the light be on the state specified by the current pipeline
     */
    public void SetLightDefault()
    {
        table.getEntry("ledMode").setNumber(0);
    }

    /**
     * Set the current pipeline to run on the limelight
     * @param id The pipeline to set on the limelight
     */
    public void SetPipeline(int id)
    {
        table.getEntry("pipeline").setNumber(id);
    }
    /**
     * Get the current pipeline running on the limelight
     * @return
     */
    public int GetCurrentPipeline()
    {
        return (int)table.getEntry("getpipe").getDouble(0);
    }
}
