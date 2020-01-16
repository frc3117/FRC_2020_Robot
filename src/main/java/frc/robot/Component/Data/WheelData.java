/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Component.Data;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class WheelData
{
    public WheelData() {}

    public WheelData(int driveChannel, int directionChannel, int encoderChannel, int shifterChannel, Vector2d wheelPosition, double angleOffset)
    {
        DriveChannel = driveChannel;
        DirectionChannel = directionChannel;
        EncoderChannel = encoderChannel;
        ShifterChannel = shifterChannel;
        WheelPosition = wheelPosition;
        AngleOffset = angleOffset;
    }

    public int DriveChannel;
    public int DirectionChannel;
    public int EncoderChannel;
    public int ShifterChannel;

    public Vector2d WheelPosition;

    public double AngleOffset;

    public Vector2d GetWheelRotationVector()
    {
        //Rotation vector is a normalized normal vector from the wheel position
        Vector2d vec = new Vector2d(-WheelPosition.y, WheelPosition.x);
        double mag = vec.magnitude();

        vec.x /= mag;
        vec.y /= mag;

        return vec;
    }
}