package frc.robot.Component.Data;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Robot;
import frc.robot.Math.Timer;

public class RobotOdometry 
{
    public RobotOdometry(double WheelRadius, double LowGearRatio, double HighGearRatio)
    {
        _lowSpeedGain = (double)Robot.SwerveDrive.GetWheelCount() / (WheelRadius * LowGearRatio);
        _highSpeedGain = (double)Robot.SwerveDrive.GetWheelCount() / (WheelRadius * HighGearRatio);

        _position = new Vector2d(0, 0);
    }

    private double _lowSpeedGain;
    private double _highSpeedGain;

    private Vector2d _position;

    public void SetPosition(Vector2d Pos)
    {
        _position = Pos;
    }  
    public Vector2d GetPosition()
    {
        return _position;
    }

    public void DoOdometry()
    {
        int count = Robot.SwerveDrive.GetWheelCount();

        Vector2d sumVec = new Vector2d(0, 0);

        for(int i = 0; i < count; i++)
        {
            //Get the vector of the wheel
            Vector2d vec = Robot.SwerveDrive.GetWheelVector(i);

            sumVec.x += vec.x;
            sumVec.y += vec.y;
        }

        //True = High gear
        //False = Low gear
        boolean isHigh = Robot.SwerveDrive.GetGear();

        //Make average of all the vector and convert it to distance
        sumVec.x /= (isHigh ? _highSpeedGain : _lowSpeedGain) / Timer.GetDeltaTime();
        sumVec.y /= (isHigh ? _highSpeedGain : _lowSpeedGain) / Timer.GetDeltaTime();

        _position.x += sumVec.x;
        _position.y += sumVec.y;
    }
}
