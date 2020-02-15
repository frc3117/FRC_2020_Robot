package frc.robot.Component.Data;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Robot;
import frc.robot.Component.LimeLight;
import frc.robot.Math.Polar;

/**
 * A position estimator based on the limelight
 */
public class LimeLightPosition 
{
    public LimeLightPosition(Vector2d InitPos)
    {
        _currentPos = InitPos;
    }

    private Vector2d _currentPos;
    private LimeLight _limeLight = new LimeLight();

    public boolean IsTarget()
    {
        return _limeLight.GetCurrent().IsTarget();
    }

    public Vector2d Evaluate()
    {
        LimeLightData data = _limeLight.GetCurrent();

        double Theta = Robot.SwerveDrive.GetHeading();
        double Distance = Robot.Thrower.GetDistance();

        Vector2d newPos = new Polar(Distance, Theta - data.GetAngleX()).vector();

        return newPos;
    }
    public Vector2d GetCurrent()
    {
        return _currentPos;
    }
}
