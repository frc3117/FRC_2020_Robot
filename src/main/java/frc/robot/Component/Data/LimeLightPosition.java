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

    /**
     * Check if there is currently a target
     * @return If there is currently a target
     */
    public boolean IsTarget()
    {
        return LimeLight.GetCurrent().IsTarget();
    }

    /**
     * Evaluate the current position based on the limelight
     * @return The current position based on the limelight
     */
    public Vector2d Evaluate()
    {
        LimeLightData data = LimeLight.GetCurrent();

        double Theta = Robot.SwerveDrive.GetHeading();
        double Distance = Robot.Thrower.GetDistance();

        _currentPos = new Polar(Distance, Theta - data.GetAngleX()).vector();

        return _currentPos;
    }

    /**
     * Get the current position
     * @return The current position
     */
    public Vector2d GetCurrent()
    {
        return _currentPos;
    }
}
