package frc.robot.Component.Data;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Math.Mathf;

public class RobotPosition 
{
    public RobotPosition(Vector2d InitialPos)
    {
        _pos = InitialPos;
    }

    private Vector2d _pos;
    private Vector2d _velocity;

    public Vector2d GetPosition()
    {
        return _pos;
    }
    public Vector2d GetVelocity()
    {
        return _velocity;
    }

    public void Evaluate(Vector2d Acceleration, double DT)
    {
        _pos.x += (_velocity.x * DT) + 0.5 * (Acceleration.x * DT * DT);
        _pos.y += (_velocity.y * DT) + 0.5 * (Acceleration.y * DT * DT);

        _velocity.x += Acceleration.x * DT;
        _velocity.y += Acceleration.y * DT;
    }

    public void Reset()
    {
        _pos = new Vector2d(0, 0);
        _velocity = new Vector2d(0, 0);
    }
}
