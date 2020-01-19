package frc.robot.Component.Data;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class RobotPosition 
{
    public RobotPosition(Vector2d InitialPos)
    {
        _pos = InitialPos;
    }

    private Vector2d _pos = new Vector2d(0, 0);
    private Vector2d _velocity = new Vector2d(0, 0);
    private Vector2d _lastAccel = new Vector2d(0, 0);

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
        double xAccel = Acceleration.x;
        double yAccel = Acceleration.y;

        _pos.x += (_velocity.x * DT) + 0.5 * (xAccel * DT * DT);
        _pos.y += (_velocity.y * DT) + 0.5 * (yAccel * DT * DT);

        _velocity.x += xAccel * DT;
        _velocity.y += yAccel * DT;

        _lastAccel = new Vector2d(Acceleration.x, Acceleration.y);
    }

    public void SetPosition(Vector2d Position)
    {
        _pos = Position;
    }

    public void Reset()
    {
        _pos = new Vector2d(0, 0);
        _velocity = new Vector2d(0, 0);
        _lastAccel = new Vector2d(0, 0);
    }
}
