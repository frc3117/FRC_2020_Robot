package frc.robot.Math;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class Polar 
{
    public Polar(double Radius, double Azymuth)
    {
        radius = Radius;
        azymuth = Azymuth;
    }

    public double radius;
    public double azymuth;

    public Polar add(Polar pol)
    {
        Vector2d vec1 = vector();
        Vector2d vec2 = pol.vector();

        return Polar.fromVector(new Vector2d(vec1.x + vec2.x, vec1.y + vec2.y));
    }
    public Polar sub(Polar pol)
    {
        Vector2d vec1 = vector();
        Vector2d vec2 = pol.vector();

        return Polar.fromVector(new Vector2d(vec1.x - vec2.x, vec1.y - vec2.y));
    }

    public Vector2d vector()
    {
        return new Vector2d(radius * Math.cos(azymuth), radius * Math.sin(azymuth));
    }
    public static Polar fromVector(Vector2d vec)
    {
        return new Polar(vec.magnitude(), Math.atan2(vec.x, vec.y));
    }
}
