package frc.robot.Math;

import edu.wpi.first.wpilibj.drive.Vector2d;

import java.lang.Math;

public class Mathf 
{
    public static double Clamp(double value, double min, double max)
    {
        return Math.max(min, Math.min(max, value));
    }
    
    public static double Lerp(double v1, double v2, double t)
    {
        return (t * (v2 - v1)) + v1;
    }
    public static Vector2d Lerp(Vector2d p1, Vector2d p2, double x)
    {
        return new Vector2d(x, ((p2.y - p2.y) * ((x - p1.x) / (p2.x - p1.x))) + p1.y);
    }

    public static double DeltaAngle(Vector2d Source, Vector2d Target)
    {
        double SourceAngle = Polar.fromVector(Source).azymuth;

        double xPrim = Target.x * Math.cos(SourceAngle) - Target.y * Math.sin(SourceAngle); //Change of coordinate system
        double yPrim = Target.x * Math.sin(SourceAngle) + Target.y * Math.cos(SourceAngle);

        double angle = Math.atan2(yPrim, xPrim); //Angle betwen Source and target

        return angle;
    }
}
