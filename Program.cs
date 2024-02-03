
using System;

namespace FovCubeMaker;

internal static class Program
{
    const double EARTH_RADIUS = 6371000; // Earth's radius in meters
    static void Main(string[] args)
    {
        Console.WriteLine("start");

        Triangle triangle = new Triangle(45, 111.1, (0, 0), (0, 111.1));
        triangle.PrintCoordinated();
    }



    // Convert angles to radians



}
public class EarthIntersectPointFinder
{
    // Function to find the intersection point with Earth
    public static LLA FindEarthIntersectPoint(Vector3D origin, Vector3D vector, double earthRadius)
    {
        // Step 1: Turn the vector into a unit vector pointing in the same direction.
        Vector3D unitVectorTowardGround = vector.Normalize();

        // Step 2: Binary search to find the required vector length.
        // Step 2a: Extend the vector until it intersects with the Earth.
        double lengthGuess = 1;
        Console.WriteLine($"while 1 start ");
        while ((origin + unitVectorTowardGround * lengthGuess).Norm() > earthRadius)
        {
            Console.WriteLine((origin + unitVectorTowardGround * lengthGuess).Norm());
            lengthGuess *= 2;
        }

        // Upper bound on length is found.
        double lengthUpperBound = lengthGuess;

        // Lower bound is half the previous guess.
        double lengthLowerBound = lengthGuess / 2;

        // Step 2b: Binary search for the exact required length.
        double error = 9999;
        double tolerance = 0.1; // Tolerance of 0.1 km
        Console.WriteLine($"while 2 start ");

        while (Math.Abs(error) > tolerance)
        {
            // Standard binary search to adjust upper or lower bounds.
            double newGuess = (lengthLowerBound + lengthUpperBound) / 2;
            double newVectorEnd = (origin + unitVectorTowardGround * newGuess).Norm();

            if (newVectorEnd > earthRadius)
                lengthLowerBound = newGuess;
            else
                lengthUpperBound = newGuess;

            error = newVectorEnd - earthRadius;
            Console.WriteLine(Math.Abs(error));

        }

        // Calculate the intersection point
        Vector3D intersectionPoint = origin + unitVectorTowardGround * lengthLowerBound;

        // Here you'd need to implement your equivalent to wgs84.subpoint() in C# to get latitude and longitude.
        // Assume it returns latitude and longitude in degrees as a tuple.
        // Replace the line below with your implementation.
        (double latitude, double longitude) = GetLatitudeLongitude(intersectionPoint);

        return new LLA(latitude, longitude,0);
    }

    // Replace this method with your implementation of getting latitude and longitude from the intersection point.
    private static (double, double) GetLatitudeLongitude(Vector3D intersectionPoint)
    {
        // Implement your logic to get latitude and longitude from the intersection point.
        // Return values as a tuple of latitude and longitude.
        // For demonstration purposes, returning (0, 0) here.
        return (0, 0);
    }

    public static Vector3D AzimuthElevationToUnitVector(double azimuthDegrees, double elevationDegrees)
    {
        // Convert azimuth and elevation angles from degrees to radians
        double azimuthRad = azimuthDegrees * (Math.PI / 180.0);
        double elevationRad = elevationDegrees * (Math.PI / 180.0);

        // Calculate the components of the unit vector
        double x = Math.Cos(elevationRad) * Math.Cos(azimuthRad);
        double y = Math.Cos(elevationRad) * Math.Sin(azimuthRad);
        double z = Math.Sin(elevationRad);

        // Create and return the unit vector
        return new Vector3D(x, y, z);
    }
}

public class Vector3D // ?
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }

    public Vector3D(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    public double Norm()
    {
        return Math.Sqrt(X * X + Y * Y + Z * Z);
    }

    public Vector3D Normalize()
    {
        double length = Norm();
        return new Vector3D(X / length, Y / length, Z / length);
    }

    public static Vector3D operator *(Vector3D vector, double scalar)
    {
        return new Vector3D(vector.X * scalar, vector.Y * scalar, vector.Z * scalar);
    }

    public static Vector3D operator +(Vector3D vector1, Vector3D vector2)
    {
        return new Vector3D(vector1.X + vector2.X, vector1.Y + vector2.Y, vector1.Z + vector2.Z);
    }

}