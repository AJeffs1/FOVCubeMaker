
using System;

namespace FovCubeMaker;

internal static class Program
{
    const double EARTH_RADIUS = 6371000; // Earth's radius in meters
    static void Main(string[] args)
    {
        Console.WriteLine("Hello World!");
        Console.WriteLine("");

        // Test approach in total ecef

        ECEF Spherecenter = new ECEF(0, 0, 0);
        double SphereRadius = 6371000;
        LLA pointA = new LLA(1, 1, 0);
        LLA pointB = new LLA(1, 1, 111100);
        LLA pointC = new LLA(2, 1, 0);

        LLA estAnws2 = new LLA(0.5, 0.5, 0);


        ECEF pointaECEF = Utility.TurnLLAIntoPosition(pointA);
        ECEF pointbECEF = Utility.TurnLLAIntoPosition(pointB);
        ECEF pointcECEF = Utility.TurnLLAIntoPosition(pointC);
        ECEF estAnws2ECEF = Utility.TurnLLAIntoPosition(estAnws2);


        ECEF tempAnswer = new(6487266.966672945, -4382755.243, 4615498.646);
        ECEF tempAnswer2 = new(10940257.876, 4616226.576, -4382483.172);

        Console.WriteLine($"Shere center is : {Spherecenter.x},Y:{Spherecenter.y},Z: {Spherecenter.z}");
        Console.WriteLine($"Shere radius is : 6371000");
        Console.WriteLine($"point A:{pointaECEF.x},Y:{pointaECEF.y},Z: {pointaECEF.z}");
        Console.WriteLine($"point B:{pointbECEF.x},Y:{pointbECEF.y},Z: {pointbECEF.z}");
        Console.WriteLine($"point C:{pointcECEF.x},Y:{pointcECEF.y},Z: {pointcECEF.z} -<");
        Console.WriteLine("");
        Console.WriteLine($"estAnws2ECEF C:{estAnws2ECEF.x},Y:{estAnws2ECEF.y},Z: {estAnws2ECEF.z}");
        Console.WriteLine("");

        Console.WriteLine($"tempAnswer 1 ECEF :{tempAnswer.x},Y:{tempAnswer.y},Z: {tempAnswer.z}");
        LLA tempAnswerLLA = Utility.TurnECEFIntoLLA(tempAnswer);
        Console.WriteLine($"tempAnswer 1 LLA:{tempAnswerLLA.lat},Y:{tempAnswerLLA.lon},Z: {tempAnswerLLA.alt}");

        Console.WriteLine($"tempAnswer 2 ECEF :{tempAnswer2.x},Y:{tempAnswer2.y},Z: {tempAnswer2.z}");
        LLA tempAnswer2LLA = Utility.TurnECEFIntoLLA(tempAnswer2);
        Console.WriteLine($"tempAnswer 2 LLA:{tempAnswer2LLA.lat},Y:{tempAnswer2LLA.lon},Z: {tempAnswer2LLA.alt}");

        // thew question is, at point B, facing the correct angle of 0 degrees azimuth and -45degrees elevation, at what point do i intersect the circle

        Console.WriteLine("");
        Console.WriteLine("");

        // Test conversion
        LLA posLLA = new LLA(40.7128, -74.0060, 10.0);
        ECEF posEcef = Utility.TurnLLAIntoPosition(posLLA);
        //Console.WriteLine($"Start LLA: X:{posLLA.lat},Y:{posLLA.lon},Z{posLLA.alt}");
        //Console.WriteLine($"posEcef: X:{posEcef.x},Y:{posEcef.y},Z{posEcef.z}");
        LLA postPosLLA = Utility.TurnECEFIntoLLA(posEcef);
        //Console.WriteLine($"postPosLLA LLA: X:{postPosLLA.lat},Y:{postPosLLA.lon},Z{postPosLLA.alt}");


        // need to take reaslisc data and convert it to ECEF see if it matches

        // if we are at 0,0 lat long and 111.1 km above sea level, looking north at -45 degrees elevation. what it is coordinate we are looking at when we itnersect with the earth

        LLA CameraLLA = new LLA(1, 1, 111100);
        ECEF CameraEcef = Utility.TurnLLAIntoPosition(CameraLLA);

        LLA intersectionPoint = CalculateIntersection(CameraLLA.lat, CameraLLA.lon, CameraLLA.alt, 0, -45);
        LLA intersectionPoint2 = CalculateIntersection2(CameraLLA.lat, CameraLLA.lon, CameraLLA.alt, 0, -45);

        double FOV = 5; // field of view in degrees

        Console.WriteLine($"CameraLLA LLA: X:{CameraLLA.lat},Y:{CameraLLA.lon},Z: {CameraLLA.alt}");
        Console.WriteLine($"Test 1 LLA: X:{intersectionPoint.lat},Y:{intersectionPoint.lon},Z: {intersectionPoint.alt}");
        Console.WriteLine($"Test  LLA: Lat: {intersectionPoint2.lat},Long: {intersectionPoint2.lon},alt: {intersectionPoint2.alt}");
        // we are looking at -45 degrees elevation


        // pretend its correct 

        // take postion and values and calculate the intersection point, (we will have a testing example of it)

        // calculate the fov increase in x and y direction,
        // take the bearings calculated and record where there intersection points are.

        // plot all points as a polygon

        // plot on map and see if it matches
        Vector3D origin = new Vector3D(1, 1, 111100);
        Vector3D vector = EarthIntersectPointFinder.AzimuthElevationToUnitVector(0, -45);
        double earthRadius = EARTH_RADIUS;
        Console.WriteLine($"1 start ");
        //LLA skyfieldExample = EarthIntersectPointFinder.FindEarthIntersectPoint(origin, vector, earthRadius);

        Console.WriteLine($"1 FIN ");

        //Console.WriteLine($"skyfield LLA: Lat:{skyfieldExample.lat},Long:{skyfieldExample.lon},alt{skyfieldExample.alt}");


        // maths check 

        Console.WriteLine($"Maths check ");

        Console.WriteLine(Math.Cos(-45));
        Console.WriteLine(Math.Sin(-45));
        Console.WriteLine(Math.Sqrt(2)/2);

    }

    static LLA CalculateIntersection(double latitude, double longitude, double altitude, double azimuth, double elevation)
    {
        // Convert angles to radians for trigonometric functions
        double elevationRad = elevation * Math.PI / 180;
        double azimuthRad = azimuth * Math.PI / 180;

        // Calculate the distance from the camera to the ground along the line of sight
        double distanceToGround = altitude / Math.Tan(elevationRad);

        // Calculate the horizontal distance on the ground
        double horizontalDistance = distanceToGround * Math.Cos(elevationRad);

        // Calculate the x and y components of the horizontal distance
        double xComponent = horizontalDistance * Math.Cos(azimuthRad);
        double yComponent = horizontalDistance * Math.Sin(azimuthRad);

        // Approximate conversion factors for latitude and longitude
        double latConversion = 111111; // Approximate for latitude
        double lonConversion = 111111 * Math.Cos(latitude * Math.PI / 180); // Adjust for latitude

        double intersectionLatitude = latitude + (xComponent / latConversion);
        double intersectionLongitude = longitude + (yComponent / (lonConversion));

        return new LLA(intersectionLatitude, intersectionLongitude,0);
    }

    static LLA CalculateIntersection2(double latitude, double longitude, double altitude, double azimuth, double elevation)
    {
        // Convert angles to radians for trigonometric functions
        double azimuthRad = azimuth * Math.PI / 180;
        double elevationRad = elevation * Math.PI / 180;

        // Calculate horizontal and vertical components
        double horizontalDistance = altitude * Math.Tan(elevationRad);
        double xComponent = horizontalDistance * Math.Cos(azimuthRad);
        double yComponent = horizontalDistance * Math.Sin(azimuthRad);

        // Approximate conversion factors for latitude and longitude
        double latConversion = 111111; // Approximate for latitude
        double lonConversion = 111111 * Math.Cos(latitude * Math.PI / 180); // Adjust for latitude

        // Calculate the intersection coordinates
        double intersectionLatitude = latitude + (xComponent / latConversion);
        double intersectionLongitude = longitude + (yComponent / lonConversion);

        return new LLA(intersectionLatitude, intersectionLongitude, 0);
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