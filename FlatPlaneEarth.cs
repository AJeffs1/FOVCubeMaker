
using System;

namespace FovCubeMaker;

internal static class FlatPlaneEarth
{
    const double EARTH_RADIUS = 6371000; // Earth's radius in meters
    static void Default(string[] args)
    {
        Console.WriteLine("Hello flat World!");
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

        

    }

    //static LLA CalculateIntersection(double latitude, double longitude, double altitude, double azimuth, double elevation)
    //{
    //    // method uses a flat plane and a triangle trig equation to find the intersection point of a line of sight

    //    // convert LLA into Flat ECEF

    //    // create Triangle trig equation given one side and two  angles 90 degrees and elevation

    //    // using that calculate length of B side of triangle

    //    // using that propergate out to find the intersection point

    //    return new LLA(intersectionLatitude, intersectionLongitude,0);
    //}

    //static LLA CalculateCorners(double latitude, double longitude, double altitude, double azimuth, double elevation, double FOVAngle, double widthToHieghtRatio)
    //{
    //    // method uses a flat plane and a triangle trig equation to find the intersection point of a line of sight
    //    // creating 4 offset angles based on the FOV dimensions 

    //    // returns a 5 point polygon of the camera location and the 4 corners of the FOV meeting the earth


    //    return new LLA(intersectionLatitude, intersectionLongitude, 0);
    //}
}


