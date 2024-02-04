
using System;

namespace FovCubeMaker;

// Issues
//1. alt is in degrees of the eath, so 1 = 111.1 km
//2. if base bos is not under camera it wont work. for claritys sake.
//3. need unit tests with third party calcs
//4. would be best to use the curave of the earth instead of treating it as basically flat.
internal static class Program
{
    static void Main(string[] args)
    {
        Console.WriteLine("Start Test 1 ");
        LLA basePosLLA = new LLA(0, 0, 0);
        LLA CameraPosLLA = new LLA(0, 0, 1);
        Console.WriteLine($"CameraPos lat: {CameraPosLLA.lat}, lon {CameraPosLLA.lon}, {CameraPosLLA.alt}");
        Console.WriteLine($"basePos lat: {basePosLLA.lat}, lon {basePosLLA.lon}, {basePosLLA.alt}");
        Console.WriteLine($"Angle of view down: 45, FOV width: 2, FOV Height: 2 ,  Azimuth of Camera: 0");

        var ConvertedBoundingBoxLLA1 = CalcBoundingBoxBothPosLLA(basePosLLA, CameraPosLLA, 45, 2, 2,0);
        ConvertedBoundingBoxLLA1.PrintCoordinated();

        Console.WriteLine("Test 2 ");
        Console.WriteLine($"Angle of view down: 45, FOV width: 2, FOV Height: 2 ,  Azimuth of Camera: 90");
        var ConvertedBoundingBoxLLA2 = CalcBoundingBoxBothPosLLA(basePosLLA, CameraPosLLA, 45, 2, 2, 90);
        ConvertedBoundingBoxLLA2.PrintCoordinated();

        Console.WriteLine("Test 3 ");
        Console.WriteLine($"Angle of view down: 45, FOV width: 2, FOV Height: 2 ,  Azimuth of Camera: 45");
        var ConvertedBoundingBoxLLA3 = CalcBoundingBoxBothPosLLA(basePosLLA, CameraPosLLA, 45, 2, 2, 45);
        ConvertedBoundingBoxLLA3.PrintCoordinated();

        Console.WriteLine("Test 4 Realistic Values");
        LLA basePosLLA2 = new LLA(53.230152, -0.669415, 0);
        LLA CameraPosLLA2 = new LLA(53.230152, -0.669415, 0.5);
        Console.WriteLine($"CameraPos lat: {CameraPosLLA2.lat}, lon {CameraPosLLA2.lon}, {CameraPosLLA2.alt}");
        Console.WriteLine($"basePos lat: {basePosLLA2.lat}, lon {basePosLLA2.lon}, {basePosLLA2.alt}");
        Console.WriteLine($"Angle of view down: 32, FOV width: 3,4, FOV Height: 3.8 ,  Azimuth of Camera: 198");
        var ConvertedBoundingBoxLLA4 = CalcBoundingBoxBothPosLLA(basePosLLA2, CameraPosLLA2, 32, 3.4, 3.8, 198);
        ConvertedBoundingBoxLLA4.PrintCoordinated();

        Console.WriteLine("Fin");
    }


    static LLA CalculateIntersectionPointLLA(double h, double k, double r, double theta)
    {
        // Convert the angle to radians
        double radians = Math.PI * theta / 180.0;

        // Calculate the coordinates
        double x = h + r * Math.Cos(radians);
        double y = k + r * Math.Sin(radians);

        // Return the result as a Tuple
        LLA pos = new LLA(x, y, 0);
        //Console.WriteLine($"$Pre normalise: {pos.lat} , {pos.lon} , {pos.alt}");
        pos.NormaliseCoords();
        //Console.WriteLine($"post normalise: {pos.lat} , {pos.lon} , {pos.alt}");

        return pos;

    }

    static BoundingBoxLLA CalcBoundingBoxBothPosLLA(LLA BasePos, LLA CameraPos, double AngleOfView, double FovWidth, double FovHeight, double RotationAngle)
    {
        double sideBDistance = Utility.DistacnceBetweenLLA(BasePos, CameraPos);

        // Create two triangles to represent the field of view top and bottom
        Triangle3D triangle3dBot = new Triangle3D(AngleOfView - FovHeight, sideBDistance, (BasePos.lat, BasePos.lon, BasePos.alt), (CameraPos.lat, CameraPos.lon, CameraPos.alt), RotationAngle);
        triangle3dBot.PrintCoordinated();

        Triangle3D triangle3dTop = new Triangle3D(AngleOfView + FovHeight, sideBDistance, (BasePos.lat, BasePos.lon, BasePos.alt), (CameraPos.lat, CameraPos.lon, CameraPos.alt), RotationAngle);
        triangle3dTop.PrintCoordinated();

        // Calculate the coordinates of the bounding box, using Basic Circle Equation and Parametric Equations using trigonometry
        // a circle from each triangle and the points of itnersection at the angle of fov
        LLA topLeft = CalculateIntersectionPointLLA(BasePos.lat, BasePos.lon, triangle3dTop.sideB, RotationAngle + -FovWidth);
        LLA topRight = CalculateIntersectionPointLLA(BasePos.lat, BasePos.lon, triangle3dTop.sideB, RotationAngle + FovWidth);
        LLA botLeft = CalculateIntersectionPointLLA(BasePos.lat, BasePos.lon, triangle3dBot.sideB, RotationAngle + -FovWidth);
        LLA botRight = CalculateIntersectionPointLLA(BasePos.lat, BasePos.lon, triangle3dBot.sideB, RotationAngle + FovWidth);

        // Create and return the bounding box
        return new BoundingBoxLLA(topLeft, topRight, botLeft, botRight);
    }
}

