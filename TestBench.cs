using System;

namespace FovCubeMaker;

class TestBench
{
    public static void Test()
    {
        // Example usage:
        // Given origin point (0, 0, 0), second known point (0, 0, 111.1),
        // internal angles (45, 45, 90), and rotation angle of 45 degrees
        double[] origin = { 0, 0, 0 };
        double[] secondPoint = { 0, 0, 111.1 };
        double[] result = CalculateRotatedPoint(origin, secondPoint, 0);

        Console.WriteLine($"Point A: ({result[0]}, {result[1]}, {result[2]})");
    }

    static double[] CalculateRotatedPoint(double[] origin, double[] secondPoint, double rotationAngle)
    {
        // Calculate side lengths
        double sideA = Math.Sqrt(Math.Pow(secondPoint[0] - origin[0], 2) + Math.Pow(secondPoint[1] - origin[1], 2) + Math.Pow(secondPoint[2] - origin[2], 2));
        double sideB = sideA;
        double sideC = sideA;

        // Convert rotation angle to radians
        double rotationRadians = Math.PI * rotationAngle / 180.0;

        // Calculate coordinates of point A using trigonometry
        double xa = Math.Cos(rotationRadians) * sideA;
        double ya = Math.Sin(rotationRadians) * sideA;

        // Resulting coordinates of point A
        double[] result = { origin[0] + xa, origin[1] + ya, origin[2] };

        return result;
    }


}