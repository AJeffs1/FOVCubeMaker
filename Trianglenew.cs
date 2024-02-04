using System;

namespace FovCubeMaker;
public class Triangle
{
    public double angleA { get; set; }
    public double angleB { get; set; }
    public double angleC { get; set; }

    public double sideA { get; set; }
    public double sideB { get; set; }
    public double sideC { get; set; }


    public Triangle(double side, double angleA, double angleB)
    {
        sideA = side;
        this.angleA = angleA;
        this.angleB = angleB;
        SolveWithOneSideAndTwoAngles();
    }

    private void SolveWithOneSideAndTwoAngles()
    {
        if (sideA > 0 && angleA > 0 && angleB > 0)
        {
            if (angleA + angleB < 90)
            {
                angleC = 90 - angleA - angleB;
                sideB = sideA * Math.Tan(angleB * (Math.PI / 180));
                sideC = sideA / Math.Cos(angleB * (Math.PI / 180));
            }
            else
            {
                Console.WriteLine("The sum of angles A and B must be less than 90 degrees for a right-angled triangle.");
            }
        }
        else
        {
            Console.WriteLine("Side and angles must have positive values.");
        }
    }

}

//class Program
//{
//    static void Main()
//    {
//        // Example usage:
//        double side = 3;
//        double angleA = 30;
//        double angleB = 60;

//        Triangle myTriangle = new Triangle(side, angleA, angleB);

//        Console.WriteLine($"Side a: {myTriangle.sideA}");
//        Console.WriteLine($"Side b: {myTriangle.sideB}");
//        Console.WriteLine($"Side c: {myTriangle.sideC}");
//        Console.WriteLine($"Angle A: {myTriangle.angleA} degrees");
//        Console.WriteLine($"Angle B: {myTriangle.angleB} degrees");
//        Console.WriteLine($"Angle C: {myTriangle.angleC} degrees");
//    }
//}
