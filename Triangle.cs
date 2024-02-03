

namespace FovCubeMaker
{

    public class Triangle // Class representing a triangle
    {
        public (double, double) pointA;
        public (double, double) pointB;
        public (double, double) pointC;
        public double sideA;
        public double sideB;
        public double sideC;
        public double angleA;
        public double angleB;
        public double angleC = 90;

        public Triangle(double angleB, double sideA, (double, double) pointC, (double, double) pointB) // Target is point a
        {

            

            this.angleA = (180 - angleB) - angleC;
            this.angleB = angleB;
            // angle c still = 90

            // calculate side a
            this.sideA = sideA;
            // calculate side b
            this.sideB = Math.Sqrt(Math.Pow(sideA, 2) + Math.Pow(sideA, 2) - 2 * sideA * sideA * Math.Cos(angleA));

            // calculate side c
            this.sideC = Math.Sqrt(Math.Pow(sideA, 2) + Math.Pow(sideB, 2) - 2 * sideA * sideB * Math.Cos(angleB));

            // calculate point a
            this.pointA = (pointC.Item1 + sideA, pointC.Item2 + sideB);
            // calculate point a
            this.pointB = pointB;
            //Calculate point c
            this.pointC = pointC;

        }

        public void PrintCoordinated()
        {
            Console.WriteLine("Point A: " + this.pointA + " Point B: " + this.pointB + " Point C: " + this.pointC);
            Console.WriteLine("Side A: " + this.sideA + " Side B: " + this.sideB + " Side C: " + this.sideC);
            Console.WriteLine("Angle A: " + this.angleA + " Angle B: " + this.angleB + " Angle C: " + this.angleC);
        }
    }




}
