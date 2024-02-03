

namespace FovCubeMaker
{

    public class Triangle2D // Class representing a triangle
    {
        public (double, double) pointA2D;
        public (double, double) pointB2D;
        public (double, double) pointC2D;
        public double sideA;
        public double sideB;
        public double sideC;
        public double angleA;
        public double angleB;
        public double angleC = 90;

        public Triangle2D(double angleB, double sideA, (double, double) pointC, (double, double) pointB) // Target is point a
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
            this.pointA2D = (pointC.Item1 + sideA, pointC.Item2 + sideB); // target intersect
            // calculate point a
            this.pointB2D = pointB;
            //Calculate point c
            this.pointC2D = pointC;

        }

        public void PrintCoordinated()
        {
            Console.WriteLine("Point A: " + this.pointA2D + " Point B: " + this.pointB2D + " Point C: " + this.pointC2D);
            Console.WriteLine("Side A: " + this.sideA + " Side B: " + this.sideB + " Side C: " + this.sideC);
            Console.WriteLine("Angle A: " + this.angleA + " Angle B: " + this.angleB + " Angle C: " + this.angleC + "\n");
        }
    }

    // next how to do in 3d space
    public class Triangle3D // Class representing a triangle
    {
        public (double, double, double) pointA3D;
        public (double, double, double) pointB3D;
        public (double, double, double) pointC3D;
        public double sideA;
        public double sideB;
        public double sideC;
        public double angleA;
        public double angleB;
        public double angleC = 90;

        public Triangle3D(double angleB, double sideA, (double, double, double) pointC, (double, double, double) pointB) // Target is point a
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
            this.pointA3D = (pointC.Item1 + sideA, pointC.Item2 + sideB, pointC.Item3); // target intersect

            // calculate point a
            this.pointB3D = pointB;
            //Calculate point c
            this.pointC3D = pointC;
        }

        public void PrintCoordinated()
        {
            Console.WriteLine("Point A: " + this.pointA3D + " Point B: " + this.pointB3D + " Point C: " + this.pointC3D);
            Console.WriteLine("Side A: " + this.sideA + " Side B: " + this.sideB + " Side C: " + this.sideC);
            Console.WriteLine("Angle A: " + this.angleA + " Angle B: " + this.angleB + " Angle C: " + this.angleC + "\n");
        }
    }

}
