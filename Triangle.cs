

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

        //public Triangle3D(double angleB, double sideA1 (double, double, double) pointC, (double, double, double) pointB) // Target is point a
        //{
        //    this.angleA = (180 - angleB) - angleC;
        //    this.angleB = angleB;
        //    // angle c still = 90

        //    // calculate side a
        //    this.sideA = sideA1;
        //    // calculate side b
        //    this.sideB = Math.Sqrt(Math.Pow(sideA1, 2) + Math.Pow(sideA1, 2) - 2 * sideA * sideA * Math.Cos(angleA));

        //    // calculate side c
        //    this.sideC = Math.Sqrt(Math.Pow(sideA1, 2) + Math.Pow(sideB, 2) - 2 * sideA * sideB * Math.Cos(angleB));


        //    double sideA = Math.Sqrt(Math.Pow(secondPoint[0] - origin[0], 2) + Math.Pow(secondPoint[1] - origin[1], 2) + Math.Pow(secondPoint[2] - origin[2], 2));
        //    //double sideB = sideA;
        //    double sideC = sideA / Math.Sin(Math.PI * angleAtSecondPoint / 180.0);


        //    // calculate point a
        //    this.pointA3D = (pointC.Item1 + sideA, pointC.Item2 + sideB, pointC.Item3); // target intersect // TODO wrong

        //    // calculate point a
        //    this.pointB3D = pointB;
        //    //Calculate point c
        //    this.pointC3D = pointC;
        //}

        public Triangle3D(double angleB, double sideA, (double, double, double) pointC, (double, double, double) pointB, double rotationAngle) // Target is point a
        {

            //SolveWithOneSideAndTwoAngles();
            this.angleA = (180 - angleB) - angleC;
            this.angleB = angleB;
            // angle c still = 90

            // calculate side a
            this.sideA = sideA;
            // calculate side b
            //this.sideB = Math.Sqrt(Math.Pow(sideA, 2) + Math.Pow(sideA, 2) - 2 * sideA * sideA * Math.Cos(angleA));
            this.sideB = sideA * Math.Tan(angleB * (Math.PI / 180));

            // calculate side c
            //this.sideC = Math.Sqrt(Math.Pow(sideA, 2) + Math.Pow(sideB, 2) - 2 * sideA * sideB * Math.Cos(angleB));
            this.sideC = sideA / Math.Cos(angleB * (Math.PI / 180));
            // calculate point a

            // Convert rotation angle to radians
            double rotationRadians = Math.PI * rotationAngle / 180.0;

            double sideArotateCalc = Math.Sqrt(Math.Pow(pointB.Item1 - pointC.Item1, 2) + Math.Pow(pointB.Item2 - pointC.Item2, 2) + Math.Pow(pointB.Item3 - pointC.Item3, 2));
            // Calculate coordinates of point A using trigonometry
            double xa = Math.Cos(rotationRadians) * sideB;
            double ya = Math.Sin(rotationRadians) * sideB;

            // Resulting coordinates of point A
            this.pointA3D = (pointC.Item1 + xa, pointC.Item2 + ya, pointC.Item3); // target intersect // TODO wrong

            //this.pointA3D = (pointC.Item1 + sideA, pointC.Item2 + sideB, pointC.Item3); // target intersect // TODO wrong

            // calculate point a
            this.pointB3D = pointB;
            //Calculate point c
            this.pointC3D = pointC;
        }

        private void SolveWithOneSideAndTwoAngles()
        {
            if (sideA > 0 && angleC > 0 && angleB > 0)
            {
                if (angleC + angleB < 90)
                {
                    angleA = 90 - angleC - angleB;
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

        //static LLA CalculateRotatedPoint(double[] origin, double[] secondPoint, double rotationAngle)
        //{
        //    // Calculate side lengths
        //    double sideA = Math.Sqrt(Math.Pow(secondPoint[0] - origin[0], 2) + Math.Pow(secondPoint[1] - origin[1], 2) + Math.Pow(secondPoint[2] - origin[2], 2));
        //    double sideB = sideA;
        //    double sideC = sideA;

        //    // Convert rotation angle to radians
        //    double rotationRadians = Math.PI * rotationAngle / 180.0;

        //    // Calculate coordinates of point A using trigonometry
        //    double xa = Math.Cos(rotationRadians) * sideA;
        //    double ya = Math.Sin(rotationRadians) * sideA;

        //    // Resulting coordinates of point A
        //    LLA result = { origin[0] + xa, origin[1] + ya, origin[2] };

        //    return result;
        //}


        public void PrintCoordinated()
        {
            Console.WriteLine("Point A: " + this.pointA3D + " Point B: " + this.pointB3D + " Point C: " + this.pointC3D);
            Console.WriteLine("Side A: " + this.sideA + " Side B: " + this.sideB + " Side C: " + this.sideC);
            Console.WriteLine("Angle A: " + this.angleA + " Angle B: " + this.angleB + " Angle C: " + this.angleC + "\n");
        }
    }

 
}

