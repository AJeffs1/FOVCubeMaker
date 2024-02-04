
namespace FovCubeMaker
{
    public class Triangle3D 
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

        public Triangle3D(double angleB, double sideA, (double, double, double) pointC, (double, double, double) pointB, double rotationAngle) // Target is point a
        {
            this.angleA = (180 - angleB) - angleC;
            this.angleB = angleB;
            // angle c still = 90

            // calculate side a
            this.sideA = sideA;
            // calculate sideb
            this.sideB = sideA * Math.Tan(angleB * (Math.PI / 180));
            // calculate side c
            this.sideC = sideA / Math.Cos(angleB * (Math.PI / 180));


            // Convert rotation angle to radians
            double rotationRadians = Math.PI * rotationAngle / 180.0;

            // Calculate coordinates of point A using trigonometry
            double xa = Math.Cos(rotationRadians) * sideB;
            double ya = Math.Sin(rotationRadians) * sideB;

            // Resulting coordinates of point A
            this.pointA3D = (pointC.Item1 + xa, pointC.Item2 + ya, pointC.Item3); 
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