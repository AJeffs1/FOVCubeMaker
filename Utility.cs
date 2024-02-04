

namespace FovCubeMaker
{

    public class ECEF
    {
        public double x;
        public double y;
        public double z;

        public ECEF(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    public class FlatECEF
    {
        public double x;
        public double y;
        public double z;

        public FlatECEF(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }
    public class LLA
    {
        public double lat;
        public double lon;
        public double alt;

        public LLA(double lat, double lon, double alt)
        {
            this.lat = lat;
            this.lon = lon;
            this.alt = alt;
        }

        public void NormaliseCoords() 
        {
            if (this.lat > 90) this.lat = this .lat - 180;
            if (this.lat < -90) this.lat = this.lat + 180;
            if (this.lon > 180) this.lon = this.lon - 360;
            if (this.lon < -180) this.lon = this.lon + 360;
        }
    }
    public class Coordinate
    {
        public double x;
        public double y;
        public double z;

        public Coordinate(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }
    public class BoundingBox
    {
        public Coordinate topLeft;
        public Coordinate topRight;
        public Coordinate botLeft;
        public Coordinate botRight;

        public BoundingBox(Coordinate topLeft, Coordinate topRight, Coordinate botLeft, Coordinate botRight)
        {
            this.topLeft = topLeft;
            this.topRight = topRight;
            this.botLeft = botLeft;
            this.botRight = botRight;
        }
        public void PrintCoordinated()
        {
            Console.WriteLine("TopLeft: " + topLeft.x + " " + topLeft.y + ", " + topLeft.z + " TopRight: " + topRight.x + " " + topRight.y + ", " + topRight.z);
            Console.WriteLine("BotLeft: " + botLeft.x + " " + botLeft.y + ", " + topLeft.z + " BotRight: " + botRight.x + " " + botRight.y + ", " + botRight.z + "\n");
        }
    }
    public class BoundingBoxLLA
    {
        public LLA topLeft;
        public LLA topRight;
        public LLA botLeft;
        public LLA botRight;

        public BoundingBoxLLA(LLA topLeft, LLA topRight, LLA botLeft, LLA botRight)
        {
            this.topLeft = topLeft;
            this.topRight = topRight;
            this.botLeft = botLeft;
            this.botRight = botRight;
        }

        public void PrintCoordinated()
        {
            Console.WriteLine("TopLeft: " + topLeft.lat + ", " + topLeft.lon + ", " + topLeft.alt + "  TopRight: " + topRight.lat + ", " + topRight.lon + ", " + topRight.alt);
            Console.WriteLine("BotLeft: " + botLeft.lat + ", " + botLeft.lon + ", " + botLeft.alt + " BotRight: " + botRight.lat + ", " + botRight.lon + ", " + botRight.alt + "\n");
        }
    }


    public static class Utility
    {

        public static ECEF TurnLLAIntoPosition(LLA lla)
        {
            double a = 6378137.0; // Earth's semi-major axis in meters
            double f = 1.0 / 298.257223563; // Earth's flattening
            double b = a * (1.0 - f); // Earth's semi-minor axis
            double eSquared = 1 - (b * b) / (a * a); // Eccentricity squared

            double sinLat = Math.Sin(lla.lat * Math.PI / 180.0);
            double cosLat = Math.Cos(lla.lat * Math.PI / 180.0);
            double sinLon = Math.Sin(lla.lon * Math.PI / 180.0);
            double cosLon = Math.Cos(lla.lon * Math.PI / 180.0);

            double N = a / Math.Sqrt(1 - eSquared * sinLat * sinLat);
            double x = (N + lla.alt) * cosLat * cosLon;
            double y = (N + lla.alt) * cosLat * sinLon;
            double z = (N * (1 - eSquared) + lla.alt) * sinLat;

            return new ECEF(x, y, z);
        }

        public static LLA TurnECEFIntoLLA(ECEF ecef)
        {
            double a = 6378137.0; // Earth's semi-major axis in meters
            double f = 1.0 / 298.257223563; // Earth's flattening
            double b = a * (1.0 - f); // Earth's semi-minor axis
            double eSquared = 1 - (b * b) / (a * a); // Eccentricity squared
            double ePrimeSquared = (a * a - b * b) / (b * b); // Eccentricity prime squared

            double p = Math.Sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
            double theta = Math.Atan2(ecef.z * a, p * b);

            double latitude = Math.Atan2(ecef.z + ePrimeSquared * b * Math.Pow(Math.Sin(theta), 3), p - eSquared * a * Math.Pow(Math.Cos(theta), 3));
            double longitude = Math.Atan2(ecef.y, ecef.x);
            double N = a / Math.Sqrt(1 - eSquared * Math.Sin(latitude) * Math.Sin(latitude));

            double altitude = p / Math.Cos(latitude) - N;

            latitude *= 180.0 / Math.PI;
            longitude *= 180.0 / Math.PI;

            return new LLA(latitude, longitude, altitude);
        }

        public static double DistacnceBetweenECEF(ECEF ecef1, ECEF ecef2)
        {
            double x = ecef1.x - ecef2.x; // is this always positive ??
            double y = ecef1.y - ecef2.y;
            double z = ecef1.z - ecef2.z;
            double distance = Math.Sqrt(x * x + y * y + z * z);
            if (distance < 0)
            {
                Console.WriteLine("Distance is negative"); // TODO CLEAN
                distance = -distance;
            }
            return distance;
        }

        public static double DistacnceBetweenLLA(LLA LLA1, LLA LLA2)
        {
            double x = LLA1.lat - LLA2.lat; // is this always positive ??
            double y = LLA1.lon - LLA2.lon;
            double z = LLA1.alt - LLA2.alt;
            double distance = Math.Sqrt(x * x + y * y + z * z);
            if (distance < 0)
            {
                Console.WriteLine("Distance is negative"); // TODO CLEAN
                distance = -distance;
            }
            return distance;
        }
    }
}
