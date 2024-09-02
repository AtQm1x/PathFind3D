using OpenTK.Mathematics;

namespace PathFind3D
{
    public class MtH
    {
        public static double cubeRoot(double n)
        {
            return (System.Math.Pow(n, 1d / 3d));
        }

        public static float cubeRootF(float n)
        {
            return (System.MathF.Pow(n, 1f / 3f));
        }

        public static Vector3i Clamp(Vector3i value, Vector3i min, Vector3i max)
        {
            int clampedX = MathHelper.Clamp(value.X, min.X, max.X);
            int clampedY = MathHelper.Clamp(value.Y, min.Y, max.Y);
            int clampedZ = MathHelper.Clamp(value.Z, min.Z, max.Z);

            return new Vector3i(clampedX, clampedY, clampedZ);
        }
    }
}
