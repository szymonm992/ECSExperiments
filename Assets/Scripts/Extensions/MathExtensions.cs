using Unity.Mathematics;

namespace ECSExperiment.Wheels
{
    public static class MathExtensions
    {
        public static float Sqr(float x) => x * x;

        public static float Clamp01(this float value)
        {
            return math.clamp(value, 0f, 1f);
        }
    }
}
