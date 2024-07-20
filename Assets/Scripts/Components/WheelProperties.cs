using Unity.Entities;

namespace ECSExperiment.Wheels
{
    public struct WheelProperties : IComponentData
    {
        public Entity Entity;
        public Entity VehicleEntity;
        public Entity WheelVisualObjectEntity;

        public float Spring;
        public float Damper;
        public float Mass;
        public float Radius;
        public float SpringLength;
        public float WheelsAmountFraction;
        public float Compression;

        public bool IsGrounded;
        public bool CanDrive;

        public WheelSide Side;
    }
}
