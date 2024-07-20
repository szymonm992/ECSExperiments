using Unity.Entities;

namespace ECSExperiment.Wheels
{
    public struct VehicleProperties : IComponentData
    {
        public Entity VehicleEntity;

        public int WheelsAmount;

        public float VehicleMaximumForwardSpeed;
        public float VehicleMaximumBackwardSpeed;

        public float CurrentSpeed;
        public bool IsGrounded;
    }
}
