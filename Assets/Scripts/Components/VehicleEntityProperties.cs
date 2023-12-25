using Unity.Entities;

public struct VehicleEntityProperties : IComponentData
{
    public Entity VehicleEntity;
    public float VehicleMaximumForwardSpeed;
    public float VehicleMaximumBackwardSpeed;

    public float CurrentSpeed;
}
