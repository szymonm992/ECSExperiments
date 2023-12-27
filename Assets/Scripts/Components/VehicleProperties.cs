using Unity.Entities;

public struct VehicleProperties : IComponentData
{
    public Entity VehicleEntity;
    public float VehicleMaximumForwardSpeed;
    public float VehicleMaximumBackwardSpeed;

    public float CurrentSpeed;
    public bool IsGrounded;
}
