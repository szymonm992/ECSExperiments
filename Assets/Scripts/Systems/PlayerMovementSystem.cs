
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

public partial struct PlayerMovementSystem : ISystem
{
    public void OnUpdate(ref SystemState state)
    {
        foreach(var (inputs, properties, transform) in SystemAPI.Query<RefRO<EntityInputsData>, RefRO<VehicleEntityProperties>, RefRW<LocalTransform>>())
        {
            float3 position = transform.ValueRO.Position;
            bool isInputPositive = inputs.ValueRO.Vertical.IsNumberPositive();

            position.z += inputs.ValueRO.Vertical * 
                (isInputPositive ? properties.ValueRO.VehicleMaximumForwardSpeed : properties.ValueRO.VehicleMaximumBackwardSpeed) 
                * SystemAPI.Time.DeltaTime;

            transform.ValueRW.Position = position;
        }
    }
}
