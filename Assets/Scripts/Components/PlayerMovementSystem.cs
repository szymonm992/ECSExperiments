
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

public partial struct PlayerMovementSystem : ISystem
{
    public void OnUpdate(ref SystemState state)
    {
        foreach(var (inputs, transform) in SystemAPI.Query<RefRO<EntityInputsData>, RefRW<LocalTransform>>())
        {
            float3 position = transform.ValueRO.Position;
            position.z += inputs.ValueRO.Vertical * SystemAPI.Time.DeltaTime;
            transform.ValueRW.Position = position;
        }
    }
}
