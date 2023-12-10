using Unity.Entities;
using Unity.Mathematics;


[RequireMatchingQueriesForUpdate]
[UpdateInGroup(typeof(InitializationSystemGroup))]
partial struct InputSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<VehicleInput>();
    }

    public void OnUpdate(ref SystemState state)
    {
        var input = SystemAPI.GetSingleton<VehicleInput>();

        foreach (var (speed, steering)
                 in SystemAPI.Query<RefRW<VehicleSpeed>, RefRW<VehicleSteering>>().WithAll<ActiveVehicle>())
        {
            float x = input.Steering.x;
            float a = input.Throttle;
            float z = input.Looking.x;

            var newSpeed = a * speed.ValueRW.TopSpeed;
            speed.ValueRW.DriveEngaged = (byte)(newSpeed == 0f ? 0 : 1);
            speed.ValueRW.DesiredSpeed = math.lerp(speed.ValueRW.DesiredSpeed, newSpeed, speed.ValueRW.Damping);

            var newSteeringAngle = x * steering.ValueRW.MaxSteeringAngle;
            steering.ValueRW.DesiredSteeringAngle = math.lerp(steering.ValueRW.DesiredSteeringAngle,
                newSteeringAngle, steering.ValueRW.Damping);
        }
    }
}



struct VehicleInput : IComponentData
{
    public float2 Looking;
    public float2 Steering;
    public float Throttle;
    public int Change; // positive to change to a subsequent vehicle, negative to change to a previous one
}
