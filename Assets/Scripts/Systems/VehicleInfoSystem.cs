using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
public partial struct VehicleInfoSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
        var vehicleDriveJob = new CalculateVehicleInfoJob
        {
            PhysicsWorld = physicsWorld,
        };

        state.Dependency = vehicleDriveJob.Schedule(state.Dependency);
    }

    [BurstCompile]
    [WithAll(typeof(Simulate))]
    public partial struct CalculateVehicleInfoJob : IJobEntity
    {
        public PhysicsWorld PhysicsWorld;

        private void Execute(ref VehicleProperties vehicleProperties, in InputsData inputsData)
        {
            var rigidbodyIndex = PhysicsWorld.GetRigidBodyIndex(vehicleProperties.VehicleEntity);
            var currentVelocity = PhysicsWorld.GetLinearVelocity(rigidbodyIndex);
            var currentSpeed = math.length(currentVelocity) * 4f;

            vehicleProperties.CurrentSpeed = currentSpeed;
        }
    }
}

