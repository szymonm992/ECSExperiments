
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;


[UpdateInGroup(typeof(PhysicsSimulationGroup))]
public partial struct PlayerMovementSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
        
        foreach (var (inputs, properties) in SystemAPI.Query<RefRO<EntityInputsData>, RefRO<VehicleEntityProperties>>())
        {
            var wheelRaycastJob = new DriveJob
            {
                PhysicsWorld = physicsWorld,
            };

            state.Dependency = wheelRaycastJob.Schedule(state.Dependency);
        }
    }

    [BurstCompile]
    [WithAll(typeof(Simulate))]
    public partial struct DriveJob : IJobEntity
    {
        public PhysicsWorld PhysicsWorld;

        private void Execute(in VehicleEntityProperties Properties, in EntityInputsData Inputs)
        {
            bool isInputPositive = Inputs.Vertical.IsNumberPositive();
            var rigidbodyIndex = PhysicsWorld.GetRigidBodyIndex(Properties.VehicleEntity);

            // Calculate the impulse power based on input
            float impulsePower = Inputs.Vertical *
                (isInputPositive ? Properties.VehicleMaximumForwardSpeed : Properties.VehicleMaximumBackwardSpeed);

            // Define the point where the impulse is applied (e.g., center of mass)
            float3 impulsePoint = PhysicsWorld.Bodies[rigidbodyIndex].WorldFromBody.pos;

            // Convert impulse power to a direction and magnitude (e.g., forward direction)
            float3 impulseDirection = new float3(0, 0, 1);
            float3 impulse = impulsePower * impulseDirection;

            PhysicsWorld.ApplyImpulse(rigidbodyIndex, impulse, impulsePoint);
        }
    }
}
