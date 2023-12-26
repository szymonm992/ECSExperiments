using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
public partial struct PlayerMovementSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

        
        foreach (var (inputs, properties) in SystemAPI.Query<RefRO<EntityInputsData>, RefRW<VehicleEntityProperties>>())
        {
            var vehicleDriveJob = new DriveJob
            {
                PhysicsWorld = physicsWorld,
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true)
            };
   
            state.Dependency = vehicleDriveJob.Schedule(state.Dependency);
        }
    }

    [BurstCompile]
    [WithAll(typeof(Simulate))]
    public partial struct DriveJob : IJobEntity
    {
        [ReadOnly] public ComponentLookup<LocalTransform> LocalTransformLookup;
        public PhysicsWorld PhysicsWorld;

        private void Execute(ref VehicleEntityProperties Properties, in EntityInputsData Inputs)
        {
            bool isInputPositive = Inputs.Vertical.IsNumberPositive();
            var rigidbodyIndex = PhysicsWorld.GetRigidBodyIndex(Properties.VehicleEntity);

            var currentVelocity = PhysicsWorld.GetLinearVelocity(rigidbodyIndex);
            var currentMaxSpeed = isInputPositive ? Properties.VehicleMaximumForwardSpeed : Properties.VehicleMaximumBackwardSpeed;
            var currentSpeed = math.length(currentVelocity);

            if (currentSpeed <= currentMaxSpeed)
            {
                LocalTransform vehicleTransform = LocalTransformLookup[Properties.VehicleEntity];

                float3 worldForwardDirection = new float3(0, 0, 1);
                float3 impulsePoint = PhysicsWorld.Bodies[rigidbodyIndex].WorldFromBody.pos;
                float3 localForwardDirection = math.mul(vehicleTransform.Rotation, worldForwardDirection);

                float impulsePower = Inputs.Vertical * currentMaxSpeed;
                float3 impulse = impulsePower * localForwardDirection;

                PhysicsWorld.ApplyImpulse(rigidbodyIndex, impulse, impulsePoint);

            }

            Properties.CurrentSpeed = currentSpeed;
        }
    }
}


