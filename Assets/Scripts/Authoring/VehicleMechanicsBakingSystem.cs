using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;

[RequireMatchingQueriesForUpdate]
[UpdateAfter(typeof(EndColliderBakingSystem))]
[UpdateAfter(typeof(PhysicsBodyBakingSystem))]
[WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
partial struct VehicleMechanicsBakingSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var entityCommandBuffer = new EntityCommandBuffer(Allocator.Temp);

        foreach (var (vehicleMechanicsBaking, vehicleEntity)
                 in SystemAPI.Query<RefRO<VehicleMechanicsForBaking>>().WithEntityAccess()
                     .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities))
        {
            foreach (var wheel in vehicleMechanicsBaking.ValueRO.Wheels)
            {
                var wheelEntity = wheel.Wheel;

                RigidTransform worldFromSuspension = wheel.WorldFromSuspension;
                RigidTransform worldFromChassis = wheel.WorldFromChassis;

                var chassisFromSuspension = math.mul(math.inverse(worldFromChassis), worldFromSuspension);

                entityCommandBuffer.AddComponent(wheelEntity, new WheelData
                {
                    VehicleEntity = vehicleEntity,
                    WheelGraphicalRepresentation = wheel.WheelGraphicalRepresentation, 
                    UsedForSteering = (byte)(vehicleMechanicsBaking.ValueRO.SteeringWheels.Contains(wheelEntity) ? 1 : 0),
                    UsedForDriving = (byte)(vehicleMechanicsBaking.ValueRO.DriveWheels.Contains(wheelEntity) ? 1 : 0),
                    ChassisFromSuspension = chassisFromSuspension
                });
            }
        }

        entityCommandBuffer.Playback(state.EntityManager);
    }
}