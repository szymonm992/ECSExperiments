using Unity.Burst;
using Unity.Entities;
using Unity.Physics;
using UnityEngine;

public partial struct PlayerMovementSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {

    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    { 
        PhysicsWorld physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld;
        state.EntityManager.CompleteDependencyBeforeRW<PhysicsWorldSingleton>();
    }
}
