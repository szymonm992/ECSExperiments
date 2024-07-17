using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;


public partial struct DrivetrainSystem : ISystem, ISystemStartStop
{
   
    public static readonly float[] gearRatios = new float[] { 3.23f, 2.97f, 2.07f, 1.43f, 1f, 0.84f, 0.56f };

    public void OnStartRunning(ref SystemState state)
    {
        //state.RequireForUpdate<DrivetrainData>();
    }

    public void OnUpdate(ref SystemState state)
    {
        //state.Dependency.Complete();
        //var physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld;
        //state.EntityManager.CompleteDependencyBeforeRW<PhysicsWorldSingleton>();

        //foreach (var (wheelProperties, hitData, wheelTransform, localWheelTransform) in SystemAPI.Query<RefRW<WheelProperties>, RefRO<WheelHitData>, RefRO<LocalToWorld>, RefRW<LocalTransform>>())
        //{
        //    var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);
        //    var driveTrainComponent = SystemAPI.GetComponentLookup<DrivetrainData>().GetRefRW(wheelProperties.ValueRO.VehicleEntity);

        //    if (driveTrainComponent.Equals(null) || driveTrainComponent.Equals(default))
        //    {
        //        return;
        //    }

        //    float ratio = gearRatios[driveTrainComponent.ValueRO.gear] * driveTrainComponent.ValueRO.finalDriveRatio;
        //    float inertia = driveTrainComponent.ValueRO.engineInertia * Sqr(ratio);
        //    float engineFrictionTorque = driveTrainComponent.ValueRO.engineBaseFriction + driveTrainComponent.ValueRO.rpm * driveTrainComponent.ValueRO.engineRPMFriction;
        //    float engineTorque = (CalcEngineTorque(driveTrainComponent.ValueRO) + Mathf.Abs(engineFrictionTorque)) * driveTrainComponent.ValueRO.throttle;
        //    driveTrainComponent.ValueRW.slipRatio = 0.0f;

        //    if (ratio == 0)
        //    {
        //        // Neutral gear - just rev up engine
        //        float engineAngularAcceleration = (engineTorque - engineFrictionTorque) / driveTrainComponent.ValueRO.engineInertia;
        //        driveTrainComponent.ValueRW.engineAngularVelo += engineAngularAcceleration * Time.deltaTime;

        //        physicsWorld.ApplyAngularForce(rigidbodyIndex, -driveTrainComponent.ValueRO.engineOrientation * engineTorque);
        //        //rig.AddTorque(-engineOrientation * engineTorque);
        //    }
        //    else
        //    {
        //        //float drivetrainFraction = 1.0f / poweredWheels.Length;
        //        float drivetrainFraction = 1.0f / 4f;
        //        float averageAngularVelo = 0;

        //        if (wheelProperties.ValueRO.CanDrive)
        //        {
        //            averageAngularVelo += wheelProperties.ValueRO.angularVelocity * drivetrainFraction;
        //        }

        //        // Apply torque to wheels
        //       if (wheelProperties.ValueRO.CanDrive)
        //       { 
        //            float lockingTorque = (averageAngularVelo - wheelProperties.ValueRO.angularVelocity) * driveTrainComponent.ValueRO.differentialLockCoefficient;
        //            wheelProperties.ValueRW.drivetrainInertia = inertia * drivetrainFraction;
        //            wheelProperties.ValueRW.driveFrictionTorque = engineFrictionTorque * Mathf.Abs(ratio) * drivetrainFraction;
        //            wheelProperties.ValueRW.driveTorque = engineTorque * ratio * drivetrainFraction + lockingTorque;

        //            driveTrainComponent.ValueRW.slipRatio += wheelProperties.ValueRO.slipRatio * drivetrainFraction;
        //        }

        //        // update engine angular velo
        //        driveTrainComponent.ValueRW.engineAngularVelo = averageAngularVelo * ratio;
        //    }

        //    // update state
        //    driveTrainComponent.ValueRW.slipRatio *= math.sign(ratio);
        //    driveTrainComponent.ValueRW.rpm = driveTrainComponent.ValueRO.engineAngularVelo * (60.0f / (2 * math.PI));

        //    // very simple simulation of clutch - just pretend we are at a higher rpm.
        //    float minClutchRPM = driveTrainComponent.ValueRO.minRPM;
        //    if (driveTrainComponent.ValueRO.gear == 2)
        //    {
        //        minClutchRPM += driveTrainComponent.ValueRO.throttle * 3000;
        //    }
        //    if (driveTrainComponent.ValueRO.rpm < minClutchRPM)
        //    {
        //        driveTrainComponent.ValueRW.rpm = minClutchRPM;
        //    }

        //    // Automatic gear shifting. Bases shift points on throttle input and rpm.
        //    if (driveTrainComponent.ValueRO.automatic)
        //    {
        //        if (driveTrainComponent.ValueRO.rpm >= driveTrainComponent.ValueRO.maxRPM * (0.5f + (0.5f * driveTrainComponent.ValueRO.throttleInput)))
        //        {
        //            if (driveTrainComponent.ValueRO.gear < gearRatios.Length - 1)
        //            {
        //                driveTrainComponent.ValueRW.gear++;
        //            }
        //        }
        //        else if (driveTrainComponent.ValueRO.rpm <= driveTrainComponent.ValueRO.maxRPM * (0.25f + (0.4f * driveTrainComponent.ValueRO.throttleInput)) && driveTrainComponent.ValueRO.gear > 2)
        //        {
        //            if (driveTrainComponent.ValueRO.gear > 0)
        //            {
        //                driveTrainComponent.ValueRW.gear--;
        //            }
        //        }
        //        if (driveTrainComponent.ValueRO.throttleInput < 0 && driveTrainComponent.ValueRO.rpm <= driveTrainComponent.ValueRO.minRPM)
        //        {
        //            driveTrainComponent.ValueRW.gear = (driveTrainComponent.ValueRO.gear == 0 ? 2 : 0);
        //        }
        //    }
        //}

        //float CalcEngineTorque(DrivetrainData drivetrainData)
        //{
        //    float result;

        //    if (drivetrainData.rpm < drivetrainData.torqueRPM)
        //    {
        //        result = drivetrainData.maxTorque * (-Sqr(drivetrainData.rpm / drivetrainData.torqueRPM - 1) + 1);
        //    }
        //    else
        //    {
        //        float maxPowerTorque = drivetrainData.maxPower / (drivetrainData.powerRPM * 2 * Mathf.PI / 60);
        //        float aproxFactor = (drivetrainData.maxTorque - maxPowerTorque) / (2 * drivetrainData.torqueRPM * drivetrainData.powerRPM 
        //            - Sqr(drivetrainData.powerRPM) 
        //            - Sqr(drivetrainData.torqueRPM));

        //        float torque = aproxFactor * Sqr(drivetrainData.rpm - drivetrainData.torqueRPM) + drivetrainData.maxTorque;
        //        result = torque > 0 ? torque : 0;
        //    }
        //    if (drivetrainData.rpm > drivetrainData.maxRPM)
        //    {
        //        result *= 1 - ((drivetrainData.rpm - drivetrainData.maxRPM) * 0.006f);

        //        if (result < 0)
        //        {
        //            result = 0;
        //        }
        //    }
        //    if (drivetrainData.rpm < 0)
        //    {
        //        result = 0;
        //    }

        //    return result;
        //}

        //float Sqr(float x) { return x * x; }
    }



    public void OnStopRunning(ref SystemState state)
    {
    }
}
