using ECSExperiment.Wheels;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;
using System.Collections.Generic;

[UpdateInGroup(typeof(PhysicsSimulationGroup))]
[UpdateAfter(typeof(WheelRaycastSystem))]
public partial struct PlayerMovementSystem : ISystem
{
    public static readonly float[] a = { 1.0f, -60f, 1688f, 4140f, 6.026f, 0f, -0.3589f, 1f, 0f, -6.111f / 1000f, -3.244f / 100f, 0f, 0f, 0f, 0f };
    public static readonly float[] b = { 1.0f, -60f, 1688f, 4140f, 6.026f, 0f, -0.3589f, 1f, 0f, -6.111f / 1000f, -3.244f / 100f, 0f, 0f, 0f, 0f };

    public void OnStartRunning(ref SystemState state)
    {/*
        foreach (var wheelProperties in SystemAPI.Query<RefRW<WheelProperties>>())
        {
            const float stepSize = 0.001f;
            const float testNormalForce = 4000f;
            float force = 0;

            for (float slip = stepSize; ; slip += stepSize)
            {
                float newForce = CalcLongitudinalForce(b, testNormalForce, slip);

                if (force < newForce)
                {
                    force = newForce;
                }
                else
                {
                    wheelProperties.ValueRW.maxSlip = slip - stepSize;
                    break;
                }
            }

            force = 0;

            for (float slipAngle = stepSize; ; slipAngle += stepSize)
            {
                float newForce = CalcLateralForce(b, testNormalForce, slipAngle);

                if (force < newForce)
                {
                    force = newForce;
                }
                else
                {
                    wheelProperties.ValueRW.maxAngle = slipAngle - stepSize;
                    break;
                }
            }

            wheelProperties.ValueRW.fullCompressionSpringForce = wheelProperties.ValueRO.massFraction * 2.0f * -Physics.gravity.y;
        }*/
    }

    public void OnUpdate(ref SystemState state)
    {
        state.Dependency.Complete();
        var physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld;
        state.EntityManager.CompleteDependencyBeforeRW<PhysicsWorldSingleton>();

        /*foreach (var (wheelProperties, hitData, wheelTransform) in SystemAPI.Query<RefRW<WheelProperties>, RefRO<WheelHitData>, RefRO<LocalToWorld>>())
        {  
            var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);

            if (wheelProperties.ValueRO.IsGrounded)
            {
                wheelProperties.ValueRW.compression = 1.0f - ((hitData.ValueRO.Distance - wheelProperties.ValueRO.Radius) / wheelProperties.ValueRO.SpringLength);
                wheelProperties.ValueRW.wheelVelo = physicsWorld.GetLinearVelocity(rigidbodyIndex, hitData.ValueRO.WheelCenter);
                wheelProperties.ValueRW.localVelo = math.mul(wheelProperties.ValueRO.inverseLocalRotation, wheelProperties.ValueRW.wheelVelo);

                float normalForce = wheelProperties.ValueRW.compression * wheelProperties.ValueRW.fullCompressionSpringForce;
                wheelProperties.ValueRW.roadForce = RoadForce(ref state, wheelProperties, hitData, normalForce, wheelTransform.ValueRO);

                physicsWorld.ApplyImpulse(rigidbodyIndex, wheelProperties.ValueRW.roadForce, hitData.ValueRO.WheelCenter);
            }
            else
            {
                wheelProperties.ValueRW.compression = 0.0f;
                wheelProperties.ValueRW.roadForce = float3.zero;

                float totalInertia = wheelProperties.ValueRO.inertia + wheelProperties.ValueRO.drivetrainInertia;
                float driveAngularDelta = wheelProperties.ValueRO.driveTorque * SystemAPI.Time.DeltaTime / totalInertia;
                float totalFrictionTorque = wheelProperties.ValueRO.brakeFrictionTorque * wheelProperties.ValueRO.brake +
                    wheelProperties.ValueRO.handbrakeFrictionTorque * wheelProperties.ValueRO.handbrake + wheelProperties.ValueRO.frictionTorque + wheelProperties.ValueRO.driveFrictionTorque;
                float frictionAngularDelta = totalFrictionTorque * SystemAPI.Time.DeltaTime / totalInertia;

                wheelProperties.ValueRW.angularVelocity += driveAngularDelta;

                if (math.abs(wheelProperties.ValueRW.angularVelocity) > frictionAngularDelta)
                {
                    wheelProperties.ValueRW.angularVelocity -= frictionAngularDelta * math.sign(wheelProperties.ValueRW.angularVelocity);
                }
                else
                {
                    wheelProperties.ValueRW.angularVelocity = 0;
                }

                wheelProperties.ValueRW.slipRatio = 0;
                wheelProperties.ValueRW.slipVelo = 0;
            }

            wheelProperties.ValueRW.compression = Mathf.Clamp01(wheelProperties.ValueRW.compression);
            */


        foreach (var (wheelProperties, hitData, wheelLocalTransform) in SystemAPI.Query<RefRW<WheelProperties>, RefRO<WheelHitData>, RefRO<LocalTransform>>())
        {
            var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);

            var vehicleRigidbodyTransform = LocalTransform.FromMatrix(float4x4.TRS(
                    PhysicsWorldExtensions.GetPosition(physicsWorld, rigidbodyIndex),
                    PhysicsWorldExtensions.GetRotation(physicsWorld, rigidbodyIndex),
                    new float3(1f, 1f, 1f)
                ));

            var wheelCastOriginGlobalTransform = vehicleRigidbodyTransform.TransformTransform(wheelLocalTransform.ValueRO);

            float FRICTION_GRIP_FACTOR = 1f;

            float3 localRightDirection = math.mul(wheelCastOriginGlobalTransform.Rotation, math.right());
            float3 localForwardDirection = math.forward(wheelCastOriginGlobalTransform.Rotation);
            float3 springDir = math.mul(wheelCastOriginGlobalTransform.Rotation, math.up());
            var tireVel = physicsWorld.GetLinearVelocity(rigidbodyIndex, hitData.ValueRO.WheelCenter);

            float steeringVel = math.dot(localRightDirection, tireVel);
            float desiredSidewaysVelChange = -steeringVel * FRICTION_GRIP_FACTOR;
            float desiredSidewaysAccel = desiredSidewaysVelChange / SystemAPI.Time.DeltaTime;

            if (wheelProperties.ValueRO.IsGrounded)
            {
                
                physicsWorld.ApplyImpulse(rigidbodyIndex, desiredSidewaysAccel * localRightDirection, hitData.ValueRO.WheelCenter);
            }

            float forwardVel = math.dot(localForwardDirection, tireVel);
            float desiredForwardVelChange = -forwardVel * FRICTION_GRIP_FACTOR;
            float desiredForwardAccel = desiredForwardVelChange / SystemAPI.Time.DeltaTime;

            if (wheelProperties.ValueRO.IsGrounded)
            {
                
                physicsWorld.ApplyImpulse(rigidbodyIndex, desiredForwardAccel * localForwardDirection, hitData.ValueRO.WheelCenter);
            }


            Debug.DrawRay(hitData.ValueRO.HitPoint, localForwardDirection, Color.blue);
            Debug.DrawRay(hitData.ValueRO.HitPoint, localRightDirection, Color.red);
        }

    }
}
/*
    float3 RoadForce(ref SystemState state, RefRW<WheelProperties> wheelProperties, RefRO<WheelHitData> hitData, float normalForce, LocalToWorld worldTransform)
    {
        int slipRes = (int)((100.0f - math.abs(wheelProperties.ValueRW.angularVelocity)) / (10.0f));

        if (slipRes < 1)
        {
            slipRes = 1;
        }

        float invSlipRes = (1.0f / (float)slipRes);

        float totalInertia = wheelProperties.ValueRW.inertia + wheelProperties.ValueRW.drivetrainInertia;
        float driveAngularDelta = wheelProperties.ValueRW.driveTorque * SystemAPI.Time.DeltaTime * invSlipRes / totalInertia;
        float totalFrictionTorque = wheelProperties.ValueRW.brakeFrictionTorque * wheelProperties.ValueRW.brake + wheelProperties.ValueRW.handbrakeFrictionTorque *
            wheelProperties.ValueRW.handbrake + wheelProperties.ValueRW.frictionTorque + wheelProperties.ValueRW.driveFrictionTorque;
        float frictionAngularDelta = totalFrictionTorque * SystemAPI.Time.DeltaTime * invSlipRes / totalInertia;

        float3 totalForce = Vector3.zero;
        float newAngle = wheelProperties.ValueRW.maxSteeringAngle * wheelProperties.ValueRW.steering;

        for (int i = 0; i < slipRes; i++)
        {
            float f = i * 1.0f / (float)slipRes;
            wheelProperties.ValueRW.localRotation = quaternion.Euler(0, wheelProperties.ValueRO.oldAngle + (newAngle - wheelProperties.ValueRO.oldAngle) * f, 0);
            wheelProperties.ValueRW.inverseLocalRotation = (quaternion)Quaternion.Inverse(wheelProperties.ValueRO.localRotation);

            wheelProperties.ValueRW.slipRatio = SlipRatio(wheelProperties.ValueRO.wheelVelo, math.forward(worldTransform.Rotation), wheelProperties.ValueRO.angularVelocity, wheelProperties.ValueRO.Radius);
            wheelProperties.ValueRW.slipAngle = SlipAngle((Vector3)wheelProperties.ValueRO.localVelo);

            float3 force = invSlipRes * wheelProperties.ValueRW.grip * CombinedForce(b, normalForce, wheelProperties.ValueRW.slipRatio, wheelProperties.ValueRW.slipAngle, wheelProperties.ValueRW.maxSlip, wheelProperties.ValueRW.maxAngle, 
                wheelProperties.ValueRW.localVelo, hitData.ValueRO.Normal);

            float3 worldForce = math.mul(wheelProperties.ValueRO.localRotation, force);
            wheelProperties.ValueRW.angularVelocity -= (force.z * wheelProperties.ValueRW.Radius * SystemAPI.Time.DeltaTime) / totalInertia;
            wheelProperties.ValueRW.angularVelocity += driveAngularDelta;

            if (math.abs(wheelProperties.ValueRW.angularVelocity) > frictionAngularDelta)
            {
                wheelProperties.ValueRW.angularVelocity -= frictionAngularDelta * math.sign(wheelProperties.ValueRW.angularVelocity);
            }
            else
            {
                wheelProperties.ValueRW.angularVelocity = 0;
            }

            float bodyMass = 2000f;
            wheelProperties.ValueRW.wheelVelo += worldForce * (1 / bodyMass) * SystemAPI.Time.DeltaTime * invSlipRes;
            totalForce += worldForce;
        }

        float longitunalSlipVelo = math.abs(wheelProperties.ValueRW.angularVelocity * wheelProperties.ValueRW.Radius - math.dot(wheelProperties.ValueRW.wheelVelo, math.forward()));
        float lateralSlipVelo = math.dot(wheelProperties.ValueRW.wheelVelo, math.right());
        wheelProperties.ValueRW.slipVelo = math.sqrt(longitunalSlipVelo * longitunalSlipVelo + lateralSlipVelo * lateralSlipVelo);

        wheelProperties.ValueRW.oldAngle = newAngle;
        return totalForce;
    }

    float SlipRatio(float3 wheelVelo, float3 forward, float angularVelocity, float radius)
    {
        const float fullSlipVelo = 4.0f;

        float wheelRoadVelo = math.dot(wheelVelo, forward);

        if (wheelRoadVelo == 0)
        {
            return 0;
        }

        float absRoadVelo = math.abs(wheelRoadVelo);
        float damping = Mathf.Clamp01(absRoadVelo / fullSlipVelo);

        float wheelTireVelo = angularVelocity * radius;
        return (wheelTireVelo - wheelRoadVelo) / absRoadVelo * damping;
    }

    float SlipAngle(Vector3 localVelo)
    {
        const float fullAngleVelo = 2.0f;

        Vector3 wheelMotionDirection = localVelo;
        wheelMotionDirection.y = 0;

        if (wheelMotionDirection.sqrMagnitude < math.EPSILON)
        {
            return 0;
        }

        float sinSlipAngle = wheelMotionDirection.normalized.x;
        math.clamp(sinSlipAngle, -1, 1); // To avoid precision errors.

        float damping = Mathf.Clamp01(localVelo.magnitude / fullAngleVelo);

        return -math.asin(sinSlipAngle) * damping * damping;
    }

    Vector3 CombinedForce(float[] b, float Fz, float slip, float slipAngle, float maxSlip, float maxAngle, Vector3 localVelo, float3 groundNormal)
    {
        float unitSlip = slip / maxSlip;
        float unitAngle = slipAngle / maxAngle;
        float p = math.sqrt(unitSlip * unitSlip + unitAngle * unitAngle);

        if (p > math.EPSILON)
        {
            if (slip < -0.8f)
            {
                return -localVelo.normalized * (math.abs(unitAngle / p * CalcLateralForceUnit(b, Fz, p, maxAngle)) + math.abs(unitSlip / p * CalcLongitudinalForceUnit(b, Fz, p, maxSlip)));
            }
            else
            {
                Vector3 forward = new Vector3(0, -groundNormal.z, groundNormal.y);
                return Vector3.right * unitAngle / p * CalcLateralForceUnit(b, Fz, p, maxAngle) + forward * unitSlip / p * CalcLongitudinalForceUnit(b, Fz, p, maxSlip);
            }
        }
        else
        {
            return Vector3.zero;
        }
    }

    float CalcLongitudinalForce(float[] b, float Fz, float slip)
    {
        Fz *= 0.001f;//convert to kN
        slip *= 100f; //covert to %
        float uP = b[1] * Fz + b[2];
        float D = uP * Fz;
        float B = ((b[3] * Fz + b[4]) * math.exp(-b[5] * Fz)) / (b[0] * uP);
        float S = slip + b[9] * Fz + b[10];
        float E = b[6] * Fz * Fz + b[7] * Fz + b[8];
        float Fx = D * math.sin(b[0] * math.atan(S * B + E * (math.atan(S * B) - S * B)));
        return Fx;
    }

    float CalcLateralForce(float[] a, float Fz, float slipAngle)
    {
        Fz *= 0.001f;//convert to kN
        slipAngle *= (360f / (2f * Mathf.PI)); //convert angle to deg
        float uP = a[1] * Fz + a[2];
        float D = uP * Fz;
        float B = (a[3] * math.sin(2 * math.atan(Fz / a[4]))) / (a[0] * uP * Fz);
        float S = slipAngle + a[9] * Fz + a[10];
        float E = a[6] * Fz + a[7];
        float Sv = a[12] * Fz + a[13];
        float Fy = D * math.sin(a[0] * math.atan(S * B + E * (math.atan(S * B) - S * B))) + Sv;
        return Fy;
    }

    float CalcLongitudinalForceUnit(float[] b, float Fz, float slip, float maxSlip)
    {
        return CalcLongitudinalForce(b, Fz, slip * maxSlip);
    }

    float CalcLateralForceUnit(float[] b, float Fz, float slipAngle, float maxAngle)
    {
        return CalcLongitudinalForce(b, Fz, slipAngle * maxAngle);
    }

    [BurstCompile]
    public void OnStopRunning(ref SystemState state)
    {
    }*/
    /*
foreach (var (inputs, properties) in SystemAPI.Query<RefRO<InputsData>, RefRO<VehicleProperties>>())
{
   var vehicleDriveJob = new DriveJob
   {
       PhysicsWorld = physicsWorld,
       LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
       InputsData = inputs.ValueRO,
       VehicleProperties = properties.ValueRO,
       FixedTime = SystemAPI.Time.DeltaTime,
   };
   state.Dependency = vehicleDriveJob.Schedule(state.Dependency);
}*/








