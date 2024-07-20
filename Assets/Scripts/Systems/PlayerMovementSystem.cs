using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using ECSExperiment.Wheels;
using UnityEngine;

[BurstCompile]
[UpdateInGroup(typeof(PhysicsSimulationGroup))]
[UpdateAfter(typeof(WheelCastSystem))]
public partial struct VehicleMovementSystem : ISystem, ISystemStartStop
{
    public const float KILONEWTONS_COEFFICIENTS = 0.001f;
    public const float FULL_SLIP_VELOCITY = 4f;

    public static readonly float[] PACEJKA_COEFFICIENTS_A = { 1.0f, -60f, 1688f, 4140f, 6.026f, 0f, -0.3589f, 1f, 0f, -6.111f / 1000f, -3.244f / 100f, 0f, 0f, 0f, 0f };
    public static readonly float[] PACEJKA_COEFFICIENTS_B = { 1.0f, -60f, 1688f, 4140f, 6.026f, 0f, -0.3589f, 1f, 0f, -6.111f / 1000f, -3.244f / 100f, 0f, 0f, 0f, 0f };

    [BurstCompile]
    public void OnStartRunning(ref SystemState state)
    {
        foreach (var wheelProperties in SystemAPI.Query<RefRW<WheelProperties>>())
        {
            const float stepsize = 0.001f;
            const float testNormalForce = 4000f;
            float force = 0;

            for (float slip = stepsize; ; slip += stepsize)
            {
                float newforce = CalculateLongitudinalForce(PACEJKA_COEFFICIENTS_B, testNormalForce, slip);

                if (force < newforce)
                {
                    force = newforce;
                }
                else
                {
                    wheelProperties.ValueRW.MaxSlip = slip - stepsize;
                    break;
                }
            }

            force = 0;

            for (float slipangle = stepsize; ; slipangle += stepsize)
            {
                float newforce = CalculateLateralForce(PACEJKA_COEFFICIENTS_B, testNormalForce, slipangle);

                if (force < newforce)
                {
                    force = newforce;
                }
                else
                {
                    wheelProperties.ValueRW.MaxAngle = slipangle - stepsize;
                    break;
                }
            }
        }
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        state.Dependency.Complete();
        var physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld;
        state.EntityManager.CompleteDependencyBeforeRW<PhysicsWorldSingleton>();

        foreach (var (wheelProperties, hitData, wheelLocalTransform) in SystemAPI.Query<RefRW<WheelProperties>, RefRO<WheelHitData>, RefRO<LocalTransform>>())
        {
            var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);

            if (wheelProperties.ValueRO.IsGrounded)
            {
                var wheelGlobalTransform = WheelTransformRelativeToRigidbody(wheelLocalTransform.ValueRO, physicsWorld, rigidbodyIndex);

                wheelProperties.ValueRW.Compression = 1.0f - ((hitData.ValueRO.Distance - wheelProperties.ValueRO.Radius) / wheelProperties.ValueRO.SpringLength);
                wheelProperties.ValueRW.WheelVelocity = physicsWorld.GetLinearVelocity(rigidbodyIndex, hitData.ValueRO.WheelCenter);
                wheelProperties.ValueRW.LocalVelocity = math.mul(wheelProperties.ValueRO.InverseLocalRotation, wheelProperties.ValueRW.WheelVelocity);

                wheelProperties.ValueRW.RoadForce = CalculateRoadForce(ref state, wheelProperties, wheelProperties.ValueRO.VerticalForce, hitData.ValueRO.Normal, wheelGlobalTransform);

                RequestForceAccumulation(ref state, wheelProperties.ValueRO.VehicleEntity, wheelProperties.ValueRW.RoadForce, hitData.ValueRO.WheelCenter);
                //physicsWorld.ApplyImpulse(rigidbodyIndex, wheelProperties.ValueRW.roadForce, hitData.ValueRO.WheelCenter);
            }
            else
            {
                wheelProperties.ValueRW.Compression = 0.0f;
                wheelProperties.ValueRW.RoadForce = float3.zero;

                float totalInertia = wheelProperties.ValueRO.Inertia + wheelProperties.ValueRO.DrivetrainInertia;
                float driveAngularDelta = wheelProperties.ValueRO.DriveTorque * SystemAPI.Time.DeltaTime / totalInertia;
                float totalFrictionTorque = wheelProperties.ValueRO.BrakeFrictionTorque * wheelProperties.ValueRO.Brake +
                    wheelProperties.ValueRO.HandbrakeFrictionTorque * wheelProperties.ValueRO.Handbrake + wheelProperties.ValueRO.FrictionTorque + wheelProperties.ValueRO.DriveFrictionTorque;
                float frictionAngularDelta = totalFrictionTorque * SystemAPI.Time.DeltaTime / totalInertia;

                wheelProperties.ValueRW.AngularVelocity += driveAngularDelta;

                if (math.abs(wheelProperties.ValueRW.AngularVelocity) > frictionAngularDelta)
                {
                    wheelProperties.ValueRW.AngularVelocity -= frictionAngularDelta * math.sign(wheelProperties.ValueRW.AngularVelocity);
                }
                else
                {
                    wheelProperties.ValueRW.AngularVelocity = 0;
                }

                wheelProperties.ValueRW.SlipRatio = 0;
                wheelProperties.ValueRW.SlipVelocity = 0;
            }

            wheelProperties.ValueRW.Compression = MathExtensions.Clamp01(wheelProperties.ValueRW.Compression);
        }
    }

    [BurstCompile]
    private LocalTransform WheelTransformRelativeToRigidbody(LocalTransform wheelLocalTransform, PhysicsWorld physicsWorld, int rigidbodyIndex)
    {
        if (!IsRigidbodyIndexValid(rigidbodyIndex, physicsWorld))
        {
            return default;
        }

        var vehicleRigidbodyTransform = LocalTransform.FromMatrix(float4x4.TRS(
                    PhysicsWorldExtensions.GetPosition(physicsWorld, rigidbodyIndex),
                    PhysicsWorldExtensions.GetRotation(physicsWorld, rigidbodyIndex),
                    new float3(1f, 1f, 1f)
                ));

        return vehicleRigidbodyTransform.TransformTransform(wheelLocalTransform);
    }

    [BurstCompile]
    private bool IsRigidbodyIndexValid(int rigidbodyIndex, PhysicsWorld physicsWorld)
    {
        return rigidbodyIndex > -1 && rigidbodyIndex < physicsWorld.NumDynamicBodies;
    }

    [BurstCompile]
    private float3 CalculateRoadForce(ref SystemState state, RefRW<WheelProperties> wheelProperties, float normalForce, float3 groundNormal, LocalTransform worldTransform)
    {
        int slipRes = (int)((100f - math.abs(wheelProperties.ValueRO.AngularVelocity)) / 10f);

        if (slipRes < 1)
        {
            slipRes = 1;
        }

        float invertedSlipRes = 1f / (float)slipRes;
        float totalInertia = wheelProperties.ValueRO.Inertia + wheelProperties.ValueRO.DrivetrainInertia;
        float driveAngularDelta = wheelProperties.ValueRO.DriveTorque * SystemAPI.Time.DeltaTime * invertedSlipRes / totalInertia;
        float totalFrictionTorque = wheelProperties.ValueRO.BrakeFrictionTorque * wheelProperties.ValueRO.Brake + wheelProperties.ValueRO.HandbrakeFrictionTorque *
            wheelProperties.ValueRO.Handbrake + wheelProperties.ValueRO.FrictionTorque + wheelProperties.ValueRO.DriveFrictionTorque;
        float frictionAngularDelta = totalFrictionTorque * SystemAPI.Time.DeltaTime * invertedSlipRes / totalInertia;

        float3 totalForce = float3.zero;
        float newAngle = wheelProperties.ValueRO.MaxSteeringAngle * wheelProperties.ValueRO.Steering;

        for (int i = 0; i < slipRes; i++)
        {
            float f = i * 1f / (float)slipRes;

            wheelProperties.ValueRW.LocalRotation = quaternion.Euler(0, wheelProperties.ValueRO.OldAngle + (newAngle - wheelProperties.ValueRO.OldAngle) * f, 0);
            wheelProperties.ValueRW.InverseLocalRotation = math.inverse(wheelProperties.ValueRO.LocalRotation);

            wheelProperties.ValueRW.SlipRatio = CalculateSlipRatio(wheelProperties.ValueRO.WheelVelocity, math.forward(worldTransform.Rotation), wheelProperties.ValueRO.AngularVelocity, wheelProperties.ValueRO.Radius);
            wheelProperties.ValueRW.SlipAngle = CalculateSlipAngle(wheelProperties.ValueRO.LocalVelocity);

            float3 localForce = invertedSlipRes * wheelProperties.ValueRO.Grip * CalculateCombinedForce(PACEJKA_COEFFICIENTS_B, normalForce, wheelProperties.ValueRO.SlipRatio, wheelProperties.ValueRO.SlipAngle, wheelProperties.ValueRO.MaxSlip, wheelProperties.ValueRO.MaxAngle,
                wheelProperties.ValueRO.LocalVelocity, groundNormal);

            float3 worldForce = math.mul(wheelProperties.ValueRO.LocalRotation, localForce);
            wheelProperties.ValueRW.AngularVelocity -= (localForce.z * wheelProperties.ValueRO.Radius * SystemAPI.Time.DeltaTime) / totalInertia;
            wheelProperties.ValueRW.AngularVelocity += driveAngularDelta;

            if (math.abs(wheelProperties.ValueRO.AngularVelocity) > frictionAngularDelta)
            {
                wheelProperties.ValueRW.AngularVelocity -= frictionAngularDelta * math.sign(wheelProperties.ValueRO.AngularVelocity);
            }
            else
            {
                wheelProperties.ValueRW.AngularVelocity = 0;
            }

            float bodyMass = 1170f;
            wheelProperties.ValueRW.WheelVelocity += worldForce * (1 / bodyMass) * SystemAPI.Time.DeltaTime * invertedSlipRes;
            totalForce += worldForce;
        }

        float longitunalSlipVelo = math.abs(wheelProperties.ValueRO.AngularVelocity * wheelProperties.ValueRO.Radius - math.dot(wheelProperties.ValueRO.WheelVelocity, math.forward()));
        float lateralSlipVelo = math.dot(wheelProperties.ValueRO.WheelVelocity, math.right());

        wheelProperties.ValueRW.SlipVelocity = math.sqrt(longitunalSlipVelo * longitunalSlipVelo + lateralSlipVelo * lateralSlipVelo);
        wheelProperties.ValueRW.OldAngle = newAngle;

        return totalForce;
    }

    [BurstCompile]
    private float CalculateSlipRatio(float3 wheelVelo, float3 forward, float angularVelocity, float radius)
    {
        float wheelRoadVelo = math.dot(wheelVelo, forward);

        if (wheelRoadVelo == 0)
        {
            return 0;
        }

        float absRoadVelo = math.abs(wheelRoadVelo);
        float damping = MathExtensions.Clamp01(absRoadVelo / FULL_SLIP_VELOCITY);

        float wheelTireVelo = angularVelocity * radius;
        return (wheelTireVelo - wheelRoadVelo) / absRoadVelo * damping;
    }

    [BurstCompile]
    private float CalculateSlipAngle(float3 localVelocity)
    {
        const float fullAngleVelo = 2.0f;

        float3 wheelMotionDirection = localVelocity;
        wheelMotionDirection.y = 0f;

        if (math.lengthsq(wheelMotionDirection) < math.EPSILON)
        {
            return 0;
        }

        float sinSlipAngle = math.normalize(wheelMotionDirection).x;
        math.clamp(sinSlipAngle, -1f, 1f);

        float damping = MathExtensions.Clamp01(math.length(localVelocity) / fullAngleVelo);

        return -math.asin(sinSlipAngle) * damping * damping;
    }

    [BurstCompile]
    private float3 CalculateCombinedForce(float[] pacejkaCoefficientsB, float verticalForceInNewtons, float slip, float slipAngle, float maxSlip, float maxAngle, float3 localVelocity, float3 groundNormal)
    {
        float unitSlip = slip / maxSlip;
        float unitAngle = slipAngle / maxAngle;
        float p = math.sqrt(unitSlip * unitSlip + unitAngle * unitAngle);

        if (p > math.EPSILON)
        {
            if (slip < -0.8f)
            {
                return -math.normalize(localVelocity) * (math.abs(unitAngle / p * CalculateLateralForceUnit(pacejkaCoefficientsB, verticalForceInNewtons, p, maxAngle))
                    + math.abs(unitSlip / p * CalculateLongitudinalForceUnit(pacejkaCoefficientsB, verticalForceInNewtons, p, maxSlip)));
            }
            else
            {
                var forward = new float3(0, -groundNormal.z, groundNormal.y);
                return math.right() * unitAngle / p * CalculateLateralForceUnit(pacejkaCoefficientsB, verticalForceInNewtons, p, maxAngle) + forward * unitSlip / p * CalculateLongitudinalForceUnit(pacejkaCoefficientsB, verticalForceInNewtons, p, maxSlip);
            }
        }
        else
        {
            return float3.zero;
        }
    }

    [BurstCompile]
    private float CalculateLongitudinalForce(float[] pacejkaCoefficientsB, float verticalForceInNewtons, float slip)
    {
        verticalForceInNewtons *= KILONEWTONS_COEFFICIENTS;
        slip *= 100f; //covert to %
        float uP = pacejkaCoefficientsB[1] * verticalForceInNewtons + pacejkaCoefficientsB[2];
        float D = uP * verticalForceInNewtons;
        float B = ((pacejkaCoefficientsB[3] * verticalForceInNewtons + pacejkaCoefficientsB[4]) * math.exp(-pacejkaCoefficientsB[5] * verticalForceInNewtons)) / (pacejkaCoefficientsB[0] * uP);
        float S = slip + pacejkaCoefficientsB[9] * verticalForceInNewtons + pacejkaCoefficientsB[10];
        float E = pacejkaCoefficientsB[6] * verticalForceInNewtons * verticalForceInNewtons + pacejkaCoefficientsB[7] * verticalForceInNewtons + pacejkaCoefficientsB[8];
        float Fx = D * math.sin(pacejkaCoefficientsB[0] * math.atan(S * B + E * (math.atan(S * B) - S * B)));
        return Fx;
    }

    [BurstCompile]
    private float CalculateLateralForce(float[] pacejkaCoefficientsA, float verticalForceInNewtons, float slipAngle)
    {
        verticalForceInNewtons *= KILONEWTONS_COEFFICIENTS;
        slipAngle = math.degrees(slipAngle);
        float uP = pacejkaCoefficientsA[1] * verticalForceInNewtons + pacejkaCoefficientsA[2];
        float D = uP * verticalForceInNewtons;
        float B = (pacejkaCoefficientsA[3] * math.sin(2 * math.atan(verticalForceInNewtons / pacejkaCoefficientsA[4]))) / (pacejkaCoefficientsA[0] * uP * verticalForceInNewtons);
        float S = slipAngle + pacejkaCoefficientsA[9] * verticalForceInNewtons + pacejkaCoefficientsA[10];
        float E = pacejkaCoefficientsA[6] * verticalForceInNewtons + pacejkaCoefficientsA[7];
        float Sv = pacejkaCoefficientsA[12] * verticalForceInNewtons + pacejkaCoefficientsA[13];
        float Fy = D * math.sin(pacejkaCoefficientsA[0] * math.atan(S * B + E * (math.atan(S * B) - S * B))) + Sv;
        return Fy;
    }

    [BurstCompile]
    private float CalculateLongitudinalForceUnit(float[] pacejkaCoefficientsB, float verticalForceInNewtons, float slip, float maxSlip)
    {
        return CalculateLongitudinalForce(pacejkaCoefficientsB, verticalForceInNewtons, slip * maxSlip);
    }

    [BurstCompile]
    private float CalculateLateralForceUnit(float[] pacejkaCoefficientsB, float verticalForceInNewtons, float slipAngle, float maxAngle)
    {
        return CalculateLongitudinalForce(pacejkaCoefficientsB, verticalForceInNewtons, slipAngle * maxAngle);
    }

    [BurstCompile]
    private void RequestForceAccumulation(ref SystemState state, Entity bufferEntity, float3 appliedForce, float3 appliedForcePoint)
    {
        var vehicleForceAccumulationBuffer = SystemAPI.GetBuffer<ForceAccumulationBufferElement>(bufferEntity);
        vehicleForceAccumulationBuffer.Add(new()
        {
            force = appliedForce,
            point = appliedForcePoint,
        });
    }

    [BurstCompile]
    public void OnStopRunning(ref SystemState _)
    {
    }
}


