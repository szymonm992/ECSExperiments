using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;
using ECSExperiment.Wheels;

[BurstCompile]
[UpdateInGroup(typeof(PhysicsSimulationGroup))]
[UpdateAfter(typeof(WheelCastSystem))]
public partial struct VehicleMovementSystem : ISystem, ISystemStartStop
{
    public static readonly float[] a = { 1.0f, -60f, 1688f, 4140f, 6.026f, 0f, -0.3589f, 1f, 0f, -6.111f / 1000f, -3.244f / 100f, 0f, 0f, 0f, 0f };
    public static readonly float[] b = { 1.0f, -60f, 1688f, 4140f, 6.026f, 0f, -0.3589f, 1f, 0f, -6.111f / 1000f, -3.244f / 100f, 0f, 0f, 0f, 0f };

    public void OnStartRunning(ref SystemState state)
    {
        foreach (var wheelProperties in SystemAPI.Query<RefRW<WheelProperties>>())
        {
            const float stepsize = 0.001f;
            const float testnormalforce = 4000f;
            float force = 0;

            for (float slip = stepsize; ; slip += stepsize)
            {
                float newforce = CalcLongitudinalForce(b, testnormalforce, slip);

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
                float newforce = CalcLateralForce(b, testnormalforce, slipangle);

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

            var vehicleProperties = SystemAPI.GetComponent<VehicleProperties>(wheelProperties.ValueRO.VehicleEntity);
            var wheelMassFraction = 1f / vehicleProperties.WheelsAmount;
            wheelProperties.ValueRW.FullCompressionSpringForce = wheelMassFraction * 2.0f * -Physics.gravity.y;
        }
    }


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

                float normalForce = wheelProperties.ValueRW.Compression * wheelProperties.ValueRW.FullCompressionSpringForce;
                wheelProperties.ValueRW.RoadForce = RoadForce(ref state, wheelProperties, hitData, normalForce, wheelGlobalTransform);

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

            wheelProperties.ValueRW.Compression = Mathf.Clamp01(wheelProperties.ValueRW.Compression);
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

    private float3 RoadForce(ref SystemState state, RefRW<WheelProperties> wheelProperties, RefRO<WheelHitData> hitData, float normalForce, LocalTransform worldTransform)
    {
        int slipRes = (int)((100f - math.abs(wheelProperties.ValueRW.AngularVelocity)) / 10f);

        if (slipRes < 1)
        {
            slipRes = 1;
        }

        float invertedSlipRes = (1f / (float)slipRes);

        float totalInertia = wheelProperties.ValueRW.Inertia + wheelProperties.ValueRW.DrivetrainInertia;
        float driveAngularDelta = wheelProperties.ValueRW.DriveTorque * SystemAPI.Time.DeltaTime * invertedSlipRes / totalInertia;
        float totalFrictionTorque = wheelProperties.ValueRW.BrakeFrictionTorque * wheelProperties.ValueRW.Brake + wheelProperties.ValueRW.HandbrakeFrictionTorque *
            wheelProperties.ValueRW.Handbrake + wheelProperties.ValueRW.FrictionTorque + wheelProperties.ValueRW.DriveFrictionTorque;
        float frictionAngularDelta = totalFrictionTorque * SystemAPI.Time.DeltaTime * invertedSlipRes / totalInertia;

        float3 totalForce = float3.zero;
        float newAngle = wheelProperties.ValueRW.MaxSteeringAngle * wheelProperties.ValueRW.Steering;

        for (int i = 0; i < slipRes; i++)
        {
            float f = i * 1f / (float)slipRes;

            wheelProperties.ValueRW.LocalRotation = quaternion.Euler(0, wheelProperties.ValueRO.OldAngle + (newAngle - wheelProperties.ValueRO.OldAngle) * f, 0);
            wheelProperties.ValueRW.InverseLocalRotation = math.inverse(wheelProperties.ValueRO.LocalRotation);

            wheelProperties.ValueRW.SlipRatio = SlipRatio(wheelProperties.ValueRO.WheelVelocity, math.forward(worldTransform.Rotation), wheelProperties.ValueRO.AngularVelocity, wheelProperties.ValueRO.Radius);
            wheelProperties.ValueRW.SlipAngle = SlipAngle((Vector3)wheelProperties.ValueRO.LocalVelocity);

            float3 force = invertedSlipRes * wheelProperties.ValueRW.Grip * CombinedForce(b, normalForce, wheelProperties.ValueRW.SlipRatio, wheelProperties.ValueRW.SlipAngle, wheelProperties.ValueRW.MaxSlip, wheelProperties.ValueRW.MaxAngle,
                wheelProperties.ValueRW.LocalVelocity, hitData.ValueRO.Normal);

            float3 worldForce = math.mul(wheelProperties.ValueRO.LocalRotation, force);
            wheelProperties.ValueRW.AngularVelocity -= (force.z * wheelProperties.ValueRW.Radius * SystemAPI.Time.DeltaTime) / totalInertia;
            wheelProperties.ValueRW.AngularVelocity += driveAngularDelta;

            if (math.abs(wheelProperties.ValueRW.AngularVelocity) > frictionAngularDelta)
            {
                wheelProperties.ValueRW.AngularVelocity -= frictionAngularDelta * math.sign(wheelProperties.ValueRW.AngularVelocity);
            }
            else
            {
                wheelProperties.ValueRW.AngularVelocity = 0;
            }

            float bodyMass = 2000f;
            wheelProperties.ValueRW.WheelVelocity += worldForce * (1 / bodyMass) * SystemAPI.Time.DeltaTime * invertedSlipRes;
            totalForce += worldForce;
        }

        float longitunalSlipVelo = math.abs(wheelProperties.ValueRW.AngularVelocity * wheelProperties.ValueRW.Radius - math.dot(wheelProperties.ValueRW.WheelVelocity, math.forward()));
        float lateralSlipVelo = math.dot(wheelProperties.ValueRW.WheelVelocity, math.right());
        wheelProperties.ValueRW.SlipVelocity = math.sqrt(longitunalSlipVelo * longitunalSlipVelo + lateralSlipVelo * lateralSlipVelo);

        wheelProperties.ValueRW.OldAngle = newAngle;
        return totalForce;
    }

    private float SlipRatio(float3 wheelVelo, float3 forward, float angularVelocity, float radius)
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

    private float SlipAngle(Vector3 localVelo)
    {
        const float fullAngleVelo = 2.0f;

        Vector3 wheelMotionDirection = localVelo;
        wheelMotionDirection.y = 0f;

        if (wheelMotionDirection.sqrMagnitude < math.EPSILON)
        {
            return 0;
        }

        float sinSlipAngle = wheelMotionDirection.normalized.x;
        math.clamp(sinSlipAngle, -1f, 1f);

        float damping = MathExtensions.Clamp01(localVelo.magnitude / fullAngleVelo);

        return -math.asin(sinSlipAngle) * damping * damping;
    }

    private Vector3 CombinedForce(float[] b, float Fz, float slip, float slipAngle, float maxSlip, float maxAngle, Vector3 localVelo, float3 groundNormal)
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

    [BurstCompile]
    private float CalcLongitudinalForce(float[] pacejkaCoefficientsB, float Fz, float slip)
    {
        Fz *= 0.001f;//convert to kN
        slip *= 100f; //covert to %
        float uP = pacejkaCoefficientsB[1] * Fz + pacejkaCoefficientsB[2];
        float D = uP * Fz;
        float B = ((pacejkaCoefficientsB[3] * Fz + pacejkaCoefficientsB[4]) * math.exp(-pacejkaCoefficientsB[5] * Fz)) / (pacejkaCoefficientsB[0] * uP);
        float S = slip + pacejkaCoefficientsB[9] * Fz + pacejkaCoefficientsB[10];
        float E = pacejkaCoefficientsB[6] * Fz * Fz + pacejkaCoefficientsB[7] * Fz + pacejkaCoefficientsB[8];
        float Fx = D * math.sin(pacejkaCoefficientsB[0] * math.atan(S * B + E * (math.atan(S * B) - S * B)));
        return Fx;
    }

    [BurstCompile]
    private float CalcLateralForce(float[] pacejkaCoefficientsA, float Fz, float slipAngle)
    {
        Fz *= 0.001f;//convert to kN
        slipAngle *= (360f / (2f * math.PI)); //convert angle to deg
        float uP = pacejkaCoefficientsA[1] * Fz + pacejkaCoefficientsA[2];
        float D = uP * Fz;
        float B = (pacejkaCoefficientsA[3] * math.sin(2 * math.atan(Fz / pacejkaCoefficientsA[4]))) / (pacejkaCoefficientsA[0] * uP * Fz);
        float S = slipAngle + pacejkaCoefficientsA[9] * Fz + pacejkaCoefficientsA[10];
        float E = pacejkaCoefficientsA[6] * Fz + pacejkaCoefficientsA[7];
        float Sv = pacejkaCoefficientsA[12] * Fz + pacejkaCoefficientsA[13];
        float Fy = D * math.sin(pacejkaCoefficientsA[0] * math.atan(S * B + E * (math.atan(S * B) - S * B))) + Sv;
        return Fy;
    }

    [BurstCompile]
    private float CalcLongitudinalForceUnit(float[] pacejkaCoefficientsB, float Fz, float slip, float maxSlip)
    {
        return CalcLongitudinalForce(pacejkaCoefficientsB, Fz, slip * maxSlip);
    }

    [BurstCompile]
    private float CalcLateralForceUnit(float[] pacejkaCoefficientsB, float Fz, float slipAngle, float maxAngle)
    {
        return CalcLongitudinalForce(pacejkaCoefficientsB, Fz, slipAngle * maxAngle);
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


