using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;

namespace ECSExperiment.Wheels
{
    [BurstCompile]
    [UpdateAfter(typeof(VehicleMovementSystem))]
    public partial struct DrivetrainSystem : ISystem
    {
        public static readonly float[] gearRatios = new float[] { 3.23f, 2.97f, 2.07f, 1.43f, 1f, 0.84f, 0.56f };

        [BurstCompile]
        public void OnStartRunning(ref SystemState state)
        {
            state.RequireForUpdate<DrivetrainData>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency.Complete();
            var physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld;
            state.EntityManager.CompleteDependencyBeforeRW<PhysicsWorldSingleton>();

            foreach (var wheelProperties in SystemAPI.Query<RefRW<WheelProperties>>())
            {
                var rigidbodyIndex = physicsWorld.GetRigidBodyIndex(wheelProperties.ValueRO.VehicleEntity);
                var driveTrainComponent = SystemAPI.GetComponentLookup<DrivetrainData>().GetRefRW(wheelProperties.ValueRO.VehicleEntity);
                var vehiclePropertiesComponent = SystemAPI.GetComponentLookup<VehicleProperties>().GetRefRW(wheelProperties.ValueRO.VehicleEntity);

                float currentRatio = gearRatios[driveTrainComponent.ValueRO.Gear] * driveTrainComponent.ValueRO.FinalDriveRatio;
                float currentEngineIntertia = driveTrainComponent.ValueRO.EngineInertia * MathExtensions.Sqr(currentRatio);
                float engineFrictionTorque = driveTrainComponent.ValueRO.EngineBaseFriction + driveTrainComponent.ValueRO.CurrentRPM * driveTrainComponent.ValueRO.EngineRPMFriction;
                float engineTorque = (CalcEngineTorque(driveTrainComponent.ValueRO) + math.abs(engineFrictionTorque)) * driveTrainComponent.ValueRO.Throttle;

                driveTrainComponent.ValueRW.SlipRatio = 0f;

                if (currentRatio == 0)
                {
                    // Neutral gear - just rev up engine
                    float engineAngularAcceleration = (engineTorque - engineFrictionTorque) / driveTrainComponent.ValueRO.EngineInertia;
                    driveTrainComponent.ValueRW.EngineAngularVelocity += engineAngularAcceleration * SystemAPI.Time.DeltaTime;

                    physicsWorld.ApplyAngularImpulse(rigidbodyIndex, -driveTrainComponent.ValueRO.EngineOrientation * engineTorque);
                }
                else
                {
                    float drivetrainFraction = 1f / vehiclePropertiesComponent.ValueRO.WheelsAmount;
                    float averageAngularVelo = 0f;

                    if (wheelProperties.ValueRO.CanDrive)
                    {
                        averageAngularVelo += wheelProperties.ValueRO.AngularVelocity * drivetrainFraction;
                    }

                    if (wheelProperties.ValueRO.CanDrive)
                    {
                        float lockingTorque = (averageAngularVelo - wheelProperties.ValueRO.AngularVelocity) * driveTrainComponent.ValueRO.DifferentialLockCoefficient;
                        wheelProperties.ValueRW.DrivetrainInertia = currentEngineIntertia * drivetrainFraction;
                        wheelProperties.ValueRW.DriveFrictionTorque = engineFrictionTorque * math.abs(currentRatio) * drivetrainFraction;
                        wheelProperties.ValueRW.DriveTorque = engineTorque * currentRatio * drivetrainFraction + lockingTorque;
                        driveTrainComponent.ValueRW.SlipRatio += wheelProperties.ValueRO.SlipRatio * drivetrainFraction;
                    }

                    driveTrainComponent.ValueRW.EngineAngularVelocity = averageAngularVelo * currentRatio;
                }

                driveTrainComponent.ValueRW.SlipRatio *= math.sign(currentRatio);
                driveTrainComponent.ValueRW.CurrentRPM = driveTrainComponent.ValueRO.EngineAngularVelocity * (60f / (2f * math.PI));

                // very simple simulation of clutch - just pretend we are at a higher rpm.
                float minClutchRPM = driveTrainComponent.ValueRO.MinRPM;

                if (driveTrainComponent.ValueRO.Gear == 2)
                {
                    minClutchRPM += driveTrainComponent.ValueRO.Throttle * 3000f;
                }
                if (driveTrainComponent.ValueRO.CurrentRPM < minClutchRPM)
                {
                    driveTrainComponent.ValueRW.CurrentRPM = minClutchRPM;
                }

                // Automatic gear shifting. Bases shift points on throttle input and rpm.
                if (driveTrainComponent.ValueRO.AutomaticGearbox)
                {
                    if (driveTrainComponent.ValueRO.CurrentRPM >= driveTrainComponent.ValueRO.MaxRPM * (0.5f + (0.5f * driveTrainComponent.ValueRO.ThrottleInput)))
                    {
                        if (driveTrainComponent.ValueRO.Gear < gearRatios.Length - 1)
                        {
                            driveTrainComponent.ValueRW.Gear++;
                        }
                    }
                    else if (driveTrainComponent.ValueRO.CurrentRPM <= driveTrainComponent.ValueRO.MaxRPM * (0.25f + (0.4f * driveTrainComponent.ValueRO.ThrottleInput)) && driveTrainComponent.ValueRO.Gear > 2)
                    {
                        if (driveTrainComponent.ValueRO.Gear > 0)
                        {
                            driveTrainComponent.ValueRW.Gear--;
                        }
                    }
                    if (driveTrainComponent.ValueRO.ThrottleInput < 0 && driveTrainComponent.ValueRO.CurrentRPM <= driveTrainComponent.ValueRO.MinRPM)
                    {
                        driveTrainComponent.ValueRW.Gear = (driveTrainComponent.ValueRO.Gear == 0 ? 2 : 0);
                    }
                }
            }
        }

        [BurstCompile]
        private float CalcEngineTorque(DrivetrainData drivetrainData)
        {
            float result;

            if (drivetrainData.CurrentRPM < drivetrainData.TorqueRPM)
            {
                result = drivetrainData.MaxTorque * (-MathExtensions.Sqr(drivetrainData.CurrentRPM / drivetrainData.TorqueRPM - 1f) + 1f);
            }
            else
            {
                float maxPowerTorque = drivetrainData.MaxPower / (drivetrainData.PowerRPM * 2f * math.PI / 60f);
                float aproxFactor = (drivetrainData.MaxTorque - maxPowerTorque) / (2f * drivetrainData.TorqueRPM * drivetrainData.PowerRPM
                    - MathExtensions.Sqr(drivetrainData.PowerRPM)
                    - MathExtensions.Sqr(drivetrainData.TorqueRPM));

                float torque = aproxFactor * MathExtensions.Sqr(drivetrainData.CurrentRPM - drivetrainData.TorqueRPM) + drivetrainData.MaxTorque;
                result = torque > 0f ? torque : 0f;
            }
            if (drivetrainData.CurrentRPM > drivetrainData.MaxRPM)
            {
                result *= 1f - ((drivetrainData.CurrentRPM - drivetrainData.MaxRPM) * 0.006f);

                if (result < 0f)
                {
                    result = 0f;
                }
            }
            if (drivetrainData.CurrentRPM < 0f)
            {
                result = 0f;
            }

            return result;
        }

        [BurstCompile]
        public void OnStopRunning(ref SystemState _)
        {
        }
    }
}
