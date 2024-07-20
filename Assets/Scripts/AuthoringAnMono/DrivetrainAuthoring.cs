using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;
namespace ECSExperiment.Wheels
{
    public class DrivetrainAuthoring : MonoBehaviour
    {
        // The gear ratios, including neutral (0) and reverse (negative) gears
        public float[] gearRatios;

        // The final drive ratio, which is multiplied to each gear ratio
        public float finalDriveRatio = 3.23f;

        // The engine's torque curve characteristics. Since actual curves are often hard to come by,
        // we approximate the torque curve from these values instead.

        // powerband RPM range
        public float minRPM = 800;
        public float maxRPM = 2400;

        // engine's maximal torque (in Nm) and RPM.
        public float maxTorque = 664;
        public float torqueRPM = 4000;

        // engine's maximal power (in Watts) and RPM.
        public float maxPower = 317000;
        public float powerRPM = 5000;

        // engine inertia (how fast the engine spins up), in kg * m^2
        public float engineInertia = 0.3f;

        // engine's friction coefficients - these cause the engine to slow down, and cause engine braking.
        // constant friction coefficient
        public float engineBaseFriction = 25f;
        // linear friction coefficient (higher friction when engine spins higher)
        public float engineRPMFriction = 0.02f;

        // Engine orientation (typically either Vector3.forward or Vector3.right). 
        // This determines how the car body moves as the engine revs up.	
        public Vector3 engineOrientation = Vector3.forward;

        // Coefficient determining how muchg torque is transfered between the wheels when they move at 
        // different speeds, to simulate differential locking.
        public float differentialLockCoefficient = 0;

        // inputs
        // engine throttle
        public float throttle = 0;
        // engine throttle without traction control (used for automatic gear shifting)
        public float throttleInput = 0;

        // shift gears automatically?
        public bool automatic = true;


        public class Baker : Baker<DrivetrainAuthoring>
        {
            public override void Bake(DrivetrainAuthoring authoring)
            {
                var entity = GetEntity(authoring.gameObject, TransformUsageFlags.Dynamic);

                AddComponent(entity, new DrivetrainData()
                {
                    AutomaticGearbox = authoring.automatic,
                    DifferentialLockCoefficient = authoring.differentialLockCoefficient,
                    EngineBaseFriction = authoring.engineBaseFriction,
                    EngineInertia = authoring.engineInertia,
                    EngineOrientation = authoring.engineOrientation,
                    EngineRPMFriction = authoring.engineRPMFriction,
                    FinalDriveRatio = authoring.finalDriveRatio,
                    MaxPower = authoring.maxPower,
                    MaxRPM = authoring.maxRPM,
                    MaxTorque = authoring.maxTorque,
                    MinRPM = authoring.minRPM,
                    PowerRPM = authoring.powerRPM,
                    Throttle = authoring.throttle,
                    ThrottleInput = authoring.throttleInput,
                    TorqueRPM = authoring.torqueRPM,
                });
            }
        }
    }
}