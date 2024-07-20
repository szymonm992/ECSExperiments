using Unity.Entities;
using Unity.Mathematics;

namespace ECSExperiment.Wheels
{
    public struct DrivetrainData : IComponentData
    {
        public float FinalDriveRatio;
        public float MinRPM;
        public float MaxRPM;
        public float MaxTorque;
        public float TorqueRPM;
        public float MaxPower;
        public float PowerRPM;
        public float EngineInertia;
        public float EngineBaseFriction;
        public float EngineRPMFriction;
        public float3 EngineOrientation;
        public float DifferentialLockCoefficient;
        public float Throttle;
        public float ThrottleInput;
        public bool AutomaticGearbox;
        public int Gear;
        public float CurrentRPM;
        public float SlipRatio;
        public float EngineAngularVelocity;
    }
}