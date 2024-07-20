using Unity.Entities;
using UnityEngine;
namespace ECSExperiment.Wheels
{
    public class DrivetrainAuthoring : MonoBehaviour
    {
        // The gear ratios, including neutral (0) and reverse (negative) gears
        public float[] gearRatios;

        [SerializeField] private float finalDriveRatio = 3.23f;
        [SerializeField] private float minRPM = 800;
        [SerializeField] private float maxRPM = 2400;
        [SerializeField] private float maxTorque = 664;
        [SerializeField] private float torqueRPM = 4000;
        [SerializeField] private float maxPower = 317000;
        [SerializeField] private float powerRPM = 5000;
        [SerializeField] private float engineInertia = 0.3f;
        [SerializeField] private float engineBaseFriction = 25f;
        [SerializeField] private float engineRPMFriction = 0.02f;	
        [SerializeField] private Vector3 engineOrientation = Vector3.forward;

        public float differentialLockCoefficient = 0;
        public float throttle = 0;
        public float throttleInput = 0;
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