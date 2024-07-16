using Unity.Entities;
using Unity.Mathematics;

public struct DrivetrainData : IComponentData
{ 
    public float finalDriveRatio;
    public float minRPM;
    public float maxRPM;
    public float maxTorque;
    public float torqueRPM;
    public float maxPower;
    public float powerRPM;
    public float engineInertia;
    public float engineBaseFriction;
    public float engineRPMFriction;	
    public float3 engineOrientation;
    public float differentialLockCoefficient;
    public float throttle;
    public float throttleInput;
    public bool automatic;
    public int gear;
    public float rpm;
    public float slipRatio;
    public float engineAngularVelo;
}
