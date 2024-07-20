using Unity.Entities;
using Unity.Mathematics;

public struct WheelProperties : IComponentData
{
    public Entity Entity;
    public Entity VehicleEntity;
    public float Spring;
    public float Damper;
    public float Mass;
    public float Radius;
    public float Thickness;
    public float SpringLength;
    public bool IsGrounded;
    public bool CanDrive;
    public WheelSide Side;

    public float Compression;
    public float Inertia;
    public float DriveTorque;
    public float AngularVelocity;
    public float SlipRatio;
    public float SlipVelocity;
    public float SlipAngle;
    public float DriveFrictionTorque;
    public float Brake;
    public float Handbrake;
    public float Steering;
    public float MaxSteeringAngle;
    public float DrivetrainInertia;
    public float Grip;
    public float BrakeFrictionTorque;
    public float HandbrakeFrictionTorque;
    public float FrictionTorque;
    public float MaxSlip;
    public float MaxAngle;
    public float OldAngle;
    public float VerticalForce;
    public float3 WheelVelocity;
    public float3 LocalVelocity;
    public float3 RoadForce;
    public quaternion InverseLocalRotation;
    public quaternion LocalRotation;
}
