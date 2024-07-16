using Unity.Collections;
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

    public float compression;
    public float3 wheelVelo;
    public float3 localVelo;
    public float3 roadForce;
    public quaternion inverseLocalRotation;
    public quaternion localRotation;
    public float angularVelocity;
    public float slipRatio;
    public float slipVelo;
    public float slipAngle;

    public float inertia;
    public float massFraction;
    public float driveTorque;
    public float driveFrictionTorque;
    public float brake;
    public float handbrake;
    public float steering;
    public float maxSteeringAngle;
    public float drivetrainInertia;
    public float fullCompressionSpringForce;

    public float grip;
    public float brakeFrictionTorque;
    public float handbrakeFrictionTorque;
    public float frictionTorque;

    public float maxSlip;
    public float maxAngle;
    public float oldAngle;

}
