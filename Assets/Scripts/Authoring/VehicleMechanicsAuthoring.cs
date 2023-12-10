using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Entities;
using Unity.Physics.Authoring;
using UnityEngine;

[RequireComponent(typeof(PhysicsBodyAuthoring))]
public class VehicleMechanicsAuthoring : MonoBehaviour
{
    [SerializeField] private Wheel[] Wheels;

    public class Baker : Baker<VehicleMechanicsAuthoring>
    {
        public override void Bake(VehicleMechanicsAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(entity, new VehicleMechanicsForBaking
            {
                Wheels = GetWheelInfo(authoring.Wheels, Allocator.Temp),

                SteeringWheels = WheelsToNativeArray
                (authoring.Wheels.Where(wheel => wheel.CanSteer).ToArray(), Allocator.Temp),

                DriveWheels = WheelsToNativeArray
                (authoring.Wheels.Where(wheel => wheel.CanDrive).ToArray(), Allocator.Temp),
            });
        }

        private NativeArray<Entity> WheelsToNativeArray(Wheel[] wheels, Allocator allocator)
        {
            if (wheels == null)
            {
                return default;
            }

            var array = new NativeArray<Entity>(wheels.Length, allocator);
            for (int i = 0; i < wheels.Length; i++)
            {
                array[i] = GetEntity(wheels[i], TransformUsageFlags.Dynamic);
            }

            return array;
        }
        private NativeArray<WheelBakingInfo> GetWheelInfo(Wheel[] wheels, Allocator allocator)
        {
            if (wheels == null)
            {
                return default;
            }

            var array = new NativeArray<WheelBakingInfo>(wheels.Length, allocator);
            int i = 0;

            foreach (var wheel in wheels)
            {
                array[i++] = new WheelBakingInfo()
                {
                    Wheel = GetEntity(wheel, TransformUsageFlags.Dynamic),
                    WheelGraphicalRepresentation = GetEntity(wheel.GraphicsRepresentation, TransformUsageFlags.Dynamic),

                    Radius = wheel.TireRadius,
                    Spring = wheel.Spring,
                    Damper = wheel.Damper,

                    CanDrive = wheel.CanDrive,
                    CanSteer = wheel.CanSteer,
                    IsGrounded = wheel.IsGrounded,
                };
            }

            return array;
        }
    }
}

[TemporaryBakingType]
public struct VehicleMechanicsForBaking : IComponentData
{
    public NativeArray<WheelBakingInfo> Wheels;
    public NativeArray<Entity> DriveWheels;
    public NativeArray<Entity> SteeringWheels;
}

public struct WheelBakingInfo
{
    public Entity Wheel;
    public Entity WheelGraphicalRepresentation;

    public float Radius;
    public float Spring;
    public float Damper;

    public bool CanDrive;
    public bool CanSteer;
    public bool IsGrounded;
}

public struct WheelData : IComponentData
{
    public Entity VehicleEntity;    
    public Entity WheelGraphicalRepresentation;
    public bool CanDrive;
    public bool CanSteer;
}
