using UnityEngine;
using Unity.Entities;

public class WheelMono : MonoBehaviour
{
    public float Spring => spring;
    public float Damper => damper;
    public float Mass => mass;
    public float Radius => radius;
    public float Thickness => thickness;
    public float RestLength => restLength;

    [SerializeField] private float spring;
    [SerializeField] private float damper;
    [SerializeField] private float mass;
    [SerializeField] private float radius;
    [SerializeField] private float thickness;
    [SerializeField] private float restLength;

    public class Baker : Baker<WheelMono>
    {
        public override void Bake(WheelMono authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(entity, new WheelProperties()
            {
                Spring = authoring.Spring,
                Damper = authoring.Damper,
                Mass = authoring.Mass,
                Radius = authoring.Radius,
                Thickness = authoring.Thickness,
                Travel = authoring.RestLength,
            });
        }
    }
}

