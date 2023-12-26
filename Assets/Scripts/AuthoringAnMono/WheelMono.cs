using UnityEngine;
using Unity.Entities;

public class WheelMono : MonoBehaviour
{
    public float Spring => spring;
    public float Damper => damper;
    public float Mass => mass;

    [SerializeField] private float spring;
    [SerializeField] private float damper;
    [SerializeField] private float mass;

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
            });
        }
    }
}
