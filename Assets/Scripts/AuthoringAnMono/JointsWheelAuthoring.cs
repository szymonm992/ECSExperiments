using Unity.Entities;
using Unity.Physics;
using UnityEngine;

namespace ECSExperiment.Wheels
{
    public class JointsWheelAuthoring : MonoBehaviour
    {
        public GameObject vehicleEntity;

        public class Baker : Baker<JointsWheelAuthoring>
        {
            public override void Bake(JointsWheelAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                
                AddComponent(entity, new JointWheel());
            }
        }
    }
}
