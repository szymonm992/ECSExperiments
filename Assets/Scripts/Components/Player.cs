using Unity.Entities;
using UnityEngine;

public class Player : MonoBehaviour
{
    public float Speed => speed;

    [SerializeField] private float speed;
}

public struct PlayerData : IComponentData
{
    public float Speed;
}

public class PlayerBaker : Baker<Player>
{
    public override void Bake(Player authoring)
    {
        var entity = GetEntity(TransformUsageFlags.Dynamic);
        AddComponent(entity, new PlayerData
        {
            Speed = authoring.Speed,
        });

        AddComponent(entity, new InputsData
        {

        });
    }
}
