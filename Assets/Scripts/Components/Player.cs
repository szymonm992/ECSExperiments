using Unity.Entities;
using UnityEngine;

public class Player : MonoBehaviour
{
}

public class PlayerBaker : Baker<Player>
{
    public override void Bake(Player authoring)
    {
        var entity = GetEntity(TransformUsageFlags.Dynamic);
        AddComponent(entity, new EntityInputsData
        {

        });
    }
}