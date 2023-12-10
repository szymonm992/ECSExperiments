using Unity.Entities;
using Unity.Physics;
using UnityEditor.Search;
using UnityEngine;

public partial class PlayerInputsSystem : SystemBase
{
    private Controls controls = null;

    protected override void OnCreate()
    {
        controls = new Controls();
        controls.Enable();
       
    }

    protected override void OnUpdate()
    {
        foreach(var data in SystemAPI.Query<RefRW<EntityInputsData>>())
        {
            data.ValueRW.Vertical = controls.Player.Move.ReadValue<Vector2>().y;
            data.ValueRW.Horizontal = controls.Player.Move.ReadValue<Vector2>().x;
        }
    }
}
