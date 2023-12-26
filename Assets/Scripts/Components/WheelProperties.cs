using Unity.Entities;

public struct WheelProperties : IComponentData
{

    public Entity Entity;
    public float Spring;
    public float Damper;
    public float Mass;
    public float Radius;
    public float Thickness;
    public float Travel;
}

public struct Suspension : IComponentData
{
    public float RestLength;
    public float SpringStiffness;
    public float DamperStiffness;
    public float SuspensionForce;
    public float SpringLength;
    public float SpringForce;
    public float DamperForce;

    public void Reset()
    {
        SuspensionForce = 0;
        SpringLength = RestLength;
        SpringForce = 0;
        DamperForce = 0;
    }
}
