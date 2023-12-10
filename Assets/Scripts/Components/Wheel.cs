using UnityEngine;

public class Wheel : MonoBehaviour
{
    public bool CanDrive => canDrive;
    public bool CanSteer => canSteer;
    public bool IsGrounded => isGrounded;
    public float Spring => spring;
    public float Damper => damper;
    public float TireMass => tireMass;
    public float TireRadius => tireRadius;
    public GameObject GraphicsRepresentation => graphicsRepresentation;

    [SerializeField] private float tireRadius = 0.5f;
    [SerializeField] private float spring = 30000;
    [SerializeField] private float damper = 2000;
    [SerializeField] private float tireMass = 20;
    [SerializeField] private bool canDrive = false;
    [SerializeField] private bool canSteer = false;
    [SerializeField] private GameObject graphicsRepresentation;

    private bool isGrounded = false;

}
