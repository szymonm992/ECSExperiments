using UnityEditor;
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
    public float TireThickness => tireThickness;
    public float SuspensionTravel => suspensionTravel;
    public GameObject GraphicsRepresentation => graphicsRepresentation;

    [SerializeField] private float tireRadius = 0.5f;
    [SerializeField] private float tireThickness = 0.35f;
    [SerializeField] private float spring = 30000;
    [SerializeField] private float damper = 2000;
    [SerializeField] private float tireMass = 20;
    [Range(0.1f, 2f)]
    [SerializeField] protected float suspensionTravel = 0.5f;

    [SerializeField] private bool canDrive = false;
    [SerializeField] private bool canSteer = false;
    [SerializeField] private GameObject graphicsRepresentation;

    [SerializeField] private WheelDebug debugSettings;

    private bool isGrounded = false;
    protected Vector3 tirePosition;

    #if UNITY_EDITOR
    private void OnValidate()
    {
        tirePosition = transform.position;
    }

    private void OnDrawGizmos()
    {

        bool drawCurrently = (debugSettings.DrawGizmos) && (debugSettings.DrawMode == WheelDebug.DebugMode.All)
            || (debugSettings.DrawMode == WheelDebug.DebugMode.EditorOnly && !Application.isPlaying)
            || (debugSettings.DrawMode == WheelDebug.DebugMode.PlaymodeOnly && Application.isPlaying);

        if (drawCurrently && (this.enabled) || (debugSettings.DrawOnDisable && !this.enabled))
        {
            Handles.color = Color.white;
            Handles.DrawWireDisc(tirePosition, transform.right, tireRadius);
            //Handles.DrawWireDisc(tirePosition + (transform.right * tireThickness), transform.right, tireRadius);
        }
    }
    #endif
}
