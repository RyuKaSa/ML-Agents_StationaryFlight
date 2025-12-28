using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DronePhysics : MonoBehaviour
{

    [Header("References")]
    [SerializeField] private Rigidbody rb;
    [SerializeField] private Transform[] motors = new Transform[4];
    
    [Header("Settings")]
    [SerializeField] private float maxMotorForce = 20f;
    
    // Thrust values (0 to 1)
    private float[] motorThrust = new float[4];

    private void Awake()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        float totalForce = 0f;
        for (int i = 0; i < 4; i++)
        {
            if (motors[i] == null) continue;
            
            Vector3 thrustDir = transform.up;
            float force = motorThrust[i] * maxMotorForce;
            
            rb.AddForceAtPosition(thrustDir * force, motors[i].position, ForceMode.Force);
        }

        Debug.Log($"Total thrust: {totalForce}N, Gravity: {rb.mass * 9.81f}N");
    }

    public void SetMotorThrust(float m0, float m1, float m2, float m3)
    {
        motorThrust[0] = Mathf.Clamp01(m0);
        motorThrust[1] = Mathf.Clamp01(m1);
        motorThrust[2] = Mathf.Clamp01(m2);
        motorThrust[3] = Mathf.Clamp01(m3);
    }

    public void SetMotorThrust(int index, float thrust)
    {
        if (index >= 0 && index < 4)
            motorThrust[index] = Mathf.Clamp01(thrust);
    }

    public float[] GetMotorThrust() => motorThrust;

    private void OnDrawGizmos()
    {
        if (motors == null) return;
        
        for (int i = 0; i < motors.Length; i++)
        {
            if (motors[i] == null) continue;
            
            // Thrust vector
            Gizmos.color = Color.green;
            float thrustVis = Application.isPlaying ? motorThrust[i] * 0.5f : 0.2f;
            Gizmos.DrawLine(motors[i].position, motors[i].position + transform.up * thrustVis);
        }
    }

}
