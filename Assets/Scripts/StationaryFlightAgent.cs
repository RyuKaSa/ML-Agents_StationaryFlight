using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class StationaryFlightAgent : Agent
{
    [SerializeField] private DronePhysics drone;
    [SerializeField] private Rigidbody rb;
    [SerializeField] private Transform spawnPoint;

    [SerializeField, Range(1f, 20f)] private float rayDistance;

    private Vector3 initialPosition;
    private Quaternion initialRotation;

    private void Start()
    {
        // Capture the initial position/rotation once at the beginning
        initialPosition = spawnPoint.position;
        initialRotation = spawnPoint.rotation;
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log("Episode started!");
        
        // Reset drone position/rotation relative to spawn
        transform.position = initialPosition;
        transform.rotation = initialRotation;
        
        // Reset physics
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // fighting chance on init
        drone.SetMotorThrust(0.5f, 0.5f, 0.5f, 0.5f);
        
        // Reset motors
        drone.SetMotorThrust(0f, 0f, 0f, 0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Debug.Log("Collecting observations...");
        // Self state (9 floats total)
        sensor.AddObservation(rb.velocity);             // 3
        sensor.AddObservation(rb.angularVelocity);      // 3
        sensor.AddObservation(transform.up);            // 3
        
        // Raycasts handled
        // 6 cardinal directions
        sensor.AddObservation(Raycast(Vector3.forward));
        sensor.AddObservation(Raycast(Vector3.back));
        sensor.AddObservation(Raycast(Vector3.left));
        sensor.AddObservation(Raycast(Vector3.right));
        sensor.AddObservation(Raycast(Vector3.up));
        sensor.AddObservation(Raycast(Vector3.down));
        
        // 8 corners
        sensor.AddObservation(Raycast(new Vector3( 1,  1,  1).normalized));  // front-right-up
        sensor.AddObservation(Raycast(new Vector3(-1,  1,  1).normalized));  // front-left-up
        sensor.AddObservation(Raycast(new Vector3( 1, -1,  1).normalized));  // front-right-down
        sensor.AddObservation(Raycast(new Vector3(-1, -1,  1).normalized));  // front-left-down
        sensor.AddObservation(Raycast(new Vector3( 1,  1, -1).normalized));  // back-right-up
        sensor.AddObservation(Raycast(new Vector3(-1,  1, -1).normalized));  // back-left-up
        sensor.AddObservation(Raycast(new Vector3( 1, -1, -1).normalized));  // back-right-down
        sensor.AddObservation(Raycast(new Vector3(-1, -1, -1).normalized));  // back-left-down
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        Debug.Log($"Actions received: {actions.ContinuousActions[0]}, {actions.ContinuousActions[1]}, {actions.ContinuousActions[2]}, {actions.ContinuousActions[3]}");

        // Remap -1,1 → 0,1 since ML gives -1,1 and thrust is clamped 0,1
        float m0 = (actions.ContinuousActions[0] + 1f) / 2f;
        float m1 = (actions.ContinuousActions[1] + 1f) / 2f;
        float m2 = (actions.ContinuousActions[2] + 1f) / 2f;
        float m3 = (actions.ContinuousActions[3] + 1f) / 2f;
        
        drone.SetMotorThrust(m0, m1, m2, m3);
        
        // 1. Reward staying upright
        float uprightness = Vector3.Dot(transform.up, Vector3.up);
        AddReward(uprightness * 0.01f);  // Max +0.01 when perfectly upright
        
        // 2. Penalize movement (we want stationary)
        float velocityPenalty = -rb.velocity.magnitude * 0.005f;
        float angularPenalty = -rb.angularVelocity.magnitude * 0.005f;
        AddReward(velocityPenalty + angularPenalty);
        
        // 3. Reward staying near spawn position
        float distanceFromSpawn = Vector3.Distance(transform.position, initialPosition);
        AddReward(-distanceFromSpawn * 0.01f);
        
        // 4. Small time reward
        AddReward(+0.100f);
    }

    // use a single box, and as long as in box, trigger on, is fine
    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<Wall>(out Wall wall))
        {
            Debug.Log("-1 hit wall");
            SetReward(-1f);
            EndEpisode();
        }
    }

    private float Raycast(Vector3 direction)
    {
        if (Physics.Raycast(transform.position, direction, out RaycastHit hit, rayDistance))
        {
            return hit.distance / rayDistance;
        }
        return 1f;
    }
}