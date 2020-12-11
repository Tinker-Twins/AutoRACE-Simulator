using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class BouncerAgent : Agent
{
    [Header("Bouncer Specific")]
    public GameObject target;
    public GameObject bodyObject;
    Rigidbody m_Rb;
    Vector3 m_LookDir;
    public float strength = 10f;
    float m_JumpCooldown;
    int m_NumberJumps = 20;
    int m_JumpLeft = 20;

    EnvironmentParameters m_ResetParams;

    public override void Initialize()
    {
        m_Rb = gameObject.GetComponent<Rigidbody>();
        m_LookDir = Vector3.zero;

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        SetResetParameters();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(gameObject.transform.localPosition);
        sensor.AddObservation(target.transform.localPosition);
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        for (var i = 0; i < vectorAction.Length; i++)
        {
            vectorAction[i] = Mathf.Clamp(vectorAction[i], -1f, 1f);
        }
        var x = vectorAction[0];
        var y = ScaleAction(vectorAction[1], 0, 1);
        var z = vectorAction[2];
        m_Rb.AddForce(new Vector3(x, y + 1, z) * strength);

        AddReward(-0.05f * (
            vectorAction[0] * vectorAction[0] +
            vectorAction[1] * vectorAction[1] +
            vectorAction[2] * vectorAction[2]) / 3f);

        m_LookDir = new Vector3(x, y, z);
    }

    public override void OnEpisodeBegin()
    {
        gameObject.transform.localPosition = new Vector3(
            (1 - 2 * Random.value) * 5, 2, (1 - 2 * Random.value) * 5);
        m_Rb.velocity = default(Vector3);
        var environment = gameObject.transform.parent.gameObject;
        var targets =
            environment.GetComponentsInChildren<BouncerTarget>();
        foreach (var t in targets)
        {
            t.Respawn();
        }
        m_JumpLeft = m_NumberJumps;

        SetResetParameters();
    }


    void FixedUpdate()
    {
        if (Physics.Raycast(transform.position, new Vector3(0f, -1f, 0f), 0.51f) && m_JumpCooldown <= 0f)
        {
            RequestDecision();
            m_JumpLeft -= 1;
            m_JumpCooldown = 0.1f;
            m_Rb.velocity = default(Vector3);
        }

        m_JumpCooldown -= Time.fixedDeltaTime;

        if (gameObject.transform.position.y < -1)
        {
            AddReward(-1);
            EndEpisode();
            return;
        }

        if (gameObject.transform.localPosition.x < -19 || gameObject.transform.localPosition.x > 19
            || gameObject.transform.localPosition.z < -19 || gameObject.transform.localPosition.z > 19)
        {
            AddReward(-1);
            EndEpisode();
            return;
        }
        if (m_JumpLeft == 0)
        {
            EndEpisode();
        }
    }

    public override void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = Input.GetAxis("Horizontal");
        actionsOut[1] = Input.GetKey(KeyCode.Space) ? 1.0f : 0.0f;
        actionsOut[2] = Input.GetAxis("Vertical");
    }

    void Update()
    {
        if (m_LookDir.magnitude > float.Epsilon)
        {
            bodyObject.transform.rotation = Quaternion.Lerp(bodyObject.transform.rotation,
                Quaternion.LookRotation(m_LookDir),
                Time.deltaTime * 10f);
        }
    }

    public void SetTargetScale()
    {
        var targetScale = m_ResetParams.GetWithDefault("target_scale", 1.0f);
        target.transform.localScale = new Vector3(targetScale, targetScale, targetScale);
    }

    public void SetResetParameters()
    {
        SetTargetScale();
    }
}
