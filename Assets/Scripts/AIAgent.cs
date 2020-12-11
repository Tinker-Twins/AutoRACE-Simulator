using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class AIAgent : Agent
{
    public CarController carController; // Instance of `CarController` class
    public Transform ResetPoint; // Reset point transform
    public Transform CarTransform; // Vehicle gameobjct transform

    public Text txtLapTime, txtLastLap, txtBestLap, txtLapCount; // GUI elements

    public float SteeringCommand; // Steering command

    private Rigidbody Car; // Vehicle rigidbody
    private int LapCount = 0; // Measure lap count
    private float LapTime = 0; // Measure lap time
    private float BestLapTime = 1e+6f; // Holds best lap time
    private bool FinishLineFlag = false; // Finish line flag
    private bool CheckpointFlag = false; // Checkpoint flag
    private bool CollisionFlag = false; // Collision flag
    private bool LapCompletionFlag = false; // Lap completion flag
    private bool CheckpointPassingFlag = false; // Checkpoint passing flag
    private bool LapTimeReducedFlag = false; // Best lap time flag

    private List<float> PositionX = new List<float>();
		private List<float> PositionZ = new List<float>();
    private List<float> PositionY = new List<float>();
    private List<float> Steering = new List<float>();
    private List<float> Speed = new List<float>();

    private List<float> BestLapPositionX = new List<float>();
		private List<float> BestLapPositionZ = new List<float>();
    private List<float> BestLapPositionY = new List<float>();
    private List<float> BestLapSteering = new List<float>();
    private List<float> BestLapSpeed = new List<float>();

    void OnCollisionEnter(Collision collision)
    {
        CollisionFlag = true; // Collision detected

        PositionX.Clear();
    		PositionZ.Clear();
        PositionY.Clear();
        Steering.Clear();
        Speed.Clear();
    }

    // Reset lap time and update lap count when crossing start line
    private void OnTriggerEnter(Collider collider)
    {
        if (collider.tag == "Finish Line" && !FinishLineFlag)
        {
            // Update only on positive edge of trigger
            LapCompletionFlag = true;
            LapCount += 1;
            if (LapCount < 10) txtLapCount.text = "0" + LapCount.ToString();
            else txtLapCount.text = LapCount.ToString();
            if (LapTime < 10) txtLastLap.text = "0" + LapTime.ToString("f1");
            else txtLastLap.text = LapTime.ToString("f1");
            if (LapTime < BestLapTime)
            {
                LapTimeReducedFlag = true;
                BestLapTime = LapTime;
                if (BestLapTime < 10) txtBestLap.text = "0" + BestLapTime.ToString("f1");
                else txtBestLap.text = BestLapTime.ToString("f1");

                BestLapPositionX = PositionX;
                BestLapPositionZ = PositionZ;
                BestLapPositionY = PositionY;
                BestLapSteering = Steering;
                BestLapSpeed = Speed;
                GenerateLog();
            }
            LapTime = 0;
            FinishLineFlag = true;
            PositionX.Clear();
        		PositionZ.Clear();
            PositionY.Clear();
            Steering.Clear();
            Speed.Clear();
        }
        else if (collider.tag == "Checkpoint" && !CheckpointFlag)
        {
            CheckpointFlag = true;
            CheckpointPassingFlag = true;
        }
    }

    private void OnTriggerExit(Collider collider)
    {
        FinishLineFlag = false;
        CheckpointFlag = false;
    }

    public override void Initialize()
    {
        Car = gameObject.GetComponent<Rigidbody>();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(carController.currSpeed);
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        // DISCRETE ACTION SPACE
        var SteerAction = Mathf.FloorToInt(vectorAction[0]);

        switch (SteerAction)
        {
            case 0:
                SteeringCommand = 0;
                break;
            case 1:
                SteeringCommand = -1;
                break;
            case 2:
                SteeringCommand = 1;
                break;
        }

        // CONTINUOUS ACTION SPACE
        //SteeringCommand = Mathf.Clamp(vectorAction[0], -1, 1);

        // REWARD FUNCTION
        if (CollisionFlag)
        {
            Debug.Log("Agent Collided!");
            SetReward(-100);
            EndEpisode();
        }
        if (CheckpointPassingFlag)
        {
            Debug.Log("Agent Passed Checkpoint!");
            SetReward(+1);
            CheckpointPassingFlag = false;
        }
        if (LapCompletionFlag)
        {
            Debug.Log("Agent Completed Lap!");
            SetReward(+1);
            if (LapTimeReducedFlag)
            {
                Debug.Log("Lap Time Reduced!");
                SetReward(+10);
            }
            LapTimeReducedFlag = false;
            LapCompletionFlag = false;
            EndEpisode();
        }
        else
        {
            SetReward(carController.currSpeed*0.01f);
        }
    }

    public override void OnEpisodeBegin()
    {
        // Reset momentum
        Car.velocity = new Vector3(0,0,0);
        Car.angularVelocity = Vector3.zero;
        // Reset position
        gameObject.transform.position = ResetPoint.position;
        // Reset orientation
        gameObject.transform.rotation = new Quaternion(0f, 0f, 0f, 0f);
        gameObject.transform.Rotate(new Vector3(0, 1, 0), -32);
        // Reset collision flag
        CollisionFlag = false;
        // Reset lap time
        LapTime = 0;
    }

    public override void Heuristic(float[] actionsOut)
    {
        // DISCRETE ACTION SPACE
        if (Input.GetKey(KeyCode.A)) actionsOut[0] = 1;
        else if (Input.GetKey(KeyCode.D)) actionsOut[0] = 2;
        else actionsOut[0] = 0;

        // CONTINUOUS ACTION SPACE
        //actionsOut[0] = Input.GetAxis("Horizontal");
    }

    private void Update()
    {
        // Update lap time on GUI
        if (LapTime < 10) txtLapTime.text = "0" + LapTime.ToString("f1");
        else txtLapTime.text = LapTime.ToString("f1");
    }

    public void FixedUpdate()
    {
        LapTime += Time.fixedDeltaTime; // Update lap time

        PositionX.Add(CarTransform.position.x);
        PositionZ.Add(CarTransform.position.z);
        PositionY.Add(CarTransform.position.y);
        Steering.Add(SteeringCommand);
        Speed.Add(carController.currSpeed);
    }

    public void GenerateLog()
  	{
    		string PositionXArrayString = "Position X Array: ";
    		foreach(var item in BestLapPositionX)
    		{
    			PositionXArrayString += item.ToString() + " ";
    		}
    		Debug.Log(PositionXArrayString);

        string PositionZArrayString = "Position Z Array: ";
    		foreach(var item in BestLapPositionZ)
    		{
    			PositionZArrayString += item.ToString() + " ";
    		}
    		Debug.Log(PositionZArrayString);

        string PositionYArrayString = "Position Y Array: ";
    		foreach(var item in BestLapPositionY)
    		{
    			PositionYArrayString += item.ToString() + " ";
    		}
    		Debug.Log(PositionYArrayString);

        string SteeringArrayString = "Steering Array: ";
    		foreach(var item in BestLapSteering)
    		{
    			SteeringArrayString += item.ToString() + " ";
    		}
    		Debug.Log(SteeringArrayString);

    		string SpeedArrayString = "Speed Array: ";
    		foreach(var item in BestLapSpeed)
    		{
    			SpeedArrayString += item.ToString() + " ";
    		}
    		Debug.Log(SpeedArrayString);

        Debug.Log("Best Lap Time: " + BestLapTime.ToString());
    }
}
