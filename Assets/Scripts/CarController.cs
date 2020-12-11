using UnityEngine;
using System.Collections;

public class CarController : MonoBehaviour
{
	internal enum driveType
	{
		FrontWheelDrive,
		RearWheelDrive,
		AllWheelDrive
	}
	public AIAgent AIAgent;
	public float topSpeed;
	public float accleration;
	public float brakingPower;
	public float driftPower;
	public bool driftAllowed = false;
	[SerializeField] private driveType carDriveType;
	public AnimationCurve gearRatios;
	public AnimationCurve torqueCurve;
	public bool checkSpeeds = true;
	public float[] gearSpeeds;
	public float idleRPM, maxGearChangeRPM, minGearChangeRPM;
	[HideInInspector]public float smoothTime = 0.3f;
	public float finalDriveRatio1, finalDriveRatio2;
	public float speedMultiplier = 2.23694f;
	public float topSpeedDrag = 0.032f, idleDrag = 0.05f, runningDrag = 0.01f;
	public float forwardFrictionSpeedFactor;
	public float baseFwdExtremum = 1, baseFwdAsymptote = 0.5f;
	public float baseSideAsymptote, baseSideExtremum;
	public float driftVelocityFactor;
	public float defaultForwardStiffness;
	public float maxSidewaysStiffness;
	public float maxSidewaysFrictionValue;
	public float turnPower;
	public float revTorquePower;
	public float differentialTorqueDrop;
	[HideInInspector]public float turnRange = 4f;
	[HideInInspector]public float autoStraight = 1;
	[HideInInspector]public float cgy;
	[HideInInspector]public float turnCheckSense = 10000;
	[HideInInspector]public float acc;
	[HideInInspector]public float throttle;
	public Transform resetPoint;
	public Transform[] wheelMesh;
	public WheelCollider[] wheelColliders;
	public GameObject steeringWheel;
	public float maxSteerAngle;
	public float steerRatio;
	public float steerSensitivity;
	public float speedDependencyFactor;
	public float steerAngleLimitingFactor;
	[HideInInspector]public float engineRPM;
	[HideInInspector]public int gearNum;
	[Range (0, 1)] public float _steerHelper;
	[HideInInspector]public float currSpeed;
	[HideInInspector]public float fwdInput, backInput, horizontalInput;
	[HideInInspector]public float traction;
	[HideInInspector]public float slipLimit;
	[HideInInspector]public float headingAngle;
	public float downForce;
	public float criticalDonutSpeed;

	public float driftX { get; private set; }

	private Rigidbody car;
	private float totalTorque;
	private float outputTorque;
	private float wheelRPM;

	public float steerAngle{ get; private set; }

	private float turnAngle;
	private float maxReverseSpeed = 30;
	private float reverseDrag = 0.1f;
	private float local_finalDrive;
	private WheelFrictionCurve fwf, swf;
	private float iRPM;
	private float thrAgg = 0.8f;
	private float currentTorque;
	private float oldRotation;
	private float localSteerHelper;
	private bool drifting;
	private float upClamp = 0;
	private float prevAngularVelocity;
	private float angularAcclY;

	[SerializeField] private WheelEffects[] m_WheelEffects = new WheelEffects[4];

	void Awake ()
	{
		car = GetComponent<Rigidbody> ();
	}

	void FixedUpdate ()
	{
		getInput ();
		adjustFinalDrive ();
		moveCar ();
		steerCar ();
		brakeCar ();
		animateWheels ();
		animateSteeringWheel ();
		getCarSpeed ();
		calcTurnAngle ();
		adjustDrag ();
		adjustForwardFriction ();
		adjustSidewaysFriction ();
		rotationalStabilizer ();
		steerHelper ();
		tractionControl ();
		driftCar ();
		resetCar ();
		CalculateHeadingAngle ();
		CheckForWheelSpin();
	}

	void getInput ()
	{
		fwdInput = 1;
		backInput = 0;
		horizontalInput = AIAgent.SteeringCommand;
	}

	void resetCar ()
	{
		if (Input.GetKeyDown (KeyCode.R)) {
			car.velocity = Vector3.zero;
			transform.position = resetPoint.position;
			transform.forward = resetPoint.forward;
		}
	}

	void moveCar ()
	{
		float leftWheelTorque = 0;
		float rightWheelTorque = 0;
		calcTorque ();
		if (carDriveType == driveType.AllWheelDrive) {
			outputTorque = totalTorque / 4;
			leftWheelTorque = outputTorque * (1 - Mathf.Clamp (differentialTorqueDrop * ((steerAngle < 0) ? -steerAngle : 0), 0, 0.9f));
			rightWheelTorque = outputTorque * (1 - Mathf.Clamp (differentialTorqueDrop * ((steerAngle > 0) ? steerAngle : 0), 0, 0.9f));
			wheelColliders [0].motorTorque = wheelColliders [2].motorTorque = leftWheelTorque;
			wheelColliders [1].motorTorque = wheelColliders [3].motorTorque = rightWheelTorque;
		} else if (carDriveType == driveType.FrontWheelDrive) {
			outputTorque = totalTorque / 2;
			for (int i = 0; i < 2; i++) {
				wheelColliders [i].motorTorque = outputTorque;
			}
		} else {
			outputTorque = totalTorque / 2;
			for (int i = 2; i < 4; i++) {
				wheelColliders [i].motorTorque = outputTorque;
			}
		}
	}

	void steerCar ()
	{
		float x = horizontalInput * (maxSteerAngle - (currSpeed / topSpeed) * steerAngleLimitingFactor);
		float steerSpeed = steerSensitivity + (currSpeed / topSpeed) * speedDependencyFactor;

		steerAngle = Mathf.SmoothStep (steerAngle, x, steerSpeed);

		wheelColliders [0].steerAngle = steerAngle;
		wheelColliders [1].steerAngle = steerAngle;

		if (!isFlying ())
			car.AddRelativeTorque (transform.up * turnPower * currSpeed * horizontalInput);
	}

	void brakeCar ()
	{
		// Hydraulic Disc Brake
		for (int i = 0; i < 4; i++) {
			if (Input.GetKey (KeyCode.X)) {
				wheelColliders [i].brakeTorque = brakingPower;
			} else
				wheelColliders [i].brakeTorque = 0;
		}

		// Emergency Brakes
		if (Input.GetKey (KeyCode.Space))
				wheelColliders [2].brakeTorque = wheelColliders [3].brakeTorque = brakingPower;
	}

	void driftCar ()
	{
		if (currSpeed > 0 && Mathf.Abs (horizontalInput) > 0 && (backInput < 0 || Input.GetKey (KeyCode.Space)) && (!isFlying ())) {
			float localDriftPower = Input.GetKey (KeyCode.Space) ? driftPower : 0.8f * driftPower;
			float torque = Mathf.Clamp (localDriftPower * horizontalInput * currSpeed, -15000, 15000);
			car.AddRelativeTorque (transform.up * torque);
		}
	}

	void adjustFinalDrive ()
	{
		if (gearNum == 1 || gearNum == 4 || gearNum == 5) {
			local_finalDrive = finalDriveRatio1;
		} else {
			local_finalDrive = finalDriveRatio2;
		}
	}

	void calcTorque ()
	{
		acc = (gearNum == 1) ? Mathf.MoveTowards (0, 1 * fwdInput, thrAgg) : accleration;
		throttle = (gearNum == -1) ? backInput : fwdInput;
		shiftGear ();
		getEngineRPM ();
		totalTorque = torqueCurve.Evaluate (engineRPM) * (gearRatios.Evaluate (gearNum)) * local_finalDrive * throttle * acc;
		if (engineRPM >= maxGearChangeRPM)
			totalTorque = 0;
		tractionControl ();
	}

	void shiftGear ()
	{
		if ((gearNum < gearRatios.length - 1 && engineRPM >= maxGearChangeRPM || (gearNum == 0 && (fwdInput > 0 || backInput < 0))) && !isFlying () && checkGearSpeed ()) {
			gearNum++;
		}

		if (gearNum > 1 && engineRPM <= minGearChangeRPM)
			gearNum--;
		if (checkStandStill () && backInput < 0)
			gearNum = -1;
		if (gearNum == -1 && checkStandStill () && fwdInput > 0)
			gearNum = 1;
	}

	bool checkGearSpeed ()
	{
		if (gearNum != -1) {
			if (checkSpeeds) {
				return currSpeed >= gearSpeeds [gearNum - 1];
			} else
				return true;
		} else
			return false;
	}

	void idlingRPM ()
	{
		iRPM = (gearNum > 1) ? 0 : idleRPM;
	}

	void getEngineRPM ()
	{
		idlingRPM ();
		getWheelRPM ();
		float velocity = 0.0f;
		engineRPM = Mathf.SmoothDamp (engineRPM, iRPM + (Mathf.Abs (wheelRPM) * local_finalDrive * gearRatios.Evaluate (gearNum)), ref velocity, smoothTime);
	}

	void getWheelRPM ()
	{
		float sum = 0;
		int c = 0;
		for (int i = 0; i < 4; i++) {
			if (wheelColliders [i].isGrounded) {
				sum += wheelColliders [i].rpm;
				c++;
			}
		}
		wheelRPM = (c != 0) ? sum / c : 0;
	}

	void getCarSpeed ()
	{
		currSpeed = Vector3.Dot (transform.forward.normalized, car.velocity);
		currSpeed *= speedMultiplier;
		currSpeed = Mathf.Round (currSpeed);
	}

	void animateWheels ()
	{
		Vector3 wheelPosition = Vector3.zero;
		Quaternion wheelRotation = Quaternion.identity;

		for (int i = 0; i < 4; i++) {
			wheelColliders [i].GetWorldPose (out wheelPosition, out wheelRotation);
			wheelMesh [i].position = wheelPosition;
			wheelMesh [i].rotation = wheelRotation;
		}
	}

	void animateSteeringWheel ()
	{
		steeringWheel.transform.localEulerAngles = new Vector3(steeringWheel.transform.localEulerAngles.x, steeringWheel.transform.localEulerAngles.y, steerAngle*steerRatio);
	}

	void adjustDrag ()
	{
		if (currSpeed >= topSpeed)
			car.drag = topSpeedDrag;
		else if (outputTorque == 0)
			car.drag = idleDrag;
		else if (currSpeed >= maxReverseSpeed && gearNum == -1 && wheelRPM <= 0)
			car.drag = reverseDrag;
		else {
			car.drag = runningDrag;
		}
	}

	bool isFlying ()
	{
		if (!wheelColliders [0].isGrounded && !wheelColliders [1].isGrounded && !wheelColliders [2].isGrounded && !wheelColliders [3].isGrounded) {
			return true;
		} else
			return false;
	}

	bool checkStandStill ()
	{
		if (currSpeed == 0) {
			return true;
		} else {
			return false;
		}
	}

	void calcTurnAngle ()
	{
		Vector3 flatForward = transform.forward;
		flatForward.y = 0;
		if (flatForward.sqrMagnitude > 0) {
			flatForward.Normalize ();
			Vector3 localFlatForward = transform.InverseTransformDirection (flatForward);
			turnAngle = Mathf.Atan2 (localFlatForward.x, localFlatForward.z) * turnCheckSense;
		}
	}

	bool isTurning ()
	{
		if (turnAngle > -turnRange && turnAngle < turnRange)
			return false;
		else
			return true;
	}

	void adjustForwardFriction ()
	{
		fwf = wheelColliders [0].forwardFriction;
		fwf.extremumValue = baseFwdExtremum + ((currSpeed <= 0) ? 0 : currSpeed / topSpeed) * forwardFrictionSpeedFactor;
		fwf.asymptoteValue = baseFwdAsymptote + ((currSpeed <= 0) ? 0 : currSpeed / topSpeed) * forwardFrictionSpeedFactor;

		fwf.extremumValue = Mathf.Clamp (fwf.extremumValue, baseFwdExtremum, 5);
		fwf.asymptoteValue = Mathf.Clamp (fwf.asymptoteValue, baseFwdAsymptote, 5);

		for (int i = 0; i < 4; i++) {
			wheelColliders [i].forwardFriction = fwf;
		}
	}

	void adjustSidewaysFriction ()
	{
		upClamp = Mathf.SmoothStep (upClamp, maxSidewaysFrictionValue, 0.2f);
		driftX = Mathf.Abs (transform.InverseTransformVector (car.velocity).x);
		float driftFactor = driftX * driftVelocityFactor;

		swf = wheelColliders [0].sidewaysFriction;

		float x = baseSideAsymptote + driftFactor;
		float y = baseSideExtremum + driftFactor;

		if (Mathf.Abs (currSpeed) < criticalDonutSpeed) {
			swf.stiffness = 0.8f;
		} else
			swf.stiffness = maxSidewaysStiffness;

		swf.asymptoteValue = Mathf.Clamp (x, baseSideAsymptote, maxSidewaysFrictionValue);
		swf.extremumValue = Mathf.Clamp (y, baseSideAsymptote, maxSidewaysFrictionValue);

		for (int i = 0; i < 4; i++) {
			wheelColliders [i].sidewaysFriction = swf;
		}
	}

	void steerHelper ()
	{
		localSteerHelper = Mathf.SmoothStep (localSteerHelper, _steerHelper * Mathf.Abs (horizontalInput), 0.1f);
		if (Input.GetKey (KeyCode.S) || Input.GetKey (KeyCode.DownArrow)) {
			_steerHelper *= -1;
		}

		foreach (WheelCollider wc in wheelColliders) {
			WheelHit wheelHit;
			wc.GetGroundHit (out wheelHit);
			if (wheelHit.normal == Vector3.zero)
				return;
		}

		if (Mathf.Abs (oldRotation - transform.eulerAngles.y) < 10) {
			float turnAdjust = (transform.eulerAngles.y - oldRotation) * _steerHelper;
			Quaternion velRotation = Quaternion.AngleAxis (turnAdjust, Vector3.up);
			car.velocity = velRotation * car.velocity;
		}

		oldRotation = transform.eulerAngles.y;
	}

	void addDownForce ()
	{
		car.AddForce (-transform.up * downForce * car.velocity.magnitude);
	}

	void adjustTorque (float forwardSlip)
	{
		if (forwardSlip >= slipLimit && currentTorque >= 0) {
			currentTorque -= 1000 * traction;
		} else {
			currentTorque += 1000 * traction;
			if (currentTorque >= totalTorque) {
				currentTorque = totalTorque;
			}
		}
	}

	void rotationalStabilizer ()
	{
		calcAngularAccl ();

		float reverseTorque = -1 * Mathf.Abs (angularAcclY) * revTorquePower * Mathf.Sign (car.angularVelocity.y) * (currSpeed / topSpeed);

		car.AddRelativeTorque (transform.up * reverseTorque);
	}

	void calcAngularAccl ()
	{
		angularAcclY = (prevAngularVelocity - car.angularVelocity.y) / Time.deltaTime;
		prevAngularVelocity = car.angularVelocity.y;
	}

	void tractionControl ()
	{
		WheelHit wheelHit;

		switch (carDriveType) {
		case driveType.AllWheelDrive:
			foreach (WheelCollider wc in wheelColliders) {
				wc.GetGroundHit (out wheelHit);
				adjustTorque (wheelHit.forwardSlip);
			}

			break;
		case driveType.RearWheelDrive:
			wheelColliders [2].GetGroundHit (out wheelHit);
			adjustTorque (wheelHit.forwardSlip);
			wheelColliders [3].GetGroundHit (out wheelHit);
			adjustTorque (wheelHit.forwardSlip);

			break;
		case driveType.FrontWheelDrive:
			wheelColliders [0].GetGroundHit (out wheelHit);
			adjustTorque (wheelHit.forwardSlip);
			wheelColliders [1].GetGroundHit (out wheelHit);
			adjustTorque (wheelHit.forwardSlip);

			break;
		}
	}

	void CalculateHeadingAngle ()
	{
		headingAngle = Mathf.Clamp (Mathf.Round (Vector3.SignedAngle (transform.forward, Vector3.forward, Vector3.up)), -90, 90);
	}

	// checks if the wheels are spinning and if so does three things
	// 1) emits particles
	// 2) plays tire skidding sounds
	// 3) leaves skidmarks on the ground
	// these effects are controlled through the WheelEffects class
	private void CheckForWheelSpin()
	{
			// loop through all wheels
			for (int i = 0; i < 4; i++)
			{
					WheelHit wheelHit;
					wheelColliders[i].GetGroundHit(out wheelHit);

					// is the tire slipping above the given threshhold
					if (Mathf.Abs(wheelHit.forwardSlip) >= 4*slipLimit || Mathf.Abs(wheelHit.sidewaysSlip) >= 1.2f*slipLimit)
					{
							m_WheelEffects[i].EmitTyreSmoke();

							// avoiding all four tires screeching at the same time
							// if they do it can lead to some strange audio artefacts
							if (!AnySkidSoundPlaying())
							{
									m_WheelEffects[i].PlayAudio();
							}
							continue;
					}

					// if it wasnt slipping stop all the audio
					if (m_WheelEffects[i].PlayingAudio)
					{
							m_WheelEffects[i].StopAudio();
					}
					// end the trail generation
					m_WheelEffects[i].EndSkidTrail();
			}
	}

	private bool AnySkidSoundPlaying()
	{
			for (int i = 0; i < 4; i++)
			{
					if (m_WheelEffects[i].PlayingAudio)
					{
							return true;
					}
			}
			return false;
	}
}
