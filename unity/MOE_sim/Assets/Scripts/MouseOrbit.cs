using UnityEngine;
using System.Collections;
using UnityEngine.EventSystems;
 
[AddComponentMenu("Camera-Control/Mouse Orbit with zoom")]
public class MouseOrbit : MonoBehaviour {
 
    public Transform target;
    public float distance = 0.75f;
    public float xSpeed = 100.0f;
    public float ySpeed = 100.0f;
 
    public float yMinLimit = -20f;
    public float yMaxLimit = 90f;
 
    public float distanceMin = .001f;
    public float distanceMax = 50f;
 
    new private Rigidbody rigidbody;

    public float scroll_scale = 1;
 
    float x = 0.0f;
    float y = 0.0f;
 
    // Use this for initialization
    void Start () 
    {
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;
 
        rigidbody = GetComponent<Rigidbody>();
 
        // Make the rigid body not change rotation
        if (rigidbody != null)
        {
            rigidbody.freezeRotation = true;
        }
    }
 
    void LateUpdate () 
    {
        if (Input.GetMouseButton(0) && EventSystem.current.currentSelectedGameObject == null) 
        {
            x += Input.GetAxis("Mouse X") * xSpeed * distance * 0.02f;
            y -= Input.GetAxis("Mouse Y") * ySpeed * 0.02f;
        }
        if (EventSystem.current.currentSelectedGameObject != null){
            Debug.Log(EventSystem.current.currentSelectedGameObject.name);
        }
 
        y = ClampAngle(y, yMinLimit, yMaxLimit);

        Quaternion rotation = Quaternion.Euler(y, x, 0);

        distance = Mathf.Clamp(distance - Input.GetAxis("Mouse ScrollWheel")*2*distance*scroll_scale, distanceMin, distanceMax);

        // RaycastHit hit;
        // if (Physics.Linecast (target.position, transform.position, out hit)) 
        // {
        //     distance -=  hit.distance;
        // }
        Vector3 negDistance = new Vector3(0.0f, 0.0f, -distance);
        Vector3 position = rotation * negDistance + target.position;

        transform.rotation = rotation;
        transform.position = position;
    }
 
    public static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360F)
            angle += 360F;
        if (angle > 360F)
            angle -= 360F;
        return Mathf.Clamp(angle, min, max);
    }
}