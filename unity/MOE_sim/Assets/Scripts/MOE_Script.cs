using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using System;

using fts;

public class MOE_Script : MonoBehaviour {

    [Header("Positions")]
    public float q0;
    public float q1;
    public float q2;
    public float q3;

    [Header("Inertial Frame References")]
    public GameObject Fx;
    public GameObject J0;
    public GameObject J1;
    public GameObject J2;
    public GameObject J3;

    double[] qs = new double[4];

    //const string import_module = "tuner_moe";
     const string import_module = "virtual_moe";

    static IntPtr nativeLibraryPtr;
 
    delegate void start();
    delegate void stop();

    delegate void get_positions(double[] positions);

    // bool printed = false;

    void Awake()
    {
        if (nativeLibraryPtr != IntPtr.Zero) return;
 
        nativeLibraryPtr = Native.LoadLibrary(import_module);
        if (nativeLibraryPtr == IntPtr.Zero)
        {
            Debug.LogError("Failed to load native library");
        }
    }

    // Use this for initialization
    void Start () {
        // var coating_mat = Resources.Load("Materials/coating.mat", typeof(Material)) as Material;
        // coating_mat.SetColor("_Color",Color.red);
        Native.Invoke<start>(nativeLibraryPtr);
	}

	// Update is called once per frame
	void Update () {
        // Dll.get_positions(qs);
        Native.Invoke<get_positions>(nativeLibraryPtr, qs);
        q0     = (float)qs[0];
        q1     = (float)qs[1];
        q2     = (float)qs[2];
        q3     = (float)qs[3];
        
        J0.transform.localEulerAngles   = new Vector3(0, 0, -q0*Mathf.Rad2Deg);
        J1.transform.localEulerAngles   = new Vector3(-q1*Mathf.Rad2Deg, 0, 0);
        J2.transform.localEulerAngles   = new Vector3(0, -q2*Mathf.Rad2Deg, 0);
        J3.transform.localEulerAngles   = new Vector3(0, 0, -q3*Mathf.Rad2Deg);
    
        if (Input.GetKeyDown(KeyCode.R))
        {
            // Dll.stop();
            // Dll.start();
            Native.Invoke<stop>(nativeLibraryPtr);
            Native.Invoke<start>(nativeLibraryPtr);
        }
    }

    void OnApplicationQuit() {
        // Dll.stop();
        Native.Invoke<stop>(nativeLibraryPtr);
        if (nativeLibraryPtr == IntPtr.Zero) return;
 
        Debug.Log(Native.FreeLibrary(nativeLibraryPtr)
                      ? "Native library successfully unloaded."
                      : "Native library could not be unloaded.");
    }

    // Dll Imports
    // public class Dll {
    //     [DllImport(import_module)] 
    //     public static extern void start();
    //     [DllImport(import_module)] 
    //     public static extern void stop();
    //     // [DllImport(import_module)] 
    //     // public static extern void set_torques(double tau1, double tau2, double tau3, double tau4, double tau5);
    //     // [DllImport(import_module)]
    //     // public static extern void set_positions(double q1, double q2, double q3, double q4, double q5, double q6, double q7, double q8);
    //     // [DllImport(import_module)]
    //     // public static extern void set_velocities(double q1d, double q2d, double q3d, double q4d, double q5d, double q6d, double q7d, double q8d);
    //     [DllImport(import_module)]
    //     public static extern void get_positions(double[] positions);
    // }

    // [PluginAttr("meii_model")]
    // public static class Dll
    // {        
    //     [PluginFunctionAttr("start")]
    //     public static Start start = null;
    //     public delegate void Start();
        
    //     [PluginFunctionAttr("stop")]
    //     public static Stop stop = null;
    //     public delegate void Stop();
        
    //     [PluginFunctionAttr("set_torques")]
    //     public static Set_torques set_torques = null;
    //     public delegate void Set_torques(double tau1, double tau2, double tau3, double tau4);
        
    //     [PluginFunctionAttr("set_positions")]
    //     public static Set_positions set_positions = null;
    //     public delegate void Set_positions(double q1, double q2, double q3, double q4, double q5, double q6, double q7);
        
    //     [PluginFunctionAttr("set_velocities")]
    //     public static Set_velocities set_velocities = null;
    //     public delegate void Set_velocities(double q1d, double q2d, double q3d, double q4d, double q5d, double q6d, double q7d);
        
    //     [PluginFunctionAttr("get_positions")]
    //     public static Get_positions get_positions = null;
    //     public delegate void Get_positions(double[] positions);
    // }
}

