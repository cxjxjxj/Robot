using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

public class RunTime : MonoBehaviour
{
    public Stopwatch stopwatch = new Stopwatch();
    // Start is called before the first frame update
    public void RunOn()
    {
        stopwatch.Start(); // 开始监视代码运行时间
    }

    public void RunOff()
    {
        stopwatch.Stop(); // 开始监视代码运行时间
    }

    public void TimeDebug()
    {
        //UnityEngine.Debug.Log( stopwatch.Elapsed); //timespan
        UnityEngine.Debug.Log("求逆所用时间：" + stopwatch.ElapsedMilliseconds + "ms"); //毫秒
        //UnityEngine.Debug.Log(stopwatch.ElapsedTicks);
    }
}
