using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GetPosition : MonoBehaviour
{
    public Transform thisobj;
    // Start is called before the first frame update
    void Start()
    {
        thisobj = this.gameObject.GetComponent<Transform>();
        
        Debug.Log(this.gameObject.name);
        Debug.Log("世界坐标：" + thisobj.position + "世界旋转：" + thisobj.rotation);
        Debug.Log("局部坐标：" + thisobj.localPosition + "局部旋转：" + thisobj.localRotation);
        //Debug.Log("对象网格中心在世界坐标系位置：" + this.GetComponent<MeshRenderer>().bounds.center.ToString("f4"));


        //Vector3 length = this.GetComponent<Collider>().bounds.size;
        //Debug.Log("长度：" + length);
    }


}
