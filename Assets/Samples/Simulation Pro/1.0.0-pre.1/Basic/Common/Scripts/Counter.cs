using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Counter : MonoBehaviour
{
    [SerializeField]
    private int m_Counter = 0;

    void Update()
    {
        if (TryGetComponent<TextMesh>(out TextMesh tm))
        {
            tm.text = m_Counter.ToString();
        }
        m_Counter += 1;
    }
}
