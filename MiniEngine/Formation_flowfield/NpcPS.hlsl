struct VSOutput
{
    float4 pos : SV_Position;
};

cbuffer WireConstants : register(b1)
{
    uint IsWireframePass;
};


float4 main(VSOutput input) : SV_Target
{
    // 와이어프레임 패스면 단색(검정)으로 경계선 표시
    if (IsWireframePass)
    {
        return float4(0.0f, 0.0f, 0.0f, 1.0f); // 검정 경계선
    }

    return float4(0.2f, 0.5f, 1.0f, 1.0f);
}
