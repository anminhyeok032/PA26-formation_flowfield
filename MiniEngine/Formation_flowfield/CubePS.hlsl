struct VSOutput
{
    float4 pos       : SV_Position;
    uint   colorType : COLOR;
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

    // 솔리드 패스는 기존 colorType 색상
    if (input.colorType == 0) return float4(1.0f, 1.0f, 1.0f, 1.0f);
    if (input.colorType == 1) return float4(1.0f, 0.3f, 0.3f, 1.0f);
    if (input.colorType == 2) return float4(0.3f, 1.0f, 0.3f, 1.0f);
    return float4(1.0f, 1.0f, 0.0f, 1.0f);
}