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
    if (IsWireframePass)
    {
        return float4(0.0f, 0.0f, 0.0f, 1.0f); // black
    }

    // 0~2 0=Walkable, 1=Blocked, 2=hover
    if (input.colorType == 0) return float4(1.0f, 1.0f, 1.0f, 1.0f);
    if (input.colorType == 1) return float4(1.0f, 0.3f, 0.3f, 1.0f);
    if (input.colorType == 2) return float4(0.3f, 1.0f, 0.3f, 1.0f);

    // 3: A* - red
    if (input.colorType == 3) return float4(0.9f, 0.05f, 0.05f, 1.0f);

    // 10~17: FlowField chunk color
    if (input.colorType == 10) return float4(0.3f, 0.5f, 1.0f, 1.0f);  // blue
    if (input.colorType == 11) return float4(1.0f, 0.6f, 0.2f, 1.0f);  // orange
    if (input.colorType == 12) return float4(0.7f, 0.4f, 1.0f, 1.0f);  // purple
    if (input.colorType == 13) return float4(0.2f, 0.9f, 0.9f, 1.0f);  // green
    if (input.colorType == 14) return float4(1.0f, 0.9f, 0.3f, 1.0f);  // yellow
    if (input.colorType == 15) return float4(1.0f, 0.4f, 0.7f, 1.0f);  // pink
    if (input.colorType == 16) return float4(0.5f, 0.8f, 0.4f, 1.0f);  // lite green
    if (input.colorType == 17) return float4(0.6f, 0.6f, 0.6f, 1.0f);  // grey

    return float4(1.0f, 1.0f, 0.0f, 1.0f);  // UB - yellow
}
