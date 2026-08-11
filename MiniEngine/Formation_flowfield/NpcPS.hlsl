struct VSOutput
{
    float4 pos : SV_Position;
};

cbuffer WireConstants : register(b1)
{
    uint IsWireframePass;
};


float4 main(float4 pos : SV_Position, uint colorType : COLOR) : SV_Target
{
    if (IsWireframePass)
        return float4(0, 0, 0, 1);

    if (colorType == 10)   return float4(1.0, 0.4, 0.7, 1.0);   // Player
    if (colorType == 11)   return float4(1.0, 0.9, 0.2, 1.0);   // Agro Range ring
    if (colorType == 5)    return float4(0.3, 1.0, 0.3, 1.0);   // Selected

    if (colorType == 1)    return float4(1.0, 0.85, 0.1, 1.0);  // Alerted - Yellow
    if (colorType == 2)    return float4(1.0, 0.15, 0.15, 1.0); // Chase   - Red
    if (colorType == 3)    return float4(0.55, 0.55, 0.6, 1.0); // Lost    - Grey

    return float4(0.2, 0.5, 1.0, 1.0);                          // Idle    - Blue
}
