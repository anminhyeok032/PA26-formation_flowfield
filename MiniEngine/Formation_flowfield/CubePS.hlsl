struct VSOutput
{
    float4 pos       : SV_Position;
    uint   colorType : COLOR;
};

float4 main(VSOutput input) : SV_Target
{
    if (input.colorType == 0) return float4(1.0f, 1.0f, 1.0f, 1.0f); // Èò»ö
    if (input.colorType == 1) return float4(1.0f, 0.3f, 0.3f, 1.0f); // »¡°­
    if (input.colorType == 2) return float4(0.3f, 1.0f, 0.3f, 1.0f); // ÃÊ·Ï
    return float4(1.0f, 1.0f, 0.0f, 1.0f);                           // ³ë¶û (¿¹¿Ü)
}