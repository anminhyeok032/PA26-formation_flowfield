cbuffer ColorConstants : register(b1)
{
    float4 Color;
};

float4 main() : SV_Target
{
    return Color;
}