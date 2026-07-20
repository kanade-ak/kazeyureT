Texture2D<float4> source_texture : register(t0);

float4 psmain(float4 position : SV_Position) : SV_Target
{
    float4 color = source_texture.Load(int3(int2(position.xy), 0));
    return color / max(color.a, 1.0f);
}
