#include <metal_stdlib>
using namespace metal;

struct PointVertex {
    float4 clipPosition [[position]];
    float intensity;
};

vertex PointVertex v_plotting(constant float    *xPositions  [[buffer(0)]],
                              constant float    *yPositions  [[buffer(1)]],
                              constant float    *zPositions  [[buffer(2)]],
                              constant uchar    *intensities [[buffer(3)]],
                              constant float4x4 &projection  [[buffer(4)]],
                              uint               vertexID    [[vertex_id]]) {
    PointVertex out;
    out.clipPosition = projection * float4(xPositions[vertexID], yPositions[vertexID], zPositions[vertexID], 1);
    out.intensity = float(intensities[vertexID]) / 255;
    return out;
}

fragment float4 f_plotting(PointVertex in [[stage_in]]) {
    return float4(float3(in.intensity), 1);
}
